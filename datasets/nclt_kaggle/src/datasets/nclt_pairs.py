"""NCLT pairs dataset for place recognition training with triplet sampling"""

from __future__ import annotations

import csv
import logging
from pathlib import Path
from typing import Any, Callable

import numpy as np
import torch
import yaml
from scipy.spatial import KDTree
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)

# NCLT velodyne binary: (x, y, z, intensity, ring)
_VELODYNE_POINT_FIELDS = 5
_BYTES_PER_FLOAT32 = 4


def load_point_cloud(path: Path) -> np.ndarray:
    """load NCLT .bin point cloud (5 float32 fields per point), return (N, 3) XYZ"""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {path}")

    file_size = path.stat().st_size
    point_stride = _VELODYNE_POINT_FIELDS * _BYTES_PER_FLOAT32
    if file_size % point_stride != 0:
        raise ValueError(
            f"File size ({file_size} bytes) is not a multiple of point stride "
            f"({point_stride} bytes): {path}"
        )

    points = np.fromfile(str(path), dtype=np.float32)
    points = points.reshape(-1, _VELODYNE_POINT_FIELDS)
    return points[:, :3].copy()


def pairs_collate_fn(batch: list[dict[str, Any]]) -> dict[str, Any]:
    """collate samples into batched tensors"""
    anchors = torch.stack([s["anchor"] for s in batch], dim=0)
    positives = torch.stack([s["positive"] for s in batch], dim=0)
    negatives = [s["negatives"] for s in batch]
    anchor_poses = torch.stack([s["anchor_pose"] for s in batch], dim=0)
    positive_poses = torch.stack([s["positive_pose"] for s in batch], dim=0)

    return {
        "anchors": anchors,
        "positives": positives,
        "negatives": negatives,
        "anchor_poses": anchor_poses,
        "positive_poses": positive_poses,
    }


class NCLTPairsDataset(Dataset):
    """PyTorch dataset yielding (anchor, positive, negatives) point cloud tuples"""

    _VALID_SPLITS = ("train", "val", "test")

    def __init__(
        self,
        config_path: str | Path,
        split: str = "train",
        transform: Callable[[np.ndarray], np.ndarray] | None = None,
        num_negatives: int = 5,
    ) -> None:
        if split not in self._VALID_SPLITS:
            raise ValueError(
                f"Invalid split '{split}'. Must be one of {self._VALID_SPLITS}."
            )

        self.split = split
        self.transform = transform
        self.num_negatives = num_negatives

        # load config
        config_path = Path(config_path)
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            full_config = yaml.safe_load(f)

        self.config: dict[str, Any] = full_config["nclt"]
        self.positive_threshold: float = float(
            self.config["positive_threshold"]
        )
        self.negative_threshold: float = float(
            self.config["negative_threshold"]
        )
        self.max_points: int = int(
            self.config.get("point_cloud", {}).get("max_points", 50_000)
        )

        # resolve data root
        self.data_root = self._resolve_data_root()
        logger.info("Using data root: %s", self.data_root)

        # sessions for split
        session_key = f"{split}_sessions"
        self.sessions: list[str] = self.config.get(
            session_key, self.config.get("sessions", [])
        )
        if not self.sessions:
            raise ValueError(
                f"No sessions found for split '{split}' in config."
            )
        logger.info(
            "Split '%s', %d session(s): %s",
            split,
            len(self.sessions),
            ", ".join(self.sessions),
        )

        # load poses
        self.poses: dict[str, list[dict[str, Any]]] = {}
        for session in self.sessions:
            self.poses[session] = self._load_session_poses(session)
        total_poses = sum(len(v) for v in self.poses.values())
        logger.info("Loaded %d total poses across all sessions.", total_poses)

        # load or generate pairs
        csv_path = self.data_root / f"{split}.csv"
        if csv_path.exists():
            logger.info("Loading pre-annotated pairs from %s", csv_path)
            self.pairs = self._load_pairs_from_csv(csv_path)
        else:
            logger.info(
                "Pair CSV not found at %s, generating pairs from poses.",
                csv_path,
            )
            self.pairs = self._generate_pairs(self.poses, self.sessions)

        logger.info(
            "NCLTPairsDataset [%s]: %d pairs ready.", split, len(self.pairs)
        )

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int) -> dict[str, Any]:
        anchor_info, positive_info, negative_infos = self.pairs[idx]

        anchor_pc = self._prepare_point_cloud(anchor_info["path"])
        positive_pc = self._prepare_point_cloud(positive_info["path"])

        negative_pcs: list[torch.Tensor] = []
        for neg in negative_infos:
            negative_pcs.append(self._prepare_point_cloud(neg["path"]))

        anchor_pose = torch.tensor(
            [
                anchor_info["x"],
                anchor_info["y"],
                anchor_info["z"],
                anchor_info["roll"],
                anchor_info["pitch"],
                anchor_info["yaw"],
                anchor_info["timestamp"],
            ],
            dtype=torch.float64,
        )
        positive_pose = torch.tensor(
            [
                positive_info["x"],
                positive_info["y"],
                positive_info["z"],
                positive_info["roll"],
                positive_info["pitch"],
                positive_info["yaw"],
                positive_info["timestamp"],
            ],
            dtype=torch.float64,
        )

        return {
            "anchor": anchor_pc,
            "positive": positive_pc,
            "negatives": negative_pcs,
            "anchor_pose": anchor_pose,
            "positive_pose": positive_pose,
        }

    # ------------------------------------------------------------------
    # Pair loading / generation
    # ------------------------------------------------------------------

    def _load_pairs_from_csv(self, csv_path: Path) -> list[tuple]:
        """parse pre-annotated pair CSV"""
        pairs: list[tuple] = []

        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row_idx, row in enumerate(reader):
                try:
                    anchor = self._resolve_pose(
                        row["anchor_session"].strip(),
                        int(row["anchor_timestamp"].strip()),
                    )
                    positive = self._resolve_pose(
                        row["positive_session"].strip(),
                        int(row["positive_timestamp"].strip()),
                    )

                    negatives: list[dict[str, Any]] = []
                    neg_sessions = row["negative_sessions"].strip().split(";")
                    neg_timestamps = (
                        row["negative_timestamps"].strip().split(";")
                    )

                    for ns, nt in zip(neg_sessions, neg_timestamps):
                        ns, nt = ns.strip(), nt.strip()
                        if ns and nt:
                            negatives.append(
                                self._resolve_pose(ns, int(nt))
                            )

                    # pad/truncate negatives
                    negatives = self._adjust_negatives(
                        negatives, anchor, positive
                    )
                    pairs.append((anchor, positive, negatives))

                except (KeyError, ValueError) as exc:
                    logger.warning(
                        "Skipping malformed row %d in %s: %s",
                        row_idx,
                        csv_path,
                        exc,
                    )

        return pairs

    def _generate_pairs(
        self,
        poses: dict[str, list[dict[str, Any]]],
        sessions: list[str],
    ) -> list[tuple]:
        """generate pairs from pose distances (closest positive, random negatives)"""
        all_poses: list[dict[str, Any]] = []
        for session in sessions:
            all_poses.extend(poses[session])

        if len(all_poses) < 2:
            logger.warning("Not enough poses to generate pairs.")
            return []

        coords = np.array([[p["x"], p["y"], p["z"]] for p in all_poses])
        tree = KDTree(coords)

        pairs: list[tuple] = []
        rng = np.random.default_rng(seed=42)

        for anchor_idx, anchor_pose in enumerate(all_poses):
            anchor_xy = coords[anchor_idx]

            # --- Find positives (within positive_threshold) ----------------
            positive_indices = tree.query_ball_point(
                anchor_xy, r=self.positive_threshold
            )
            # exclude anchor
            positive_indices = [
                i for i in positive_indices if i != anchor_idx
            ]
            if not positive_indices:
                continue

            # closest positive
            dists = np.linalg.norm(
                coords[positive_indices] - anchor_xy, axis=1
            )
            best_pos_idx = positive_indices[int(np.argmin(dists))]

            # --- Find negatives (beyond negative_threshold) ----------------
            all_dists = np.linalg.norm(coords - anchor_xy, axis=1)
            negative_mask = all_dists > self.negative_threshold
            negative_indices = np.where(negative_mask)[0]

            if len(negative_indices) < self.num_negatives:
                # not enough negatives
                continue

            chosen_neg_indices = rng.choice(
                negative_indices, size=self.num_negatives, replace=False
            )

            pairs.append(
                (
                    all_poses[anchor_idx],
                    all_poses[best_pos_idx],
                    [all_poses[int(ni)] for ni in chosen_neg_indices],
                )
            )

        logger.info("Generated %d pairs from %d poses.", len(pairs), len(all_poses))
        return pairs

    def _mine_hard_negatives(
        self,
        anchor_descriptor: np.ndarray,
        candidates: list[dict[str, Any]],
        k: int,
    ) -> list[dict[str, Any]]:
        """return up to k negatives closest to anchor in descriptor space"""
        if not candidates:
            return []

        descriptors = np.stack([c["descriptor"] for c in candidates], axis=0)
        distances = np.linalg.norm(
            descriptors - anchor_descriptor[np.newaxis, :], axis=1
        )

        # ascending: smallest distance = hardest negative
        sorted_indices = np.argsort(distances)
        selected = [candidates[int(i)] for i in sorted_indices[:k]]

        logger.debug(
            "Mined %d hard negatives (descriptor dist range: %.4f–%.4f).",
            len(selected),
            float(distances[sorted_indices[0]]),
            float(distances[sorted_indices[min(k, len(sorted_indices)) - 1]]),
        )
        return selected

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _resolve_data_root(self) -> Path:
        """return first existing data root (kaggle_path, then local_path)"""
        for key in ("kaggle_path", "local_path"):
            candidate = Path(self.config.get(key, ""))
            if candidate.exists():
                return candidate.resolve()

        raise FileNotFoundError(
            "Dataset root not found. Checked: "
            f"kaggle_path={self.config.get('kaggle_path')}, "
            f"local_path={self.config.get('local_path')}"
        )

    def _load_session_poses(
        self, session: str
    ) -> list[dict[str, Any]]:
        """load poses from track.csv (auto-detects Kaggle quaternion vs Euler format)"""
        track_path = self.data_root / session / "track.csv"
        if not track_path.exists():
            logger.warning("Track file not found: %s", track_path)
            return []

        # resolve velodyne directory
        velodyne_dir = self.data_root / session / "velodyne_data"
        if not velodyne_dir.exists():
            velodyne_dir = self.data_root / session / "velodyne"

        poses: list[dict[str, Any]] = []

        with open(track_path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)

            is_kaggle = False
            if header and not header[0].replace(".", "", 1).lstrip("-").isdigit():
                col_map = {
                    name.strip().lower(): i for i, name in enumerate(header)
                }
                # Kaggle format has qw column
                is_kaggle = "qw" in col_map
            else:
                col_map = None

            if is_kaggle:
                idx_ts = col_map.get("pointcloud", col_map.get("timestamp", 0))
                idx_tx = col_map["tx"]
                idx_ty = col_map["ty"]
                idx_tz = col_map["tz"]
                idx_qx = col_map["qx"]
                idx_qy = col_map["qy"]
                idx_qz = col_map["qz"]
                idx_qw = col_map["qw"]

                for row in reader:
                    if not row or not row[0].strip():
                        continue
                    pose = self._parse_kaggle_row(
                        row,
                        idx_ts, idx_tx, idx_ty, idx_tz,
                        idx_qx, idx_qy, idx_qz, idx_qw,
                        session, velodyne_dir,
                    )
                    if pose is not None:
                        poses.append(pose)
            else:
                # original format
                if col_map is not None:
                    idx_ts = col_map.get("timestamp", 0)
                    idx_x = col_map.get("x", 1)
                    idx_y = col_map.get("y", 2)
                    idx_z = col_map.get("z", 3)
                    idx_roll = col_map.get("roll", 4)
                    idx_pitch = col_map.get("pitch", 5)
                    idx_yaw = col_map.get("yaw", 6)
                else:
                    idx_ts, idx_x, idx_y, idx_z = 0, 1, 2, 3
                    idx_roll, idx_pitch, idx_yaw = 4, 5, 6
                    if header:
                        poses.append(
                            self._parse_pose_row(
                                header,
                                idx_ts, idx_x, idx_y, idx_z,
                                idx_roll, idx_pitch, idx_yaw,
                                session, velodyne_dir,
                            )
                        )

                for row in reader:
                    if not row or not row[0].strip():
                        continue
                    pose = self._parse_pose_row(
                        row,
                        idx_ts, idx_x, idx_y, idx_z,
                        idx_roll, idx_pitch, idx_yaw,
                        session, velodyne_dir,
                    )
                    if pose is not None:
                        poses.append(pose)

        logger.debug(
            "Session %s: loaded %d poses from %s.", session, len(poses), track_path
        )
        return poses

    @staticmethod
    def _parse_pose_row(
        row: list[str],
        idx_ts: int,
        idx_x: int,
        idx_y: int,
        idx_z: int,
        idx_roll: int,
        idx_pitch: int,
        idx_yaw: int,
        session: str,
        velodyne_dir: Path,
    ) -> dict[str, Any] | None:
        """parse a single row of original-format track.csv"""
        try:
            timestamp = int(float(row[idx_ts].strip()))
            return {
                "timestamp": timestamp,
                "x": float(row[idx_x]),
                "y": float(row[idx_y]),
                "z": float(row[idx_z]),
                "roll": float(row[idx_roll]),
                "pitch": float(row[idx_pitch]),
                "yaw": float(row[idx_yaw]),
                "session": session,
                "path": velodyne_dir / f"{timestamp}.bin",
            }
        except (IndexError, ValueError) as exc:
            logger.debug("Skipping malformed track row: %s (%s)", row, exc)
            return None

    @staticmethod
    def _parse_kaggle_row(
        row: list[str],
        idx_ts: int,
        idx_tx: int, idx_ty: int, idx_tz: int,
        idx_qx: int, idx_qy: int, idx_qz: int, idx_qw: int,
        session: str,
        velodyne_dir: Path,
    ) -> dict[str, Any] | None:
        """parse Kaggle-format row, converting quaternion to Euler angles"""
        try:
            timestamp = int(float(row[idx_ts].strip()))
            tx = float(row[idx_tx])
            ty = float(row[idx_ty])
            tz = float(row[idx_tz])
            qx = float(row[idx_qx])
            qy = float(row[idx_qy])
            qz = float(row[idx_qz])
            qw = float(row[idx_qw])

            # quaternion to Euler (ZYX)
            sinr = 2.0 * (qw * qx + qy * qz)
            cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = np.arctan2(sinr, cosr)

            sinp = 2.0 * (qw * qy - qz * qx)
            sinp = np.clip(sinp, -1.0, 1.0)
            pitch = np.arcsin(sinp)

            siny = 2.0 * (qw * qz + qx * qy)
            cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = np.arctan2(siny, cosy)

            return {
                "timestamp": timestamp,
                "x": tx,
                "y": ty,
                "z": tz,
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
                "session": session,
                "path": velodyne_dir / f"{timestamp}.bin",
            }
        except (IndexError, ValueError) as exc:
            logger.debug("Skipping malformed Kaggle track row: %s (%s)", row, exc)
            return None

    def _resolve_pose(
        self, session: str, timestamp: int
    ) -> dict[str, Any]:
        """look up pose by session and timestamp, fallback to nearest"""
        session_poses = self.poses.get(session, [])
        if not session_poses:
            raise ValueError(f"No poses loaded for session '{session}'.")

        # exact match
        for pose in session_poses:
            if pose["timestamp"] == timestamp:
                return pose

        # nearest timestamp fallback
        timestamps = np.array([p["timestamp"] for p in session_poses])
        nearest_idx = int(np.argmin(np.abs(timestamps - timestamp)))
        logger.debug(
            "Exact timestamp %d not found in session %s; using nearest (%d).",
            timestamp,
            session,
            session_poses[nearest_idx]["timestamp"],
        )
        return session_poses[nearest_idx]

    def _adjust_negatives(
        self,
        negatives: list[dict[str, Any]],
        anchor: dict[str, Any],
        positive: dict[str, Any],
    ) -> list[dict[str, Any]]:
        """pad or truncate negatives to exactly self.num_negatives"""
        if len(negatives) >= self.num_negatives:
            return negatives[: self.num_negatives]

        # need more negatives
        anchor_xy = np.array([anchor["x"], anchor["y"], anchor["z"]])
        existing_timestamps = {n["timestamp"] for n in negatives}
        rng = np.random.default_rng()

        candidates: list[dict[str, Any]] = []
        for session_poses in self.poses.values():
            for pose in session_poses:
                if pose["timestamp"] in existing_timestamps:
                    continue
                dist = float(
                    np.linalg.norm(
                        np.array([pose["x"], pose["y"], pose["z"]]) - anchor_xy
                    )
                )
                if dist > self.negative_threshold:
                    candidates.append(pose)

        needed = self.num_negatives - len(negatives)
        if candidates:
            chosen_indices = rng.choice(
                len(candidates),
                size=min(needed, len(candidates)),
                replace=False,
            )
            negatives.extend([candidates[int(i)] for i in chosen_indices])

        # duplicate to fill if still short
        while len(negatives) < self.num_negatives and negatives:
            negatives.append(negatives[len(negatives) % len(negatives)])

        return negatives[: self.num_negatives]

    def _prepare_point_cloud(self, path: Path) -> torch.Tensor:
        """load, transform, pad/truncate to (max_points, 3) tensor"""
        points = load_point_cloud(path)

        if self.transform is not None:
            points = self.transform(points)

        num_points = points.shape[0]
        if num_points > self.max_points:
            # deterministic stride for eval, random for train
            if self.split == "train":
                indices = np.random.choice(
                    num_points, size=self.max_points, replace=False
                )
            else:
                step = num_points / self.max_points
                indices = np.round(np.arange(self.max_points) * step).astype(
                    np.int64
                )
            points = points[indices]
        elif num_points < self.max_points:
            # pad with zeros
            padding = np.zeros(
                (self.max_points - num_points, 3), dtype=np.float32
            )
            points = np.concatenate([points, padding], axis=0)

        return torch.from_numpy(points.astype(np.float32))
