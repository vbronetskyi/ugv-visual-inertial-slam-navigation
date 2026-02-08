"""NCLT LiDAR dataset loader with Kaggle/local paths, sensor addon support"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np
import torch
import yaml
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    logger.debug("OpenCV not installed, image loading will use PIL or be skipped")


@dataclass
class SampleRecord:
    """metadata for a single dataset sample"""

    session: str
    timestamp: int
    velodyne_path: Path
    image_dir: Path
    pose: np.ndarray


@dataclass
class PointCloudConfig:
    """point cloud preprocessing parameters"""

    max_points: int = 50000
    voxel_size: float = 0.1
    remove_ground: bool = True
    ground_threshold: float = -1.5


@dataclass
class DatasetConfig:
    """parsed dataset configuration from YAML"""

    kaggle_path: str = ""
    local_path: str = ""
    sensors_kaggle_path: str = ""
    sensors_local_path: str = ""
    sessions: list[str] = field(default_factory=list)
    sensor_sessions: list[str] = field(default_factory=list)
    train_sessions: list[str] = field(default_factory=list)
    val_sessions: list[str] = field(default_factory=list)
    test_sessions: list[str] = field(default_factory=list)
    positive_threshold: float = 10.0
    negative_threshold: float = 25.0
    point_cloud: PointCloudConfig = field(default_factory=PointCloudConfig)
    sensors: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> DatasetConfig:
        """create from the 'nclt' section of dataset_config.yaml"""
        pc_raw = raw.get("point_cloud", {})
        pc_config = PointCloudConfig(
            max_points=pc_raw.get("max_points", 50000),
            voxel_size=pc_raw.get("voxel_size", 0.1),
            remove_ground=pc_raw.get("remove_ground", True),
            ground_threshold=pc_raw.get("ground_threshold", -1.5),
        )
        return cls(
            kaggle_path=raw.get("kaggle_path", ""),
            local_path=raw.get("local_path", ""),
            sensors_kaggle_path=raw.get("sensors_kaggle_path", ""),
            sensors_local_path=raw.get("sensors_local_path", ""),
            sessions=raw.get("sessions", []),
            sensor_sessions=raw.get("sensor_sessions", []),
            train_sessions=raw.get("train_sessions", []),
            val_sessions=raw.get("val_sessions", []),
            test_sessions=raw.get("test_sessions", []),
            positive_threshold=raw.get("positive_threshold", 10.0),
            negative_threshold=raw.get("negative_threshold", 25.0),
            point_cloud=pc_config,
            sensors=raw.get("sensors", {}),
        )


class NCLTDataset(Dataset):
    """PyTorch Dataset for NCLT LiDAR place recognition with optional sensor data"""

    VALID_SPLITS: tuple[str, ...] = ("train", "val", "test")

    def __init__(
        self,
        config_path: str | Path,
        split: str = "train",
        transform: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
        load_sensors: bool = False,
        sensors_root: str | Path | None = None,
        sensor_window_ms: int = 50,
    ) -> None:
        super().__init__()

        if split not in self.VALID_SPLITS:
            raise ValueError(
                f"Invalid split '{split}'. Must be one of {self.VALID_SPLITS}."
            )

        self.split = split
        self.transform = transform
        self.config = self._load_config(config_path)
        self.data_root = self._resolve_data_path()
        self.sessions = self._get_split_sessions()
        self.pc_config = self.config.point_cloud
        self.samples: list[SampleRecord] = []

        # sensor addon
        self.load_sensors = load_sensors
        self.sensor_window_ms = sensor_window_ms
        self._sensor_managers: dict[str, Any] = {}

        if self.load_sensors:
            self._sensors_root = self._resolve_sensors_path(sensors_root)
            self._init_sensor_managers()
        else:
            self._sensors_root = None

        self._build_index()

        logger.info(
            "NCLTDataset initialized: split=%s, sessions=%d, samples=%d, "
            "root=%s, sensors=%s",
            self.split,
            len(self.sessions),
            len(self.samples),
            self.data_root,
            self._sensors_root is not None,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict[str, Any]:
        """return sample dict with point_cloud, pose, session, timestamp keys"""
        if idx < 0 or idx >= len(self.samples):
            raise IndexError(
                f"Index {idx} out of range for dataset of size {len(self.samples)}."
            )

        record = self.samples[idx]

        raw_pc = self.load_point_cloud(record.velodyne_path)
        raw_pc = self._preprocess_point_cloud(raw_pc)
        pc_tensor = torch.from_numpy(raw_pc).float()

        sample: dict[str, Any] = {
            "point_cloud": pc_tensor,
            "pose": torch.from_numpy(record.pose).float(),
            "session": record.session,
            "timestamp": record.timestamp,
        }

        # optional image
        image = self._find_and_load_image(record)
        if image is not None:
            sample["image"] = torch.from_numpy(image)

        # optional sensor data
        if self.load_sensors and record.session in self._sensor_managers:
            sensor_data = self._load_sensor_data(record)
            sample.update(sensor_data)

        if self.transform is not None:
            sample = self.transform(sample)

        return sample

    def get_session_dataset(self, session: str) -> NCLTDataset:
        """return a shallow-copy dataset filtered to a single session"""
        if session not in self.sessions:
            raise ValueError(
                f"Session '{session}' not in current split sessions: {self.sessions}"
            )

        subset = object.__new__(NCLTDataset)
        subset.split = self.split
        subset.transform = self.transform
        subset.config = self.config
        subset.data_root = self.data_root
        subset.sessions = [session]
        subset.pc_config = self.pc_config
        subset.samples = [s for s in self.samples if s.session == session]
        subset.load_sensors = self.load_sensors
        subset.sensor_window_ms = self.sensor_window_ms
        subset._sensors_root = self._sensors_root
        subset._sensor_managers = {
            k: v for k, v in self._sensor_managers.items() if k == session
        }

        logger.info(
            "Session subset created: session=%s, samples=%d",
            session,
            len(subset.samples),
        )
        return subset

    # ------------------------------------------------------------------
    # I/O helpers (public, usable as standalone utilities)
    # ------------------------------------------------------------------

    @staticmethod
    def load_point_cloud(path: Path) -> np.ndarray:
        """load Velodyne .bin file, return (N, 4) float32 array [x, y, z, intensity]"""
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Point cloud file not found: {path}")

        file_size = path.stat().st_size
        if file_size == 0:
            logger.warning("Empty point cloud file: %s", path)
            return np.empty((0, 4), dtype=np.float32)

        if file_size % 16 == 0:
            # XYZI format (4 floats per point)
            points = np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)
        elif file_size % 12 == 0:
            # XYZ format (3 floats per point) -- pad with intensity=1.0
            xyz = np.fromfile(str(path), dtype=np.float32).reshape(-1, 3)
            intensity = np.ones((xyz.shape[0], 1), dtype=np.float32)
            points = np.hstack([xyz, intensity])
        else:
            raise ValueError(
                f"Point cloud file size ({file_size} bytes) is not a multiple "
                f"of 12 or 16. File may be corrupt: {path}"
            )

        return points

    @staticmethod
    def load_image(path: Path) -> np.ndarray | None:
        """load image as (H, W, 3) uint8 RGB array, or None if missing"""
        path = Path(path)
        if not path.exists():
            return None

        try:
            if HAS_CV2:
                img = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if img is None:
                    logger.warning("Failed to decode image: %s", path)
                    return None
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                return img
            else:
                from PIL import Image

                img = Image.open(path).convert("RGB")
                return np.asarray(img)
        except Exception:
            logger.warning("Error reading image: %s", path, exc_info=True)
            return None

    @staticmethod
    def load_poses(session_dir: Path) -> dict[int, np.ndarray]:
        """parse track.csv, return {timestamp: (4,4) SE3 matrix}"""
        import csv as csv_mod

        track_path = Path(session_dir) / "track.csv"
        if not track_path.exists():
            raise FileNotFoundError(f"Track file not found: {track_path}")

        # detect format from first line
        with open(track_path, "r", encoding="utf-8") as fh:
            first_line = fh.readline().strip()

        kaggle_headers = {
            "timestamp", "pointcloud", "tx", "ty", "tz",
            "qx", "qy", "qz", "qw", "image",
        }
        is_kaggle = False
        if first_line:
            parts = [p.strip().lower() for p in first_line.split(",")]
            if any(p in kaggle_headers for p in parts):
                is_kaggle = True

        poses: dict[int, np.ndarray] = {}

        if is_kaggle:
            with open(track_path, "r", newline="", encoding="utf-8") as fh:
                reader = csv_mod.DictReader(fh)
                for row in reader:
                    ts_str = (
                        row.get("pointcloud")
                        or row.get("timestamp", "0")
                    )
                    ts = int(float(ts_str))
                    pose = NCLTDataset.quaternion_to_matrix(
                        float(row["tx"]), float(row["ty"]), float(row["tz"]),
                        float(row["qx"]), float(row["qy"]),
                        float(row["qz"]), float(row["qw"]),
                    )
                    poses[ts] = pose
        else:
            try:
                data = np.loadtxt(
                    str(track_path), delimiter=",", dtype=np.float64,
                )
            except Exception as exc:
                raise RuntimeError(
                    f"Failed to parse track file {track_path}: {exc}"
                ) from exc

            if data.ndim == 1:
                data = data.reshape(1, -1)

            if data.shape[1] != 7:
                raise ValueError(
                    f"Expected 7 columns in track.csv, got {data.shape[1]}. "
                    f"File: {track_path}"
                )

            for i in range(data.shape[0]):
                ts = int(data[i, 0])
                x, y, z = data[i, 1], data[i, 2], data[i, 3]
                roll, pitch, yaw = data[i, 4], data[i, 5], data[i, 6]
                poses[ts] = NCLTDataset.pose_to_matrix(
                    x, y, z, roll, pitch, yaw,
                )

        logger.debug("Loaded %d poses from %s", len(poses), track_path)
        return poses

    @staticmethod
    def pose_to_matrix(
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
    ) -> np.ndarray:
        """convert (x, y, z, roll, pitch, yaw) to (4, 4) SE3 matrix (ZYX convention)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float32,
        )

        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    @staticmethod
    def quaternion_to_matrix(
        tx: float, ty: float, tz: float,
        qx: float, qy: float, qz: float, qw: float,
    ) -> np.ndarray:
        """convert quaternion + translation to (4, 4) SE3 matrix"""
        # normalise quaternion to guard against small numeric drift
        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q /= np.linalg.norm(q) + 1e-12

        x, y, z, w = q
        T = np.eye(4, dtype=np.float32)
        T[0, 0] = 1 - 2 * (y * y + z * z)
        T[0, 1] = 2 * (x * y - z * w)
        T[0, 2] = 2 * (x * z + y * w)
        T[1, 0] = 2 * (x * y + z * w)
        T[1, 1] = 1 - 2 * (x * x + z * z)
        T[1, 2] = 2 * (y * z - x * w)
        T[2, 0] = 2 * (x * z - y * w)
        T[2, 1] = 2 * (y * z + x * w)
        T[2, 2] = 1 - 2 * (x * x + y * y)
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz
        return T

    # ------------------------------------------------------------------
    # private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _load_config(config_path: str | Path) -> DatasetConfig:
        """load and validate the YAML configuration file"""
        config_path = Path(config_path)
        if not config_path.exists():
            raise FileNotFoundError(
                f"Dataset config file not found: {config_path}"
            )

        with open(config_path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)

        if "nclt" not in raw:
            raise KeyError(
                "Config file is missing the top-level 'nclt' key. "
                f"Found keys: {list(raw.keys())}"
            )

        return DatasetConfig.from_dict(raw["nclt"])

    def _resolve_data_path(self) -> Path:
        """determine dataset root (Kaggle first, then local)"""
        kaggle_path = Path(self.config.kaggle_path)
        if kaggle_path.exists() and kaggle_path.is_dir():
            logger.info("Using Kaggle data path: %s", kaggle_path)
            return kaggle_path

        local_path = Path(self.config.local_path)
        if local_path.exists() and local_path.is_dir():
            logger.info("Using local data path: %s", local_path)
            return local_path

        raise FileNotFoundError(
            f"Dataset not found at either path.\n"
            f"  Kaggle: {kaggle_path}\n"
            f"  Local:  {local_path}\n"
            "Please download the dataset or update the config."
        )

    def _get_split_sessions(self) -> list[str]:
        """return session list for the current split"""
        mapping: dict[str, list[str]] = {
            "train": self.config.train_sessions,
            "val": self.config.val_sessions,
            "test": self.config.test_sessions,
        }
        sessions = mapping[self.split]
        if not sessions:
            raise ValueError(
                f"No sessions configured for split '{self.split}'."
            )
        return sessions

    def _build_index(self) -> None:
        """scan filesystem and build flat sample index from poses + .bin files"""
        for session in self.sessions:
            session_dir = self.data_root / session

            if not session_dir.exists():
                logger.warning(
                    "Session directory missing, skipping: %s", session_dir
                )
                continue

            # load poses
            try:
                poses_dict = self.load_poses(session_dir)
            except (FileNotFoundError, RuntimeError, ValueError) as exc:
                logger.warning(
                    "Could not load poses for session %s: %s", session, exc
                )
                continue

            # resolve velodyne directory (Kaggle uses velodyne_data/)
            velodyne_dir = session_dir / "velodyne_data"
            if not velodyne_dir.exists():
                velodyne_dir = session_dir / "velodyne"
            image_dir = session_dir / "images_small"

            if not velodyne_dir.exists():
                logger.warning(
                    "Velodyne directory missing, skipping session: %s",
                    session,
                )
                continue

            # enumerate .bin files and match to poses
            bin_files = sorted(velodyne_dir.glob("*.bin"))
            matched = 0

            for bin_path in bin_files:
                try:
                    ts = int(bin_path.stem)
                except ValueError:
                    logger.debug(
                        "Non-numeric bin filename, skipping: %s", bin_path.name
                    )
                    continue

                if ts not in poses_dict:
                    continue

                self.samples.append(
                    SampleRecord(
                        session=session,
                        timestamp=ts,
                        velodyne_path=bin_path,
                        image_dir=image_dir,
                        pose=poses_dict[ts],
                    )
                )
                matched += 1

            logger.info(
                "Session %s: %d/%d point clouds matched to poses",
                session,
                matched,
                len(bin_files),
            )

        if not self.samples:
            logger.warning(
                "No samples found for split '%s'. Check data paths and "
                "session directories.",
                self.split,
            )

    def _preprocess_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """apply ground removal and subsampling to raw point cloud"""
        if points.shape[0] == 0:
            return points

        # 1. ground removal
        if self.pc_config.remove_ground:
            mask = points[:, 2] > self.pc_config.ground_threshold
            points = points[mask]
            if points.shape[0] == 0:
                logger.debug("All points removed by ground filter.")
                return points

        # 2. random subsampling
        if points.shape[0] > self.pc_config.max_points:
            indices = np.random.choice(
                points.shape[0], size=self.pc_config.max_points, replace=False
            )
            points = points[indices]

        return points

    def _find_and_load_image(self, record: SampleRecord) -> np.ndarray | None:
        """find and load camera image matching sample timestamp, or None"""
        if not record.image_dir.exists():
            return None

        extensions = (".png", ".jpg", ".jpeg", ".tif", ".tiff")

        # try flat layout first
        for ext in extensions:
            candidate = record.image_dir / f"{record.timestamp}{ext}"
            if candidate.exists():
                return self.load_image(candidate)

        # try camera subdirectories (prefer Cam1, then any)
        cam_dirs = sorted(
            d for d in record.image_dir.iterdir()
            if d.is_dir() and d.name.startswith("Cam")
        )
        preferred = [d for d in cam_dirs if d.name == "Cam1"]
        search_order = preferred + [d for d in cam_dirs if d.name != "Cam1"]

        for cam_dir in search_order:
            for ext in extensions:
                candidate = cam_dir / f"{record.timestamp}{ext}"
                if candidate.exists():
                    return self.load_image(candidate)

        return None

    # ------------------------------------------------------------------
    # sensor addon helpers
    # ------------------------------------------------------------------

    def _resolve_sensors_path(
        self, override: str | Path | None = None,
    ) -> Path | None:
        """determine sensors addon dataset root, or None if unavailable"""
        if override is not None:
            p = Path(override)
            if p.exists() and p.is_dir():
                logger.info("Using sensor override path: %s", p)
                return p
            logger.warning("Sensor override path not found: %s", p)
            return None

        kaggle = Path(self.config.sensors_kaggle_path)
        if kaggle.exists() and kaggle.is_dir():
            logger.info("Using Kaggle sensors path: %s", kaggle)
            return kaggle

        local = Path(self.config.sensors_local_path)
        if local.exists() and local.is_dir():
            logger.info("Using local sensors path: %s", local)
            return local

        logger.warning(
            "Sensors addon dataset not found. "
            "Checked: %s, %s", kaggle, local,
        )
        return None

    def _init_sensor_managers(self) -> None:
        if self._sensors_root is None:
            return

        try:
            from src.datasets.sensor_loader import SessionSensorManager
        except ImportError:
            logger.warning(
                "sensor_loader module not available. "
                "Sensor data will not be loaded."
            )
            return

        sensor_config = self.config.sensors
        sensor_sessions = set(self.config.sensor_sessions)

        for session in self.sessions:
            if session not in sensor_sessions:
                continue

            session_dir = self._find_sensor_session_dir(session)
            if session_dir is None:
                continue

            self._sensor_managers[session] = SessionSensorManager(
                session_dir=session_dir,
                config=sensor_config,
            )
            logger.debug("Sensor manager initialized for session %s", session)

        logger.info(
            "Sensor managers: %d/%d sessions",
            len(self._sensor_managers),
            len(self.sessions),
        )

    def _find_sensor_session_dir(self, session: str) -> Path | None:
        """locate sensor directory for a session (handles Kaggle nested paths)"""
        if self._sensors_root is None:
            return None

        # direct path
        direct = self._sensors_root / session
        if direct.exists():
            return direct

        # Kaggle nested path
        for subdir in self._sensors_root.iterdir():
            if subdir.is_dir() and not subdir.name.startswith("."):
                nested = subdir / session
                if nested.exists():
                    return nested

        logger.debug(
            "Sensor session directory not found: %s/%s",
            self._sensors_root, session,
        )
        return None

    def _load_sensor_data(self, record: SampleRecord) -> dict[str, Any]:
        """load synchronized sensor readings for a sample"""
        manager = self._sensor_managers.get(record.session)
        if manager is None:
            return {}

        window_us = self.sensor_window_ms * 1000
        result: dict[str, Any] = {}

        # IMU
        if manager.imu is not None:
            imu_val = manager.imu.query(record.timestamp, window_us=window_us)
            if imu_val is not None:
                result["imu"] = torch.from_numpy(imu_val).float()

        # GPS
        if manager.gps is not None:
            gps_val = manager.gps.query(record.timestamp, window_us=window_us)
            if gps_val is not None:
                result["gps"] = torch.from_numpy(gps_val).float()

        # wheel odometry
        if manager.odometry is not None:
            odo_val = manager.odometry.query(
                record.timestamp, window_us=window_us,
            )
            if odo_val is not None:
                result["odometry"] = torch.from_numpy(odo_val).float()

        # fiber optic gyro
        if manager.kvh is not None:
            kvh_val = manager.kvh.query(record.timestamp, window_us=window_us)
            if kvh_val is not None:
                result["kvh"] = torch.from_numpy(kvh_val).float()

        # ground truth from sensors addon
        if manager.ground_truth is not None:
            gt_val = manager.ground_truth.query(
                record.timestamp, window_us=window_us,
            )
            if gt_val is not None:
                result["sensor_gt"] = torch.from_numpy(gt_val).float()

        return result

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"NCLTDataset(split='{self.split}', sessions={self.sessions}, "
            f"samples={len(self.samples)}, root='{self.data_root}')"
        )
