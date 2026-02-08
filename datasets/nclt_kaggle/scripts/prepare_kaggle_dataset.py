#!/usr/bin/env python3
"""Prepare NCLT dataset for training on Kaggle.

Handles Kaggle notebook setup: symlinks, data integrity checks,
pair CSV generation, and dataset statistics.

Usage:
    python scripts/prepare_kaggle_dataset.py --data-path ./data/NCLT_preprocessed
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
KAGGLE_INPUT = Path("/kaggle/input/nclt-iprofi-hack-23/NCLT_preprocessed")
LOCAL_DATA = PROJECT_ROOT / "data" / "NCLT_preprocessed"

SESSIONS = [
    "2012-01-08", "2012-01-22", "2012-02-12", "2012-02-18",
    "2012-03-31", "2012-05-26", "2012-08-04", "2012-10-28",
    "2012-11-04", "2012-12-01",
]

TRAIN_SESSIONS = ["2012-01-08", "2012-01-22", "2012-02-12", "2012-02-18"]
VAL_SESSIONS = ["2012-03-31", "2012-05-26"]
TEST_SESSIONS = ["2012-08-04", "2012-10-28", "2012-11-04", "2012-12-01"]

POSITIVE_THRESHOLD = 10.0  # meters
NEGATIVE_THRESHOLD = 25.0  # meters


def resolve_data_path(custom_path: Path | None = None) -> Path:
    """find the dataset path, checking Kaggle then local"""
    if custom_path and custom_path.exists():
        return custom_path

    if KAGGLE_INPUT.exists():
        logger.info("Running on Kaggle, using: %s", KAGGLE_INPUT)
        return KAGGLE_INPUT

    if LOCAL_DATA.exists():
        logger.info("Using local data: %s", LOCAL_DATA)
        return LOCAL_DATA

    raise FileNotFoundError(
        f"Dataset not found. Checked:\n"
        f"  - {custom_path}\n"
        f"  - {KAGGLE_INPUT}\n"
        f"  - {LOCAL_DATA}\n"
        f"Run: python scripts/download_nclt_sample.py"
    )


def load_session_poses(session_dir: Path) -> pd.DataFrame:
    """load poses from a session's track.csv"""
    track_file = session_dir / "track.csv"
    if not track_file.exists():
        logger.warning("track.csv not found in %s", session_dir)
        return pd.DataFrame()

    df = pd.read_csv(track_file, header=None)
    if len(df.columns) >= 7:
        df.columns = ["timestamp", "x", "y", "z", "roll", "pitch", "yaw"][:len(df.columns)]
    return df


def generate_pairs(
    data_path: Path,
    sessions: list[str],
    positive_threshold: float = POSITIVE_THRESHOLD,
    negative_threshold: float = NEGATIVE_THRESHOLD,
    max_pairs: int = 50000,
) -> pd.DataFrame:
    """generate place recognition pairs from pose data across sessions"""
    logger.info("Generating pairs from %d sessions...", len(sessions))

    # load all poses
    session_poses: dict[str, np.ndarray] = {}
    for session in sessions:
        session_dir = data_path / session
        df = load_session_poses(session_dir)
        if not df.empty:
            session_poses[session] = df[["x", "y", "z"]].values
            logger.info("  %s: %d poses", session, len(df))

    if len(session_poses) < 2:
        logger.warning("Need at least 2 sessions for pair generation.")
        return pd.DataFrame()

    pairs = []
    session_list = list(session_poses.keys())

    for i, sess_a in enumerate(session_list):
        for sess_b in session_list[i + 1:]:
            pos_a = session_poses[sess_a]
            pos_b = session_poses[sess_b]

            # subsample for efficiency
            step_a = max(1, len(pos_a) // 1000)
            step_b = max(1, len(pos_b) // 1000)
            idx_a = np.arange(0, len(pos_a), step_a)
            idx_b = np.arange(0, len(pos_b), step_b)

            for ai in idx_a:
                dists = np.linalg.norm(pos_b[idx_b] - pos_a[ai], axis=1)
                positives = idx_b[dists < positive_threshold]

                if len(positives) > 0:
                    # take closest positive
                    best = positives[np.argmin(dists[dists < positive_threshold])]
                    dist = np.linalg.norm(pos_b[best] - pos_a[ai])
                    pairs.append({
                        "anchor_session": sess_a,
                        "anchor_idx": int(ai),
                        "positive_session": sess_b,
                        "positive_idx": int(best),
                        "distance": float(dist),
                    })

                if len(pairs) >= max_pairs:
                    break
            if len(pairs) >= max_pairs:
                break
        if len(pairs) >= max_pairs:
            break

    df = pd.DataFrame(pairs)
    logger.info("Generated %d pairs.", len(df))
    return df


def compute_statistics(data_path: Path) -> dict:
    """compute dataset statistics per session"""
    stats: dict = {"sessions": {}}

    known_sessions = [
        "2012-01-08", "2012-01-22", "2012-02-12", "2012-02-18",
        "2012-03-31", "2012-05-26", "2012-08-04", "2012-10-28",
        "2012-11-04", "2012-12-01",
    ]

    for session_dir in sorted(
        p for p in data_path.iterdir()
        if p.is_dir() and p.name in known_sessions
    ):
        if not session_dir.is_dir():
            continue

        session_name = session_dir.name
        session_stats: dict = {}

        # count velodyne files
        velodyne_dir = session_dir / "velodyne"
        if velodyne_dir.exists():
            bin_files = list(velodyne_dir.glob("*.bin"))
            session_stats["num_scans"] = len(bin_files)

        # count images
        images_dir = session_dir / "images_small"
        if images_dir.exists():
            img_files = list(images_dir.glob("*"))
            session_stats["num_images"] = len(img_files)

        # pose stats
        df = load_session_poses(session_dir)
        if not df.empty:
            session_stats["num_poses"] = len(df)
            pos = df[["x", "y"]].values
            trajectory_length = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))
            session_stats["trajectory_length_m"] = round(trajectory_length, 1)

        stats["sessions"][session_name] = session_stats

    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepare NCLT dataset")
    parser.add_argument("--data-path", type=Path, default=None)
    parser.add_argument("--generate-pairs", action="store_true",
                        help="Force regenerate pair CSVs")
    args = parser.parse_args()

    data_path = resolve_data_path(args.data_path)
    logger.info("Dataset path: %s", data_path)

    # compute and print statistics
    stats = compute_statistics(data_path)
    logger.info("Dataset statistics:")
    for session, s in stats.get("sessions", {}).items():
        logger.info("  %s: %s", session, s)

    # check/generate pair CSVs
    for split, sessions in [
        ("train", TRAIN_SESSIONS),
        ("val", VAL_SESSIONS),
        ("test", TEST_SESSIONS),
    ]:
        csv_path = data_path / f"{split}.csv"
        if csv_path.exists() and not args.generate_pairs:
            df = pd.read_csv(csv_path)
            logger.info("%s.csv: %d pairs (existing)", split, len(df))
        else:
            logger.info("Generating %s pairs...", split)
            df = generate_pairs(data_path, sessions)
            if not df.empty:
                df.to_csv(csv_path, index=False)
                logger.info("Saved %d pairs to %s", len(df), csv_path)

    logger.info("Dataset preparation complete!")


if __name__ == "__main__":
    main()
