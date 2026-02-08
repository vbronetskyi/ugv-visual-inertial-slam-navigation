"""NCLT sensor calibration: loading and applying extrinsic transforms"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# approximate NCLT extrinsics (Velodyne HDL-32E to body frame)
VELODYNE_TO_BODY = np.array([
    [1.0, 0.0, 0.0, 0.002],
    [0.0, 1.0, 0.0, -0.004],
    [0.0, 0.0, 1.0, 0.957],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float64)

BODY_TO_VELODYNE = np.linalg.inv(VELODYNE_TO_BODY)


def load_calibration(calib_dir: str | Path) -> dict[str, np.ndarray]:
    """load calibration matrices from NCLT calibration directory"""
    calib_dir = Path(calib_dir)
    calibrations: dict[str, np.ndarray] = {}

    # default calibrations
    calibrations["velodyne_to_body"] = VELODYNE_TO_BODY.copy()
    calibrations["body_to_velodyne"] = BODY_TO_VELODYNE.copy()

    # load from file if available
    velodyne_calib = calib_dir / "velodyne_to_body.txt"
    if velodyne_calib.exists():
        try:
            mat = np.loadtxt(velodyne_calib).reshape(4, 4)
            calibrations["velodyne_to_body"] = mat
            calibrations["body_to_velodyne"] = np.linalg.inv(mat)
            logger.info("Loaded velodyne calibration from %s", velodyne_calib)
        except Exception as e:
            logger.warning("Failed to load %s: %s. Using defaults.", velodyne_calib, e)

    return calibrations


def euler_to_rotation_matrix(
    roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """convert Euler angles (ZYX convention) to 3x3 rotation matrix"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)

    return R


def pose_to_matrix(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
) -> np.ndarray:
    """convert position + Euler angles to 4x4 SE3 matrix"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = euler_to_rotation_matrix(roll, pitch, yaw)
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose(T: np.ndarray) -> tuple[float, float, float, float, float, float]:
    """convert 4x4 SE3 matrix to (x, y, z, roll, pitch, yaw) tuple"""
    x, y, z = T[:3, 3]
    R = T[:3, :3]

    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0

    return float(x), float(y), float(z), float(roll), float(pitch), float(yaw)


def interpolate_pose(
    poses: np.ndarray,
    timestamps: np.ndarray,
    query_timestamp: float,
) -> np.ndarray:
    """interpolate SE3 pose at given timestamp (linear translation, SLERP rotation)"""
    if query_timestamp < timestamps[0] or query_timestamp > timestamps[-1]:
        raise ValueError(
            f"Query timestamp {query_timestamp} outside range "
            f"[{timestamps[0]}, {timestamps[-1]}]"
        )

    idx = np.searchsorted(timestamps, query_timestamp) - 1
    idx = max(0, min(idx, len(timestamps) - 2))

    t0, t1 = timestamps[idx], timestamps[idx + 1]
    alpha = (query_timestamp - t0) / (t1 - t0) if t1 != t0 else 0.0

    # linear interpolation of translation
    trans = (1 - alpha) * poses[idx, :3, 3] + alpha * poses[idx + 1, :3, 3]

    # SLERP for rotation
    from scipy.spatial.transform import Rotation, Slerp

    rots = Rotation.from_matrix([poses[idx, :3, :3], poses[idx + 1, :3, :3]])
    slerp = Slerp([0, 1], rots)
    R_interp = slerp(alpha).as_matrix()

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_interp
    T[:3, 3] = trans
    return T
