"""IMU processing for NCLT: parsing, interpolation, integration, preintegration"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# column layout in ms25.csv (after utime)
_MAG_COLS = (1, 2, 3)    # mag_x, mag_y, mag_z
_ACCEL_COLS = (4, 5, 6)  # accel_x, accel_y, accel_z
_GYRO_COLS = (7, 8, 9)   # rot_x, rot_y, rot_z

_US_TO_S = 1e-6  # microseconds to seconds


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """convert 3-vector to 3x3 skew-symmetric matrix"""
    v = np.asarray(v, dtype=np.float64).ravel()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=np.float64)


def rodrigues(axis_angle: np.ndarray) -> np.ndarray:
    """convert axis-angle (3,) to 3x3 rotation matrix via Rodrigues' formula"""
    axis_angle = np.asarray(axis_angle, dtype=np.float64).ravel()
    theta = np.linalg.norm(axis_angle)

    if theta < 1e-8:
        # first-order approximation
        K = skew_symmetric(axis_angle)
        return np.eye(3, dtype=np.float64) + K

    k = axis_angle / theta
    K = skew_symmetric(k)
    R = (
        np.eye(3, dtype=np.float64)
        + np.sin(theta) * K
        + (1.0 - np.cos(theta)) * (K @ K)
    )
    return R


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_imu_csv(csv_path: str | Path) -> dict[str, np.ndarray]:
    """parse NCLT ms25.csv into timestamps/magnetometer/accelerometer/gyroscope arrays"""
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"IMU file not found: {csv_path}")

    data = np.loadtxt(csv_path, delimiter=",", dtype=np.float64)

    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[1] != 10:
        raise ValueError(
            f"Expected 10 columns in ms25.csv, got {data.shape[1]}"
        )

    result = {
        "timestamps": data[:, 0].astype(np.int64),
        "magnetometer": data[:, _MAG_COLS[0]:_MAG_COLS[-1] + 1].copy(),
        "accelerometer": data[:, _ACCEL_COLS[0]:_ACCEL_COLS[-1] + 1].copy(),
        "gyroscope": data[:, _GYRO_COLS[0]:_GYRO_COLS[-1] + 1].copy(),
    }

    logger.info(
        "Parsed %d IMU samples from %s (%.1f s span)",
        len(result["timestamps"]),
        csv_path.name,
        (result["timestamps"][-1] - result["timestamps"][0]) * _US_TO_S,
    )
    return result


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------


def interpolate_imu(
    timestamps: np.ndarray,
    values: np.ndarray,
    target_times: np.ndarray,
) -> np.ndarray:
    """linearly interpolate (N, C) IMU values at target timestamps"""
    timestamps = np.asarray(timestamps, dtype=np.float64)
    target_times = np.asarray(target_times, dtype=np.float64)

    if values.ndim == 1:
        values = values[:, np.newaxis]

    n_channels = values.shape[1]
    result = np.empty((len(target_times), n_channels), dtype=np.float64)

    for c in range(n_channels):
        result[:, c] = np.interp(target_times, timestamps, values[:, c])

    return result


# ---------------------------------------------------------------------------
# Bias estimation
# ---------------------------------------------------------------------------


def compute_bias(
    timestamps: np.ndarray,
    gyro: np.ndarray,
    static_duration_us: int = 5_000_000,
) -> np.ndarray:
    """estimate gyroscope bias (3,) from initial static period"""
    t0 = timestamps[0]
    mask = timestamps <= t0 + static_duration_us

    if np.sum(mask) < 2:
        logger.warning(
            "Fewer than 2 samples in static window (%d us); "
            "using all available data for bias estimation",
            static_duration_us,
        )
        mask = np.ones(len(timestamps), dtype=bool)

    bias = np.mean(gyro[mask], axis=0)
    logger.info(
        "Gyro bias estimated from %d samples (%.2f s): [%.6f, %.6f, %.6f] rad/s",
        np.sum(mask),
        np.sum(mask) * _US_TO_S,  # approximate; actual span may differ
        bias[0],
        bias[1],
        bias[2],
    )
    return bias


# ---------------------------------------------------------------------------
# Gravity alignment
# ---------------------------------------------------------------------------


def gravity_alignment(
    accel: np.ndarray,
    num_samples: int = 100,
) -> np.ndarray:
    """compute (3, 3) rotation aligning body frame with gravity from initial accel readings"""
    n = min(num_samples, len(accel))
    g_body = np.mean(accel[:n], axis=0)  # measured gravity in body frame
    g_mag = np.linalg.norm(g_body)

    if g_mag < 1e-6:
        logger.warning(
            "Measured gravity magnitude is near zero (%.4f m/s^2); "
            "returning identity",
            g_mag,
        )
        return np.eye(3, dtype=np.float64)

    # normalised gravity direction
    g_hat = g_body / g_mag

    # target: gravity along -Z
    z_target = np.array([0.0, 0.0, -1.0])

    # rotation axis from cross product
    axis = np.cross(g_hat, z_target)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(g_hat, z_target)

    if sin_angle < 1e-8:
        if cos_angle > 0:
            # already aligned
            return np.eye(3, dtype=np.float64)
        else:
            # 180-degree flip
            perp = np.array([1.0, 0.0, 0.0])
            if np.abs(np.dot(g_hat, perp)) > 0.9:
                perp = np.array([0.0, 1.0, 0.0])
            axis = np.cross(g_hat, perp)
            axis /= np.linalg.norm(axis)
            return rodrigues(axis * np.pi)

    axis /= sin_angle
    angle = np.arctan2(sin_angle, cos_angle)
    R = rodrigues(axis * angle)

    logger.info(
        "Gravity alignment: measured |g|=%.4f m/s^2, rotation angle=%.2f deg",
        g_mag,
        np.degrees(angle),
    )
    return R


# ---------------------------------------------------------------------------
# Gyroscope integration
# ---------------------------------------------------------------------------


def integrate_gyroscope(
    timestamps: np.ndarray,
    gyro: np.ndarray,
    initial_orientation: np.ndarray | None = None,
) -> np.ndarray:
    """integrate gyroscope into (N, 3, 3) orientation matrices via Rodrigues"""
    n = len(timestamps)
    orientations = np.empty((n, 3, 3), dtype=np.float64)

    if initial_orientation is not None:
        orientations[0] = initial_orientation
    else:
        orientations[0] = np.eye(3, dtype=np.float64)

    for k in range(n - 1):
        dt = (timestamps[k + 1] - timestamps[k]) * _US_TO_S
        dR = rodrigues(gyro[k] * dt)
        orientations[k + 1] = orientations[k] @ dR

    return orientations


# ---------------------------------------------------------------------------
# IMU preintegration
# ---------------------------------------------------------------------------


def imu_preintegration(
    timestamps: np.ndarray,
    accel: np.ndarray,
    gyro: np.ndarray,
    gravity: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """discrete IMU preintegration returning positions, velocities, orientations"""
    if gravity is None:
        gravity = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    else:
        gravity = np.asarray(gravity, dtype=np.float64).ravel()

    n = len(timestamps)

    positions = np.zeros((n, 3), dtype=np.float64)
    velocities = np.zeros((n, 3), dtype=np.float64)
    orientations = np.zeros((n, 3, 3), dtype=np.float64)
    orientations[0] = np.eye(3, dtype=np.float64)

    for k in range(n - 1):
        dt = (timestamps[k + 1] - timestamps[k]) * _US_TO_S
        R_k = orientations[k]

        # world-frame acceleration
        accel_world = R_k @ accel[k] + gravity

        # update orientation
        dR = rodrigues(gyro[k] * dt)
        orientations[k + 1] = R_k @ dR

        # update velocity
        velocities[k + 1] = velocities[k] + accel_world * dt

        # update position
        positions[k + 1] = (
            positions[k]
            + velocities[k] * dt
            + 0.5 * accel_world * dt * dt
        )

    logger.debug(
        "IMU preintegration: %d samples, %.3f s span, "
        "final pos=[%.3f, %.3f, %.3f]",
        n,
        (timestamps[-1] - timestamps[0]) * _US_TO_S,
        positions[-1, 0],
        positions[-1, 1],
        positions[-1, 2],
    )

    return {
        "positions": positions,
        "velocities": velocities,
        "orientations": orientations,
    }
