"""NCLT sensors-addon loaders: IMU, GPS, odometry, gyro, ground truth from headerless CSVs"""

from __future__ import annotations

import bisect
import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data container
# ---------------------------------------------------------------------------

@dataclass
class SensorData:
    """container for a single sensor's time-series data"""

    timestamps: np.ndarray  # (N,) int64 microseconds
    values: np.ndarray      # (N, C) float64
    columns: list[str] = field(default_factory=list)

    def __len__(self) -> int:
        return self.timestamps.shape[0]

    def __repr__(self) -> str:
        return (
            f"SensorData(samples={len(self)}, columns={self.columns}, "
            f"time_range=[{self.timestamps[0]}, {self.timestamps[-1]}])"
            if len(self) > 0
            else "SensorData(samples=0)"
        )


# ---------------------------------------------------------------------------
# Base loader
# ---------------------------------------------------------------------------

class BaseSensorLoader:
    """base class for CSV sensor loaders with lazy I/O and bisect-based lookups"""

    def __init__(self, csv_path: Path, columns: list[str]) -> None:
        self._csv_path = Path(csv_path)
        self._columns = list(columns)
        self._data: SensorData | None = None

    # -- public API ---------------------------------------------------------

    def load(self) -> SensorData:
        """load and cache CSV, return SensorData"""
        if self._data is not None:
            return self._data

        if not self._csv_path.exists():
            raise FileNotFoundError(
                f"Sensor CSV not found: {self._csv_path}"
            )

        logger.debug("Loading sensor data from %s", self._csv_path)

        try:
            raw = np.loadtxt(
                str(self._csv_path), delimiter=",", dtype=np.float64,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to parse {self._csv_path}: {exc}"
            ) from exc

        if raw.ndim == 1:
            raw = raw.reshape(1, -1)

        expected_cols = 1 + len(self._columns)  # utime + value columns
        if raw.shape[1] != expected_cols:
            raise ValueError(
                f"Column count mismatch in {self._csv_path.name}: "
                f"expected {expected_cols}, got {raw.shape[1]}"
            )

        timestamps = raw[:, 0].astype(np.int64)
        values = raw[:, 1:]

        self._data = SensorData(
            timestamps=timestamps,
            values=values,
            columns=list(self._columns),
        )

        logger.info(
            "Loaded %d samples from %s", len(self._data), self._csv_path.name,
        )
        return self._data

    def query(
        self, utime: int, window_us: int = 50_000,
    ) -> np.ndarray | None:
        """find nearest sample within window_us, or None"""
        data = self.load()
        if len(data) == 0:
            return None

        ts = data.timestamps
        idx = bisect.bisect_left(ts, utime)

        best_idx: int | None = None
        best_delta = window_us + 1

        for candidate in (idx - 1, idx):
            if 0 <= candidate < len(ts):
                delta = abs(int(ts[candidate]) - utime)
                if delta < best_delta:
                    best_delta = delta
                    best_idx = candidate

        if best_idx is None or best_delta > window_us:
            return None

        return data.values[best_idx]

    def slice(self, start_utime: int, end_utime: int) -> SensorData:
        """return SensorData for closed interval [start, end] in microseconds"""
        data = self.load()
        ts = data.timestamps

        lo = bisect.bisect_left(ts, start_utime)
        hi = bisect.bisect_right(ts, end_utime)

        return SensorData(
            timestamps=ts[lo:hi].copy(),
            values=data.values[lo:hi].copy(),
            columns=list(data.columns),
        )

    def interpolate(self, utime: int) -> np.ndarray | None:
        """linearly interpolate values at utime, or None if outside range"""
        data = self.load()
        if len(data) < 2:
            return None

        ts = data.timestamps
        if utime < ts[0] or utime > ts[-1]:
            return None

        idx = bisect.bisect_left(ts, utime)

        # exact match
        if idx < len(ts) and ts[idx] == utime:
            return data.values[idx].copy()

        # idx points to the first element > utime, so bracket is [idx-1, idx]
        if idx == 0 or idx >= len(ts):
            return None

        t0 = int(ts[idx - 1])
        t1 = int(ts[idx])
        dt = t1 - t0
        if dt == 0:
            return data.values[idx - 1].copy()

        alpha = (utime - t0) / dt
        return (1.0 - alpha) * data.values[idx - 1] + alpha * data.values[idx]

    # -- helpers ------------------------------------------------------------

    @property
    def path(self) -> Path:
        return self._csv_path

    @property
    def is_available(self) -> bool:
        return self._csv_path.exists()

    def __repr__(self) -> str:
        status = "loaded" if self._data is not None else "not loaded"
        return (
            f"{self.__class__.__name__}(path='{self._csv_path}', "
            f"status={status})"
        )


# ---------------------------------------------------------------------------
# Concrete sensor loaders
# ---------------------------------------------------------------------------

class IMULoader(BaseSensorLoader):
    """loader for ms25.csv IMU data (Microstrain MS25)"""

    COLUMNS: list[str] = [
        "mag_x", "mag_y", "mag_z",
        "accel_x", "accel_y", "accel_z",
        "rot_x", "rot_y", "rot_z",
    ]

    def __init__(self, csv_path: Path) -> None:
        super().__init__(csv_path, self.COLUMNS)


class GPSLoader(BaseSensorLoader):
    """loader for gps.csv GPS data (Trimble)"""

    COLUMNS: list[str] = [
        "mode", "num_satell", "latitude", "longitude",
        "altitude", "track", "speed",
    ]

    def __init__(self, csv_path: Path) -> None:
        super().__init__(csv_path, self.COLUMNS)


class OdometryLoader(BaseSensorLoader):
    """loader for odometry_mu_100hz.csv wheel odometry (Segway RMP)"""

    COLUMNS: list[str] = ["x", "y", "z", "roll", "pitch", "yaw"]

    def __init__(self, csv_path: Path) -> None:
        super().__init__(csv_path, self.COLUMNS)


class KVHLoader(BaseSensorLoader):
    """loader for kvh.csv fiber optic gyro heading"""

    COLUMNS: list[str] = ["heading"]

    def __init__(self, csv_path: Path) -> None:
        super().__init__(csv_path, self.COLUMNS)


class GroundTruthLoader(BaseSensorLoader):
    """loader for groundtruth.csv post-processed ground truth poses"""

    COLUMNS: list[str] = ["x", "y", "z", "roll", "pitch", "yaw"]

    def __init__(self, csv_path: Path) -> None:
        super().__init__(csv_path, self.COLUMNS)

    def get_pose_matrix(
        self, utime: int, window_us: int = 50_000,
    ) -> np.ndarray | None:
        """return 4x4 SE(3) matrix for nearest ground truth timestamp, or None"""
        row = self.query(utime, window_us=window_us)
        if row is None:
            return None

        x, y, z, roll, pitch, yaw = row
        return _euler_to_se3(x, y, z, roll, pitch, yaw)


# ---------------------------------------------------------------------------
# Synchronizer
# ---------------------------------------------------------------------------

class SensorSynchronizer:
    """synchronize multiple sensor streams to a reference timeline"""

    def __init__(
        self,
        sensors: dict[str, BaseSensorLoader],
        window_ms: int = 50,
        interpolate: bool = True,
    ) -> None:
        self._sensors = dict(sensors)
        self._window_us = window_ms * 1000
        self._interpolate = interpolate

    def get_synchronized(
        self, utime: int,
    ) -> dict[str, np.ndarray | None]:
        """return {sensor_name: values_array | None} at given utime"""
        result: dict[str, np.ndarray | None] = {}

        for name, loader in self._sensors.items():
            if self._interpolate:
                value = loader.interpolate(utime)
                if value is None:
                    # fallback to nearest-neighbour
                    value = loader.query(utime, window_us=self._window_us)
            else:
                value = loader.query(utime, window_us=self._window_us)
            result[name] = value

        return result

    def get_window(
        self, start_utime: int, end_utime: int,
    ) -> dict[str, SensorData]:
        """return {sensor_name: SensorData} for time window [start, end]"""
        return {
            name: loader.slice(start_utime, end_utime)
            for name, loader in self._sensors.items()
        }

    def __repr__(self) -> str:
        names = list(self._sensors.keys())
        return (
            f"SensorSynchronizer(sensors={names}, "
            f"window_us={self._window_us}, "
            f"interpolate={self._interpolate})"
        )


# ---------------------------------------------------------------------------
# Session manager
# ---------------------------------------------------------------------------

# sensor name -> loader class
_SENSOR_REGISTRY: dict[str, type[BaseSensorLoader]] = {
    "imu": IMULoader,
    "gps": GPSLoader,
    "odometry": OdometryLoader,
    "kvh": KVHLoader,
    "ground_truth": GroundTruthLoader,
}


class SessionSensorManager:
    """lazy sensor loader manager for a single NCLT recording session"""

    def __init__(self, session_dir: Path, config: dict) -> None:
        self._session_dir = Path(session_dir)
        self._config = dict(config)

        # lazy-init caches
        self._loaders: dict[str, BaseSensorLoader | None] = {}

        logger.debug(
            "SessionSensorManager created for %s", self._session_dir,
        )

    # -- private helper to resolve a loader ---------------------------------

    def _get_loader(
        self, config_key: str, loader_cls: type[BaseSensorLoader],
    ) -> BaseSensorLoader | None:
        """return cached loader or create one; None if CSV missing"""
        if config_key in self._loaders:
            return self._loaders[config_key]

        sensor_cfg = self._config.get(config_key, {})
        filename = sensor_cfg.get("filename")
        if filename is None:
            logger.debug(
                "No filename configured for sensor '%s'", config_key,
            )
            self._loaders[config_key] = None
            return None

        csv_path = self._session_dir / filename
        if not csv_path.exists():
            logger.debug(
                "Sensor file not found, returning None: %s", csv_path,
            )
            self._loaders[config_key] = None
            return None

        loader = loader_cls(csv_path)
        self._loaders[config_key] = loader
        return loader

    # -- public sensor properties -------------------------------------------

    @property
    def imu(self) -> IMULoader | None:
        return self._get_loader("imu", IMULoader)  # type: ignore[return-value]

    @property
    def gps(self) -> GPSLoader | None:
        return self._get_loader("gps", GPSLoader)  # type: ignore[return-value]

    @property
    def odometry(self) -> OdometryLoader | None:
        return self._get_loader("odometry", OdometryLoader)  # type: ignore[return-value]

    @property
    def kvh(self) -> KVHLoader | None:
        return self._get_loader("kvh", KVHLoader)  # type: ignore[return-value]

    @property
    def ground_truth(self) -> GroundTruthLoader | None:
        return self._get_loader("ground_truth", GroundTruthLoader)  # type: ignore[return-value]

    # -- synchroniser factory -----------------------------------------------

    def get_synchronizer(
        self, sensor_names: list[str] | None = None,
    ) -> SensorSynchronizer:
        """build SensorSynchronizer for requested (or all available) sensors"""
        sync_cfg = self._config.get("sync", {})
        window_ms = int(sync_cfg.get("window_ms", 50))
        do_interpolate = bool(sync_cfg.get("interpolate", True))

        if sensor_names is None:
            sensor_names = list(_SENSOR_REGISTRY.keys())

        available: dict[str, BaseSensorLoader] = {}
        for name in sensor_names:
            loader_cls = _SENSOR_REGISTRY.get(name)
            if loader_cls is None:
                logger.warning("Unknown sensor name '%s', skipping.", name)
                continue

            loader = self._get_loader(name, loader_cls)
            if loader is not None:
                available[name] = loader

        if not available:
            raise ValueError(
                f"No sensor data available for the requested sensors "
                f"({sensor_names}) in {self._session_dir}"
            )

        logger.info(
            "Created synchronizer with sensors: %s (window=%d ms, "
            "interpolate=%s)",
            list(available.keys()),
            window_ms,
            do_interpolate,
        )
        return SensorSynchronizer(
            sensors=available,
            window_ms=window_ms,
            interpolate=do_interpolate,
        )

    # -- dunder -------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"SessionSensorManager(session_dir='{self._session_dir}')"
        )


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _euler_to_se3(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
) -> np.ndarray:
    """build 4x4 SE(3) matrix from position and ZYX Euler angles"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    T = np.eye(4, dtype=np.float64)
    T[0, 0] = cy * cp
    T[0, 1] = cy * sp * sr - sy * cr
    T[0, 2] = cy * sp * cr + sy * sr
    T[0, 3] = x
    T[1, 0] = sy * cp
    T[1, 1] = sy * sp * sr + cy * cr
    T[1, 2] = sy * sp * cr - cy * sr
    T[1, 3] = y
    T[2, 0] = -sp
    T[2, 1] = cp * sr
    T[2, 2] = cp * cr
    T[2, 3] = z

    return T
