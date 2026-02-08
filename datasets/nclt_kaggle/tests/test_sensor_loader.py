"""tests for sensor loaders, synchronizer, session manager, and utilities"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

MOCK_DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "nclt_mock" / "2012-01-08"

SENSOR_CONFIG = {
    "imu": {"filename": "ms25.csv", "rate_hz": 100},
    "gps": {"filename": "gps_rtk.csv", "rate_hz": 1},
    "odometry": {"filename": "odometry_mu_100hz.csv", "rate_hz": 100},
    "kvh": {"filename": "kvh.csv", "rate_hz": 100},
    "ground_truth": {"filename": "groundtruth.csv"},
    "sync": {"window_ms": 50, "interpolate": True},
}

# First timestamp in the mock data (microseconds)
T0 = 1326044400000000


@pytest.fixture
def data_dir() -> Path:
    """path to mock session data directory"""
    assert MOCK_DATA_DIR.exists(), f"Mock data dir missing: {MOCK_DATA_DIR}"
    return MOCK_DATA_DIR


@pytest.fixture
def sensor_config() -> dict:
    """sensor config dict"""
    return dict(SENSOR_CONFIG)


@pytest.fixture
def imu_loader(data_dir: Path):
    """IMULoader for mock ms25.csv"""
    from src.datasets.sensor_loader import IMULoader

    return IMULoader(data_dir / "ms25.csv")


@pytest.fixture
def gps_loader(data_dir: Path):
    """GPSLoader for mock gps_rtk.csv"""
    from src.datasets.sensor_loader import GPSLoader

    return GPSLoader(data_dir / "gps_rtk.csv")


@pytest.fixture
def odometry_loader(data_dir: Path):
    """OdometryLoader for mock odometry"""
    from src.datasets.sensor_loader import OdometryLoader

    return OdometryLoader(data_dir / "odometry_mu_100hz.csv")


@pytest.fixture
def kvh_loader(data_dir: Path):
    """KVHLoader for mock kvh.csv"""
    from src.datasets.sensor_loader import KVHLoader

    return KVHLoader(data_dir / "kvh.csv")


@pytest.fixture
def groundtruth_loader(data_dir: Path):
    """GroundTruthLoader for mock groundtruth.csv"""
    from src.datasets.sensor_loader import GroundTruthLoader

    return GroundTruthLoader(data_dir / "groundtruth.csv")


@pytest.fixture
def session_manager(data_dir: Path, sensor_config: dict):
    """SessionSensorManager for mock session"""
    from src.datasets.sensor_loader import SessionSensorManager

    return SessionSensorManager(data_dir, sensor_config)


# ---------------------------------------------------------------------------
# Tests for BaseSensorLoader (exercised via IMULoader)
# ---------------------------------------------------------------------------


class TestBaseSensorLoader:
    """BaseSensorLoader tests via IMULoader"""

    def test_load(self, imu_loader):
        """verify shape and column names"""
        data = imu_loader.load()

        assert data.values.shape == (1000, 9)
        assert data.timestamps.shape == (1000,)
        assert data.columns == [
            "mag_x", "mag_y", "mag_z",
            "accel_x", "accel_y", "accel_z",
            "rot_x", "rot_y", "rot_z",
        ]

    def test_query_exact(self, imu_loader):
        """exact timestamp returns non-None"""
        result = imu_loader.query(T0)

        assert result is not None
        assert result.shape == (9,)

    def test_query_within_window(self, imu_loader):
        """offset 5000 us still finds match"""
        result = imu_loader.query(T0 + 5000, window_us=50_000)

        assert result is not None
        assert result.shape == (9,)

    def test_query_outside_window(self, imu_loader):
        """100 ms outside range returns None"""
        data = imu_loader.load()
        far_ts = int(data.timestamps[-1]) + 100_000

        result = imu_loader.query(far_ts, window_us=50_000)

        assert result is None

    def test_slice(self, imu_loader):
        """1-second slice yields ~100 samples"""
        start = T0
        end = T0 + 1_000_000  # 1 second

        sliced = imu_loader.slice(start, end)

        # At 100 Hz we expect ~101 samples (inclusive endpoints)
        assert 95 <= len(sliced) <= 110
        assert sliced.values.shape[1] == 9
        assert sliced.columns == imu_loader.load().columns

    def test_interpolate(self, imu_loader):
        """interpolation between two timestamps"""
        # Midpoint between the first two samples
        mid_ts = T0 + 5000  # halfway between T0 and T0 + 10000

        result = imu_loader.interpolate(mid_ts)

        assert result is not None
        assert result.shape == (9,)

        # value between bounding rows
        data = imu_loader.load()
        v0 = data.values[0]
        v1 = data.values[1]
        expected = 0.5 * v0 + 0.5 * v1
        np.testing.assert_allclose(result, expected, atol=1e-8)

    def test_interpolate_outside_range(self, imu_loader):
        """outside range returns None"""
        result = imu_loader.interpolate(T0 - 1_000_000)

        assert result is None


# ---------------------------------------------------------------------------
# Tests for individual sensor loaders
# ---------------------------------------------------------------------------


class TestIndividualLoaders:
    """column counts and names for each loader"""

    def test_imu_loader(self, imu_loader):
        """IMU loader: 9 columns"""
        data = imu_loader.load()

        assert data.values.shape[1] == 9
        assert data.columns == [
            "mag_x", "mag_y", "mag_z",
            "accel_x", "accel_y", "accel_z",
            "rot_x", "rot_y", "rot_z",
        ]

    def test_gps_loader(self, gps_loader):
        """GPS loader: 7 columns"""
        data = gps_loader.load()

        assert data.values.shape[1] == 7
        assert data.columns == [
            "mode", "num_satell", "latitude", "longitude",
            "altitude", "track", "speed",
        ]

    def test_odometry_loader(self, odometry_loader):
        """odometry loader: 6 columns"""
        data = odometry_loader.load()

        assert data.values.shape[1] == 6
        assert data.columns == ["x", "y", "z", "roll", "pitch", "yaw"]

    def test_kvh_loader(self, kvh_loader):
        """KVH loader: 1 column"""
        data = kvh_loader.load()

        assert data.values.shape[1] == 1
        assert data.columns == ["heading"]

    def test_groundtruth_loader(self, groundtruth_loader):
        """ground truth: 6 columns and valid SE(3) pose"""
        data = groundtruth_loader.load()

        assert data.values.shape[1] == 6
        assert data.columns == ["x", "y", "z", "roll", "pitch", "yaw"]

        # get_pose_matrix should return a (4, 4) homogeneous matrix
        pose = groundtruth_loader.get_pose_matrix(T0)
        assert pose is not None
        assert pose.shape == (4, 4)

    def test_groundtruth_pose_matrix_is_se3(self, groundtruth_loader):
        """Verify the pose matrix is a valid SE(3) element"""
        pose = groundtruth_loader.get_pose_matrix(T0)
        assert pose is not None

        R = pose[:3, :3]

        # det(R) should be 1
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-10)

        # R @ R.T should be identity
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)

        # Bottom row should be [0, 0, 0, 1]
        np.testing.assert_allclose(pose[3, :], [0, 0, 0, 1], atol=1e-10)


# ---------------------------------------------------------------------------
# Tests for SensorSynchronizer
# ---------------------------------------------------------------------------


class TestSensorSynchronizer:
    """Tests for the SensorSynchronizer class"""

    def test_get_synchronized(self, session_manager):
        """All sensors should return data at a valid query timestamp"""
        sync = session_manager.get_synchronizer(
            sensor_names=["imu", "gps", "odometry", "kvh", "ground_truth"],
        )

        result = sync.get_synchronized(T0)

        assert isinstance(result, dict)
        for name in ("imu", "gps", "odometry", "kvh", "ground_truth"):
            assert name in result, f"Missing sensor '{name}' in synchronized output"
            assert result[name] is not None, f"Sensor '{name}' returned None at T0"

    def test_get_window(self, session_manager):
        """Slice a window and verify SensorData returned for all sensors"""
        sync = session_manager.get_synchronizer(
            sensor_names=["imu", "gps", "odometry", "kvh", "ground_truth"],
        )

        start = T0
        end = T0 + 1_000_000  # 1 second window

        window = sync.get_window(start, end)

        assert isinstance(window, dict)
        for name in ("imu", "gps", "odometry", "kvh", "ground_truth"):
            assert name in window
            sensor_data = window[name]
            assert len(sensor_data) > 0, f"Sensor '{name}' has no data in window"


# ---------------------------------------------------------------------------
# Tests for SessionSensorManager
# ---------------------------------------------------------------------------


class TestSessionSensorManager:
    """Tests for the SessionSensorManager class"""

    def test_all_sensors_available(self, session_manager):
        """All five sensor properties should return non-None loaders"""
        assert session_manager.imu is not None
        assert session_manager.gps is not None
        assert session_manager.odometry is not None
        assert session_manager.kvh is not None
        assert session_manager.ground_truth is not None

    def test_missing_sensor(self, sensor_config: dict):
        """Manager with a non-existent dir should return None for sensors"""
        from src.datasets.sensor_loader import SessionSensorManager

        bad_manager = SessionSensorManager(
            Path("/nonexistent/session/dir"), sensor_config,
        )

        assert bad_manager.imu is None
        assert bad_manager.gps is None
        assert bad_manager.odometry is None
        assert bad_manager.kvh is None
        assert bad_manager.ground_truth is None

    def test_get_synchronizer(self, session_manager):
        """Synchronizer should be created successfully with available sensors"""
        from src.datasets.sensor_loader import SensorSynchronizer

        sync = session_manager.get_synchronizer()

        assert isinstance(sync, SensorSynchronizer)


# ---------------------------------------------------------------------------
# Tests for imu_utils.py
# ---------------------------------------------------------------------------


class TestIMUUtils:
    """Tests for IMU utility functions"""

    def test_parse_imu_csv(self, data_dir: Path):
        """Parse mock ms25.csv and verify all four output keys"""
        from src.utils.imu_utils import parse_imu_csv

        result = parse_imu_csv(data_dir / "ms25.csv")

        assert "timestamps" in result
        assert "magnetometer" in result
        assert "accelerometer" in result
        assert "gyroscope" in result

        assert result["timestamps"].shape == (1000,)
        assert result["magnetometer"].shape == (1000, 3)
        assert result["accelerometer"].shape == (1000, 3)
        assert result["gyroscope"].shape == (1000, 3)

    def test_skew_symmetric(self):
        """Skew-symmetric matrix should be 3x3 and antisymmetric"""
        from src.utils.imu_utils import skew_symmetric

        v = np.array([1.0, 2.0, 3.0])
        S = skew_symmetric(v)

        assert S.shape == (3, 3)

        # Antisymmetric: S + S.T = 0
        np.testing.assert_allclose(S + S.T, np.zeros((3, 3)), atol=1e-15)

        # Diagonal should be zero
        np.testing.assert_allclose(np.diag(S), [0.0, 0.0, 0.0], atol=1e-15)

    def test_rodrigues_identity(self):
        """Rodrigues with zero rotation vector should give identity"""
        from src.utils.imu_utils import rodrigues

        R = rodrigues(np.array([0.0, 0.0, 0.0]))

        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_rodrigues_90deg(self):
        """Rodrigues rotation of pi/2 around Z-axis"""
        from src.utils.imu_utils import rodrigues

        angle = np.pi / 2
        R = rodrigues(np.array([0.0, 0.0, angle]))

        expected = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
        ])
        np.testing.assert_allclose(R, expected, atol=1e-10)

    def test_integrate_gyroscope(self, data_dir: Path):
        """Integrated gyroscope output should have shape (N, 3, 3)."""
        from src.utils.imu_utils import integrate_gyroscope, parse_imu_csv

        imu = parse_imu_csv(data_dir / "ms25.csv")
        n_samples = 100
        ts = imu["timestamps"][:n_samples]
        gyro = imu["gyroscope"][:n_samples]

        orientations = integrate_gyroscope(ts, gyro)

        assert orientations.shape == (n_samples, 3, 3)

        # First orientation should be identity
        np.testing.assert_allclose(orientations[0], np.eye(3), atol=1e-10)

    def test_imu_preintegration(self, data_dir: Path):
        """Preintegration should return positions, velocities, orientations"""
        from src.utils.imu_utils import imu_preintegration, parse_imu_csv

        imu = parse_imu_csv(data_dir / "ms25.csv")
        n_samples = 50
        ts = imu["timestamps"][:n_samples]
        accel = imu["accelerometer"][:n_samples]
        gyro = imu["gyroscope"][:n_samples]

        result = imu_preintegration(ts, accel, gyro)

        assert "positions" in result
        assert "velocities" in result
        assert "orientations" in result

        assert result["positions"].shape == (n_samples, 3)
        assert result["velocities"].shape == (n_samples, 3)
        assert result["orientations"].shape == (n_samples, 3, 3)

        # First position and velocity should be zero
        np.testing.assert_allclose(result["positions"][0], [0, 0, 0], atol=1e-15)
        np.testing.assert_allclose(result["velocities"][0], [0, 0, 0], atol=1e-15)

    def test_gravity_alignment(self, data_dir: Path):
        """Gravity alignment should return a 3x3 rotation matrix"""
        from src.utils.imu_utils import gravity_alignment, parse_imu_csv

        imu = parse_imu_csv(data_dir / "ms25.csv")
        accel = imu["accelerometer"]

        R = gravity_alignment(accel)

        assert R.shape == (3, 3)

        # Should be a valid rotation: det(R) = 1, R @ R.T = I
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-10)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)

    def test_compute_bias(self, data_dir: Path):
        """Gyroscope bias estimate should have shape (3,)."""
        from src.utils.imu_utils import compute_bias, parse_imu_csv

        imu = parse_imu_csv(data_dir / "ms25.csv")
        ts = imu["timestamps"]
        gyro = imu["gyroscope"]

        bias = compute_bias(ts, gyro)

        assert bias.shape == (3,)

        # With near-stationary mock data the bias magnitudes should be small
        assert np.all(np.abs(bias) < 1.0)


# ---------------------------------------------------------------------------
# Tests for gps_utils.py
# ---------------------------------------------------------------------------


class TestGPSUtils:
    """Tests for GPS utility functions"""

    def test_lla_to_ecef_equator(self):
        """At (0, 0, 0) the ECEF X coordinate should be ~WGS84_A"""
        from src.utils.gps_utils import WGS84_A, lla_to_ecef

        ecef = lla_to_ecef(0.0, 0.0, 0.0)

        assert ecef.shape == (3,)
        np.testing.assert_allclose(ecef[0], WGS84_A, atol=1.0)
        np.testing.assert_allclose(ecef[1], 0.0, atol=1e-6)
        np.testing.assert_allclose(ecef[2], 0.0, atol=1e-6)

    def test_ecef_to_enu_same_point(self):
        """ECEF of the reference point converted to ENU should be (0,0,0)."""
        from src.utils.gps_utils import ecef_to_enu, lla_to_ecef

        lat_ref, lon_ref, alt_ref = 42.293195, -83.709657, 270.0
        ecef = lla_to_ecef(lat_ref, lon_ref, alt_ref)

        enu = ecef_to_enu(ecef[0], ecef[1], ecef[2], lat_ref, lon_ref, alt_ref)

        np.testing.assert_allclose(enu, [0.0, 0.0, 0.0], atol=1e-6)

    def test_lla_to_enu_batch(self):
        """Batch conversion of 3 points should return shape (3, 3)."""
        from src.utils.gps_utils import lla_to_enu

        lat = np.array([42.293195, 42.293200, 42.293210])
        lon = np.array([-83.709657, -83.709660, -83.709670])
        alt = np.array([270.0, 270.5, 271.0])

        enu = lla_to_enu(lat, lon, alt)

        assert enu.shape == (3, 3)

        # First point is the reference, so ENU should be ~ (0, 0, 0)
        np.testing.assert_allclose(enu[0], [0.0, 0.0, 0.0], atol=1e-6)

    def test_lla_to_enu_empty(self):
        """Empty arrays should produce an (0, 3) output"""
        from src.utils.gps_utils import lla_to_enu

        enu = lla_to_enu(
            np.array([]), np.array([]), np.array([]),
        )

        assert enu.shape == (0, 3)

    def test_haversine_distance(self):
        """1 degree of latitude should be approximately 111 km"""
        from src.utils.gps_utils import haversine_distance

        dist = haversine_distance(0.0, 0.0, 1.0, 0.0)

        # 1 degree latitude ~ 111.19 km
        assert 110_000 < dist < 112_000

    def test_gps_to_pose(self):
        """GPS trajectory should produce (N, 4, 4) SE(3) matrices"""
        from src.utils.gps_utils import gps_to_pose

        n = 5
        ts = np.arange(n, dtype=np.int64) * 1_000_000 + T0
        lat = np.full(n, 42.293195) + np.arange(n) * 1e-5
        lon = np.full(n, -83.709657) + np.arange(n) * 1e-5
        alt = np.full(n, 270.0)

        poses = gps_to_pose(ts, lat, lon, alt)

        assert poses.shape == (n, 4, 4)

        # Each pose should be a valid SE(3) element
        for i in range(n):
            R = poses[i, :3, :3]
            np.testing.assert_allclose(
                np.linalg.det(R), 1.0, atol=1e-10,
                err_msg=f"Pose {i}: det(R) != 1",
            )
            np.testing.assert_allclose(
                R @ R.T, np.eye(3), atol=1e-10,
                err_msg=f"Pose {i}: R is not orthogonal",
            )
            np.testing.assert_allclose(
                poses[i, 3, :], [0, 0, 0, 1], atol=1e-10,
                err_msg=f"Pose {i}: invalid bottom row",
            )

    def test_parse_gps_csv(self, data_dir: Path):
        """Parse mock gps.csv and verify output keys and shapes"""
        from src.utils.gps_utils import parse_gps_csv

        result = parse_gps_csv(data_dir / "gps_rtk.csv")

        assert "timestamps" in result
        assert "latitude" in result
        assert "longitude" in result
        assert "altitude" in result
        assert "mode" in result
        assert "num_satellites" in result
        assert "heading" in result
        assert "speed" in result

        assert result["timestamps"].shape == (10,)
        assert result["latitude"].shape == (10,)

    def test_filter_gps_by_fix(self, data_dir: Path):
        """Filtering by mode should keep only rows with mode >= min_mode"""
        from src.utils.gps_utils import filter_gps_by_fix, parse_gps_csv

        gps = parse_gps_csv(data_dir / "gps_rtk.csv")

        ts_out, lat_out, lon_out, alt_out = filter_gps_by_fix(
            gps["timestamps"],
            gps["mode"],
            gps["latitude"],
            gps["longitude"],
            gps["altitude"],
            min_mode=1,
        )

        # All mock GPS data has mode=2, so everything should pass
        assert len(ts_out) == len(gps["timestamps"])

        # With a higher threshold some or all may be filtered
        ts_strict, _, _, _ = filter_gps_by_fix(
            gps["timestamps"],
            gps["mode"],
            gps["latitude"],
            gps["longitude"],
            gps["altitude"],
            min_mode=4,
        )

        # mode=2 < 4, so all should be filtered out
        assert len(ts_strict) == 0
