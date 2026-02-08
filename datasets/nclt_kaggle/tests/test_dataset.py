"""tests for NCLT dataset loading and pair generation"""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sample_config(tmp_path: Path) -> Path:
    """minimal dataset config for testing"""
    config_content = f"""\
nclt:
  kaggle_path: "/nonexistent/kaggle/path"
  local_path: "{tmp_path / 'NCLT_preprocessed'}"
  sessions:
    - "2012-01-08"
  train_sessions: ["2012-01-08"]
  val_sessions: ["2012-01-08"]
  test_sessions: ["2012-01-08"]
  positive_threshold: 10.0
  negative_threshold: 25.0
  point_cloud:
    max_points: 1000
    voxel_size: 0.1
    remove_ground: true
    ground_threshold: -1.5
"""
    config_path = tmp_path / "dataset_config.yaml"
    config_path.write_text(config_content)
    return config_path


@pytest.fixture
def sample_data(tmp_path: Path) -> Path:
    """minimal sample data for testing"""
    data_root = tmp_path / "NCLT_preprocessed"
    session_dir = data_root / "2012-01-08"
    velodyne_dir = session_dir / "velodyne"
    velodyne_dir.mkdir(parents=True)

    # Create track.csv with 5 poses
    poses = []
    for i in range(5):
        ts = 1000000 + i * 1000
        x, y, z = float(i * 10), float(i * 5), 0.0
        roll, pitch, yaw = 0.0, 0.0, float(i * 0.1)
        poses.append(f"{ts},{x},{y},{z},{roll},{pitch},{yaw}")

    track_path = session_dir / "track.csv"
    track_path.write_text("\n".join(poses) + "\n")

    # Create matching .bin point cloud files
    for i in range(5):
        ts = 1000000 + i * 1000
        # Create random point cloud: 100 points x 4 (x, y, z, intensity)
        points = np.random.randn(100, 4).astype(np.float32)
        bin_path = velodyne_dir / f"{ts}.bin"
        points.tofile(str(bin_path))

    return data_root


# ---------------------------------------------------------------------------
# Tests for NCLTDataset
# ---------------------------------------------------------------------------


class TestNCLTDataset:
    """NCLTDataset tests"""

    def test_dataset_loads(self, sample_config: Path, sample_data: Path):
        """dataset loads with sample data"""
        from src.datasets.nclt_dataset import NCLTDataset

        ds = NCLTDataset(config_path=sample_config, split="train")
        assert len(ds) == 5

    def test_dataset_getitem(self, sample_config: Path, sample_data: Path):
        """__getitem__ returns correct structure"""
        from src.datasets.nclt_dataset import NCLTDataset

        ds = NCLTDataset(config_path=sample_config, split="train")
        sample = ds[0]

        assert "point_cloud" in sample
        assert "pose" in sample
        assert "session" in sample
        assert "timestamp" in sample

    def test_point_cloud_shape(self, sample_config: Path, sample_data: Path):
        """point clouds have correct dimensions"""
        from src.datasets.nclt_dataset import NCLTDataset

        ds = NCLTDataset(config_path=sample_config, split="train")
        sample = ds[0]

        pc = sample["point_cloud"]
        # Should be (N, 4) - x, y, z, intensity
        assert pc.ndim == 2
        assert pc.shape[1] == 4

    def test_pose_format(self, sample_config: Path, sample_data: Path):
        """poses are valid SE3 matrices"""
        from src.datasets.nclt_dataset import NCLTDataset

        ds = NCLTDataset(config_path=sample_config, split="train")
        sample = ds[0]

        pose = sample["pose"]
        assert pose.shape == (4, 4), f"Expected (4, 4), got {pose.shape}"

        # bottom row [0, 0, 0, 1]
        pose_np = pose.numpy() if hasattr(pose, "numpy") else np.array(pose)
        np.testing.assert_array_almost_equal(
            pose_np[3, :], [0, 0, 0, 1], decimal=5
        )

        # rotation orthogonality
        R = pose_np[:3, :3]
        RtR = R.T @ R
        np.testing.assert_array_almost_equal(
            RtR, np.eye(3), decimal=5,
            err_msg="Rotation matrix is not orthogonal"
        )

    def test_session_subset(self, sample_config: Path, sample_data: Path):
        """get_session_dataset returns valid subset"""
        from src.datasets.nclt_dataset import NCLTDataset

        ds = NCLTDataset(config_path=sample_config, split="train")
        subset = ds.get_session_dataset("2012-01-08")

        assert len(subset) == len(ds)
        assert all(s["session"] == "2012-01-08" for s in [subset[i] for i in range(len(subset))])

    def test_invalid_split_raises(self, sample_config: Path, sample_data: Path):
        """invalid split raises ValueError"""
        from src.datasets.nclt_dataset import NCLTDataset

        with pytest.raises(ValueError, match="Invalid split"):
            NCLTDataset(config_path=sample_config, split="invalid")

    def test_missing_data_raises(self, tmp_path: Path):
        """missing data raises FileNotFoundError"""
        config_content = """\
nclt:
  kaggle_path: "/nonexistent/path1"
  local_path: "/nonexistent/path2"
  sessions: ["2012-01-08"]
  train_sessions: ["2012-01-08"]
  val_sessions: []
  test_sessions: []
  positive_threshold: 10.0
  negative_threshold: 25.0
  point_cloud:
    max_points: 1000
    voxel_size: 0.1
    remove_ground: false
    ground_threshold: -1.5
"""
        config_path = tmp_path / "bad_config.yaml"
        config_path.write_text(config_content)

        from src.datasets.nclt_dataset import NCLTDataset

        with pytest.raises(FileNotFoundError):
            NCLTDataset(config_path=config_path, split="train")


# ---------------------------------------------------------------------------
# Tests for point cloud loading
# ---------------------------------------------------------------------------


class TestPointCloudLoading:
    """point cloud loading tests"""

    def test_load_valid_bin(self, tmp_path: Path):
        """load valid .bin file"""
        from src.datasets.nclt_dataset import NCLTDataset

        points = np.random.randn(200, 4).astype(np.float32)
        bin_path = tmp_path / "test.bin"
        points.tofile(str(bin_path))

        loaded = NCLTDataset.load_point_cloud(bin_path)
        assert loaded.shape == (200, 4)
        assert loaded.dtype == np.float32
        np.testing.assert_array_equal(loaded, points)

    def test_load_missing_file_raises(self):
        """missing file raises FileNotFoundError"""
        from src.datasets.nclt_dataset import NCLTDataset

        with pytest.raises(FileNotFoundError):
            NCLTDataset.load_point_cloud(Path("/nonexistent/file.bin"))

    def test_load_xyz_format(self, tmp_path: Path):
        """load XYZ-only .bin file"""
        from src.datasets.nclt_dataset import NCLTDataset

        xyz = np.random.randn(150, 3).astype(np.float32)
        bin_path = tmp_path / "xyz.bin"
        xyz.tofile(str(bin_path))

        loaded = NCLTDataset.load_point_cloud(bin_path)
        assert loaded.shape == (150, 4)
        assert loaded.dtype == np.float32
        np.testing.assert_array_equal(loaded[:, :3], xyz)
        np.testing.assert_array_equal(loaded[:, 3], 1.0)

    def test_load_corrupted_file_raises(self, tmp_path: Path):
        """corrupted file raises ValueError"""
        from src.datasets.nclt_dataset import NCLTDataset

        bad_path = tmp_path / "bad.bin"
        bad_path.write_bytes(b"\x00" * 17)  # Not multiple of 12 or 16

        with pytest.raises(ValueError):
            NCLTDataset.load_point_cloud(bad_path)


# ---------------------------------------------------------------------------
# Tests for pose conversion
# ---------------------------------------------------------------------------


class TestPoseConversion:
    """pose_to_matrix conversion tests"""

    def test_identity_pose(self):
        """zero pose gives identity"""
        from src.datasets.nclt_dataset import NCLTDataset

        T = NCLTDataset.pose_to_matrix(0, 0, 0, 0, 0, 0)
        np.testing.assert_array_almost_equal(T, np.eye(4, dtype=np.float32))

    def test_translation_only(self):
        """pure translation"""
        from src.datasets.nclt_dataset import NCLTDataset

        T = NCLTDataset.pose_to_matrix(1, 2, 3, 0, 0, 0)
        np.testing.assert_array_almost_equal(T[:3, 3], [1, 2, 3])
        np.testing.assert_array_almost_equal(T[:3, :3], np.eye(3), decimal=5)

    def test_rotation_determinant(self):
        """rotation has determinant 1"""
        from src.datasets.nclt_dataset import NCLTDataset

        T = NCLTDataset.pose_to_matrix(0, 0, 0, 0.5, 0.3, 1.2)
        det = np.linalg.det(T[:3, :3])
        assert abs(det - 1.0) < 1e-5, f"Determinant should be 1, got {det}"
