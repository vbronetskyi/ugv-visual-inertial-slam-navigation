"""tests for place recognition models and evaluation metrics"""

from __future__ import annotations

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Tests for evaluation metrics
# ---------------------------------------------------------------------------


class TestMetrics:
    """SLAM evaluation metrics tests"""

    def test_compute_ate_identical(self):
        """ATE zero for identical trajectories"""
        from src.evaluation.metrics import compute_ate

        n = 20
        poses = np.zeros((n, 4, 4))
        for i in range(n):
            poses[i] = np.eye(4)
            poses[i, 0, 3] = float(i)

        ate = compute_ate(poses, poses)
        assert ate["rmse"] < 1e-10
        assert ate["mean"] < 1e-10

    def test_compute_ate_with_offset(self):
        """ATE detects constant offset after alignment"""
        from src.evaluation.metrics import compute_ate

        n = 50
        gt = np.zeros((n, 4, 4))
        est = np.zeros((n, 4, 4))
        for i in range(n):
            gt[i] = np.eye(4)
            gt[i, 0, 3] = float(i)
            est[i] = np.eye(4)
            est[i, 0, 3] = float(i)
            est[i, 1, 3] = 1.0  # constant Y offset

        ate = compute_ate(est, gt)
        # Umeyama alignment removes constant offset
        assert ate["rmse"] < 0.1

    def test_compute_rpe_identical(self):
        """RPE zero for identical trajectories"""
        from src.evaluation.metrics import compute_rpe

        n = 20
        poses = np.zeros((n, 4, 4))
        for i in range(n):
            poses[i] = np.eye(4)
            poses[i, 0, 3] = float(i)

        rpe = compute_rpe(poses, poses)
        assert rpe["trans_rmse"] < 1e-10
        assert rpe["rot_rmse"] < 1e-10

    def test_recall_at_k_perfect(self):
        """Recall@1 is 1.0 when queries match themselves"""
        from src.evaluation.metrics import recall_at_k

        descriptors = np.eye(5)
        positions = np.array([
            [0, 0], [10, 0], [20, 0], [30, 0], [40, 0],
        ], dtype=float)

        recall = recall_at_k(
            descriptors, descriptors,
            positions, positions,
            k_values=[1],
            threshold=5.0,
        )
        assert recall["recall@1"] == 1.0

    def test_recall_at_k_no_matches(self):
        """recall 0 when no true positives in DB"""
        from src.evaluation.metrics import recall_at_k

        desc_q = np.array([[1.0, 0.0]])
        desc_db = np.array([[0.0, 1.0]])
        pos_q = np.array([[0.0, 0.0]])
        pos_db = np.array([[100.0, 100.0]])  # very far

        recall = recall_at_k(
            desc_q, desc_db, pos_q, pos_db,
            k_values=[1], threshold=5.0,
        )
        assert recall["recall@1"] == 0.0

    def test_precision_recall_curve(self):
        """precision-recall curve computation"""
        from src.evaluation.metrics import precision_recall_curve

        sims = np.array([0.9, 0.8, 0.7, 0.6, 0.5, 0.4])
        labels = np.array([1, 1, 0, 1, 0, 0])

        prec, rec, thresholds = precision_recall_curve(sims, labels)
        assert len(prec) == len(rec) == len(thresholds)
        # highest threshold: first true positive
        assert prec[0] == 1.0

    def test_pose_error(self):
        """single pose error computation"""
        from src.evaluation.metrics import pose_error

        T1 = np.eye(4)
        T2 = np.eye(4)
        T2[0, 3] = 3.0
        T2[1, 3] = 4.0  # translation = 5m

        t_err, r_err = pose_error(T1, T2)
        assert abs(t_err - 5.0) < 1e-10
        assert abs(r_err) < 1e-10  # no rotation


# ---------------------------------------------------------------------------
# Tests for transforms
# ---------------------------------------------------------------------------


class TestTransforms:
    """point cloud transform tests"""

    def test_random_rotation_shape(self):
        """rotation preserves shape"""
        from src.datasets.transforms import RandomRotation

        points = np.random.randn(100, 4).astype(np.float32)
        transform = RandomRotation(max_angle=180)
        result = transform(points)
        assert result.shape == points.shape

    def test_random_subsample(self):
        """subsample to target count"""
        from src.datasets.transforms import RandomSubsample

        points = np.random.randn(1000, 4).astype(np.float32)
        transform = RandomSubsample(num_points=100)
        result = transform(points)
        assert result.shape == (100, 4)

    def test_random_subsample_small_input(self):
        """no expansion on smaller clouds"""
        from src.datasets.transforms import RandomSubsample

        points = np.random.randn(50, 4).astype(np.float32)
        transform = RandomSubsample(num_points=100)
        result = transform(points)
        assert result.shape == (50, 4)

    def test_voxel_downsample(self):
        """voxel downsample reduces count"""
        from src.datasets.transforms import VoxelDownsample

        points = np.random.randn(1000, 4).astype(np.float32)
        transform = VoxelDownsample(voxel_size=1.0)
        result = transform(points)
        assert len(result) < len(points)
        assert result.shape[1] == 4

    def test_remove_ground(self):
        """ground removal filters low points"""
        from src.datasets.transforms import RemoveGround

        points = np.array([
            [0, 0, 2, 1],    # above
            [0, 0, -2, 1],   # below threshold
            [0, 0, 0, 1],    # above
            [0, 0, -3, 1],   # below threshold
        ], dtype=np.float32)
        transform = RemoveGround(threshold=-1.5)
        result = transform(points)
        assert len(result) == 2  # only points above -1.5

    def test_compose(self):
        """composed transforms apply sequentially"""
        from src.datasets.transforms import Compose, RandomSubsample, RemoveGround

        points = np.random.randn(1000, 4).astype(np.float32)
        transform = Compose([
            RemoveGround(threshold=0.0),
            RandomSubsample(num_points=50),
        ])
        result = transform(points)
        assert len(result) <= 50

    def test_jitter_changes_values(self):
        """jitter modifies point positions"""
        from src.datasets.transforms import RandomJitter

        np.random.seed(42)
        points = np.ones((100, 4), dtype=np.float32)
        transform = RandomJitter(sigma=0.1)
        result = transform(points)
        # values changed by noise
        assert not np.allclose(result[:, :3], points[:, :3])
        # intensity unchanged
        np.testing.assert_array_equal(result[:, 3], points[:, 3])
