"""LiDAR odometry using ICP point cloud registration via Open3D"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

try:
    import open3d as o3d

    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    logger.warning("Open3D not installed. LiDAR odometry unavailable.")


@dataclass
class OdometryConfig:
    """LiDAR odometry configuration"""

    method: str = "icp"
    max_iterations: int = 50
    convergence_threshold: float = 1e-6
    max_correspondence_distance: float = 1.0
    voxel_size: float = 0.2
    keyframe_distance: float = 2.0
    keyframe_rotation: float = 15.0

    @classmethod
    def from_dict(cls, config: dict) -> OdometryConfig:
        return cls(**{k: v for k, v in config.items() if k in cls.__dataclass_fields__})


@dataclass
class OdometryResult:
    """odometry estimation result"""

    poses: list[np.ndarray] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    keyframe_indices: list[int] = field(default_factory=list)


class LiDARodometry:
    """frame-to-frame LiDAR odometry using ICP registration"""

    def __init__(self, config: OdometryConfig | None = None) -> None:
        if not HAS_OPEN3D:
            raise RuntimeError("Open3D is required for LiDAR odometry.")

        self.config = config or OdometryConfig()
        self._current_pose = np.eye(4, dtype=np.float64)
        self._prev_pcd: o3d.geometry.PointCloud | None = None
        self._keyframe_pcd: o3d.geometry.PointCloud | None = None
        self._keyframe_pose = np.eye(4, dtype=np.float64)
        self._result = OdometryResult()
        self._frame_count = 0

        logger.info(
            "LiDAR odometry initialized with method=%s, voxel_size=%.2f",
            self.config.method,
            self.config.voxel_size,
        )

    def process_frame(
        self, points: np.ndarray, timestamp: float = 0.0
    ) -> np.ndarray:
        """process single frame, return estimated 4x4 world pose"""
        pcd = self._preprocess(points)

        if self._prev_pcd is None:
            # first frame
            self._prev_pcd = pcd
            self._keyframe_pcd = pcd
            self._result.poses.append(self._current_pose.copy())
            self._result.timestamps.append(timestamp)
            self._result.keyframe_indices.append(0)
            self._frame_count += 1
            return self._current_pose.copy()

        # register against previous frame
        relative_transform = self._register(pcd, self._prev_pcd)
        self._current_pose = self._current_pose @ relative_transform

        self._result.poses.append(self._current_pose.copy())
        self._result.timestamps.append(timestamp)

        # check keyframe
        if self._is_keyframe():
            self._keyframe_pcd = pcd
            self._keyframe_pose = self._current_pose.copy()
            self._result.keyframe_indices.append(self._frame_count)
            logger.debug("New keyframe at frame %d", self._frame_count)

        self._prev_pcd = pcd
        self._frame_count += 1
        return self._current_pose.copy()

    def _preprocess(self, points: np.ndarray) -> o3d.geometry.PointCloud:
        """downsample and estimate normals"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

        # downsample
        pcd = pcd.voxel_down_sample(self.config.voxel_size)

        # normals for GICP
        if self.config.method in ("gicp", "ndt"):
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.config.voxel_size * 3, max_nn=30
                )
            )

        return pcd

    def _register(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
    ) -> np.ndarray:
        """register source to target, return 4x4 relative transform"""
        init_transform = np.eye(4, dtype=np.float64)

        if self.config.method == "icp":
            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                self.config.max_correspondence_distance,
                init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.max_iterations,
                    relative_rmse=self.config.convergence_threshold,
                ),
            )
        elif self.config.method == "gicp":
            result = o3d.pipelines.registration.registration_generalized_icp(
                source,
                target,
                self.config.max_correspondence_distance,
                init_transform,
                o3d.pipelines.registration.
                TransformationEstimationForGeneralizedICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.max_iterations,
                    relative_rmse=self.config.convergence_threshold,
                ),
            )
        else:
            # fallback to point-to-point ICP
            logger.warning("Unknown method '%s', falling back to ICP.", self.config.method)
            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                self.config.max_correspondence_distance,
                init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            )

        logger.debug(
            "Registration fitness=%.4f, RMSE=%.4f",
            result.fitness,
            result.inlier_rmse,
        )

        return result.transformation

    def _is_keyframe(self) -> bool:
        """check distance/rotation from last keyframe"""
        delta = np.linalg.inv(self._keyframe_pose) @ self._current_pose
        trans = np.linalg.norm(delta[:3, 3])

        # rotation angle
        R = delta[:3, :3]
        cos_angle = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))

        return (
            trans > self.config.keyframe_distance
            or angle_deg > self.config.keyframe_rotation
        )

    def get_result(self) -> OdometryResult:
        return self._result

    def get_trajectory(self) -> np.ndarray:
        """return trajectory as (N, 4, 4) SE3 poses"""
        if not self._result.poses:
            return np.empty((0, 4, 4), dtype=np.float64)
        return np.array(self._result.poses)

    def reset(self) -> None:
        self._current_pose = np.eye(4, dtype=np.float64)
        self._prev_pcd = None
        self._keyframe_pcd = None
        self._keyframe_pose = np.eye(4, dtype=np.float64)
        self._result = OdometryResult()
        self._frame_count = 0
        logger.info("Odometry state reset.")
