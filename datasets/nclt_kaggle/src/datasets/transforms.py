"""composable point cloud transforms for data augmentation"""

from __future__ import annotations

import logging
from typing import Protocol

import numpy as np

logger = logging.getLogger(__name__)


class PointCloudTransform(Protocol):
    """protocol for point cloud transforms"""

    def __call__(self, points: np.ndarray) -> np.ndarray: ...


class Compose:
    """compose multiple transforms sequentially"""

    def __init__(self, transforms: list[PointCloudTransform]) -> None:
        self.transforms = transforms

    def __call__(self, points: np.ndarray) -> np.ndarray:
        for t in self.transforms:
            points = t(points)
        return points

    def __repr__(self) -> str:
        lines = [f"  {t}" for t in self.transforms]
        return f"Compose([\n" + "\n".join(lines) + "\n])"


class RandomRotation:
    """random rotation around Z-axis"""

    def __init__(self, max_angle: float = 180.0) -> None:
        self.max_angle = max_angle

    def __call__(self, points: np.ndarray) -> np.ndarray:
        angle = np.random.uniform(-self.max_angle, self.max_angle)
        angle_rad = np.deg2rad(angle)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        rotation = np.array([
            [cos_a, -sin_a, 0],
            [sin_a, cos_a, 0],
            [0, 0, 1],
        ], dtype=np.float32)

        result = points.copy()
        result[:, :3] = points[:, :3] @ rotation.T
        return result

    def __repr__(self) -> str:
        return f"RandomRotation(max_angle={self.max_angle})"


class RandomFlip:
    """random flip along X or Y axis"""

    def __init__(self, prob: float = 0.5) -> None:
        self.prob = prob

    def __call__(self, points: np.ndarray) -> np.ndarray:
        result = points.copy()
        if np.random.random() < self.prob:
            result[:, 0] = -result[:, 0]
        if np.random.random() < self.prob:
            result[:, 1] = -result[:, 1]
        return result

    def __repr__(self) -> str:
        return f"RandomFlip(prob={self.prob})"


class RandomJitter:
    """add gaussian noise to point coordinates"""

    def __init__(self, sigma: float = 0.01, clip: float = 0.05) -> None:
        self.sigma = sigma
        self.clip = clip

    def __call__(self, points: np.ndarray) -> np.ndarray:
        result = points.copy()
        noise = np.clip(
            np.random.normal(0, self.sigma, size=(len(points), 3)),
            -self.clip,
            self.clip,
        ).astype(np.float32)
        result[:, :3] += noise
        return result

    def __repr__(self) -> str:
        return f"RandomJitter(sigma={self.sigma}, clip={self.clip})"


class RandomSubsample:
    """randomly subsample to fixed point count"""

    def __init__(self, num_points: int = 50000) -> None:
        self.num_points = num_points

    def __call__(self, points: np.ndarray) -> np.ndarray:
        n = len(points)
        if n <= self.num_points:
            return points
        indices = np.random.choice(n, self.num_points, replace=False)
        return points[indices]

    def __repr__(self) -> str:
        return f"RandomSubsample(num_points={self.num_points})"


class VoxelDownsample:
    """voxel grid downsampling"""

    def __init__(self, voxel_size: float = 0.1) -> None:
        self.voxel_size = voxel_size

    def __call__(self, points: np.ndarray) -> np.ndarray:
        coords = points[:, :3]
        voxel_indices = np.floor(coords / self.voxel_size).astype(np.int32)

        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return points[np.sort(unique_idx)]

    def __repr__(self) -> str:
        return f"VoxelDownsample(voxel_size={self.voxel_size})"


class Normalize:
    """center at origin and optionally scale to unit sphere"""

    def __init__(self, center: bool = True, scale: bool = False) -> None:
        self.center = center
        self.scale = scale

    def __call__(self, points: np.ndarray) -> np.ndarray:
        result = points.copy()
        if self.center:
            centroid = result[:, :3].mean(axis=0)
            result[:, :3] -= centroid
        if self.scale:
            max_dist = np.linalg.norm(result[:, :3], axis=1).max()
            if max_dist > 0:
                result[:, :3] /= max_dist
        return result

    def __repr__(self) -> str:
        return f"Normalize(center={self.center}, scale={self.scale})"


class RemoveGround:
    """remove ground points by Z-height threshold"""

    def __init__(self, threshold: float = -1.5) -> None:
        self.threshold = threshold

    def __call__(self, points: np.ndarray) -> np.ndarray:
        mask = points[:, 2] > self.threshold
        return points[mask]

    def __repr__(self) -> str:
        return f"RemoveGround(threshold={self.threshold})"


def build_transforms(config: dict, is_train: bool = True) -> Compose:
    """build transform pipeline from config dict"""
    transforms: list[PointCloudTransform] = []

    pc_config = config.get("point_cloud", {})
    aug_config = config.get("augmentation", {})

    if pc_config.get("remove_ground", False):
        transforms.append(RemoveGround(pc_config.get("ground_threshold", -1.5)))

    if pc_config.get("voxel_size"):
        transforms.append(VoxelDownsample(pc_config["voxel_size"]))

    if is_train:
        if aug_config.get("random_rotation", False):
            max_angle = aug_config.get("rotation_range", 180.0)
            transforms.append(RandomRotation(max_angle))
        if aug_config.get("random_flip", False):
            transforms.append(RandomFlip())
        if aug_config.get("jitter"):
            transforms.append(RandomJitter(sigma=aug_config["jitter"]))

    max_points = pc_config.get("max_points", 50000)
    transforms.append(RandomSubsample(max_points))

    logger.debug("Built transform pipeline: %s", transforms)
    return Compose(transforms)
