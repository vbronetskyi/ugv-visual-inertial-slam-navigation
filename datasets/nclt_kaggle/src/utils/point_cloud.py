"""point cloud processing: loading, filtering, transformation for NCLT Velodyne data"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import open3d as o3d

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def load_velodyne_bin(path: str | Path) -> np.ndarray:
    """load NCLT .bin point cloud as (N, 4) float32 [x, y, z, intensity]"""
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {path}")

    file_size = path.stat().st_size

    if file_size == 0:
        raise ValueError(f"Point cloud file is empty: {path}")

    if file_size % 16 != 0:
        raise ValueError(
            f"Corrupted point cloud file (size {file_size} bytes is not a "
            f"multiple of 16): {path}"
        )

    try:
        points = np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)
    except Exception as exc:
        raise ValueError(f"Failed to parse point cloud file {path}: {exc}") from exc

    logger.debug("Loaded %d points from %s", len(points), path.name)
    return points


# ---------------------------------------------------------------------------
# Downsampling
# ---------------------------------------------------------------------------


def voxel_downsample(points: np.ndarray, voxel_size: float = 0.1) -> np.ndarray:
    """voxel grid downsampling (Open3D if available, NumPy fallback)"""
    if voxel_size <= 0:
        raise ValueError(f"voxel_size must be positive, got {voxel_size}")

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    try:
        import open3d as o3d  # noqa: F811

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
        pcd_down = pcd.voxel_down_sample(voxel_size)
        indices = _closest_point_indices(points[:, :3], np.asarray(pcd_down.points))
        result = points[indices]
        logger.debug(
            "Voxel downsample (Open3D): %d -> %d points (voxel=%.3f m)",
            len(points),
            len(result),
            voxel_size,
        )
        return result
    except ImportError:
        logger.debug("Open3D not available; using NumPy voxel downsample fallback")

    # numpy fallback: quantise to voxel indices
    voxel_indices = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    unique_idx.sort()  # preserve original ordering
    result = points[unique_idx]
    logger.debug(
        "Voxel downsample (NumPy): %d -> %d points (voxel=%.3f m)",
        len(points),
        len(result),
        voxel_size,
    )
    return result


def _closest_point_indices(
    source: np.ndarray, targets: np.ndarray
) -> np.ndarray:
    """return source indices of nearest neighbour for each target point"""
    from scipy.spatial import cKDTree

    tree = cKDTree(source)
    _, indices = tree.query(targets)
    return np.unique(indices)


# ---------------------------------------------------------------------------
# Ground removal
# ---------------------------------------------------------------------------


def remove_ground_plane(
    points: np.ndarray,
    threshold: float = -1.5,
    method: str = "height",
) -> np.ndarray:
    """remove ground points via height threshold or RANSAC"""
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    if method == "height":
        mask = points[:, 2] >= threshold
        result = points[mask]
        logger.debug(
            "Ground removal (height z>=%.2f): %d -> %d points",
            threshold,
            len(points),
            len(result),
        )
        return result

    if method == "ransac":
        try:
            import open3d as o3d  # noqa: F811
        except ImportError as exc:
            raise ImportError(
                "Open3D is required for RANSAC ground removal but is not installed."
            ) from exc

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

        # RANSAC plane segmentation
        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=threshold if threshold > 0 else 0.2,
            ransac_n=3,
            num_iterations=1000,
        )
        a, b, c, d = plane_model
        logger.debug(
            "RANSAC ground plane: %.3fx + %.3fy + %.3fz + %.3f = 0 "
            "(%d inliers)",
            a,
            b,
            c,
            d,
            len(inlier_indices),
        )

        inlier_set = set(inlier_indices)
        mask = np.array(
            [i not in inlier_set for i in range(len(points))], dtype=bool
        )
        result = points[mask]
        logger.debug(
            "Ground removal (RANSAC dist=%.3f): %d -> %d points",
            threshold,
            len(points),
            len(result),
        )
        return result

    raise ValueError(
        f"Unknown ground removal method '{method}'. "
        f"Supported: 'height', 'ransac'."
    )


# ---------------------------------------------------------------------------
# Transformation
# ---------------------------------------------------------------------------


def transform_point_cloud(
    points: np.ndarray, transform: np.ndarray
) -> np.ndarray:
    """apply (4, 4) SE3 transform to point cloud, preserving extra columns"""
    if transform.shape != (4, 4):
        raise ValueError(
            f"Transform must be (4, 4), got {transform.shape}"
        )

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    xyz = points[:, :3]
    # R @ p + t  for each point (vectorised).
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    xyz_transformed = (rotation @ xyz.T).T + translation

    if points.shape[1] == 4:
        result = np.column_stack([xyz_transformed, points[:, 3]])
    else:
        result = xyz_transformed

    return result.astype(points.dtype)


# ---------------------------------------------------------------------------
# Normal estimation
# ---------------------------------------------------------------------------


def compute_normals(
    points: np.ndarray, k_neighbors: int = 20
) -> np.ndarray:
    """estimate (N, 3) surface normals using Open3D KNN"""
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for normal estimation but is not installed."
        ) from exc

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) < 3:
        raise ValueError(
            f"Need at least 3 points for normal estimation, got {len(points)}"
        )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k_neighbors)
    )
    normals = np.asarray(pcd.normals)

    logger.debug("Computed normals for %d points (k=%d)", len(points), k_neighbors)
    return normals


# ---------------------------------------------------------------------------
# Cropping
# ---------------------------------------------------------------------------


def crop_point_cloud(
    points: np.ndarray,
    min_bound: np.ndarray,
    max_bound: np.ndarray,
) -> np.ndarray:
    """crop point cloud to axis-aligned bounding box"""
    min_bound = np.asarray(min_bound, dtype=np.float64).ravel()
    max_bound = np.asarray(max_bound, dtype=np.float64).ravel()

    if min_bound.shape != (3,) or max_bound.shape != (3,):
        raise ValueError(
            f"Bounds must be length-3 vectors, got min={min_bound.shape}, "
            f"max={max_bound.shape}"
        )

    if np.any(min_bound > max_bound):
        raise ValueError(
            f"min_bound must be <= max_bound element-wise. "
            f"min={min_bound}, max={max_bound}"
        )

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    xyz = points[:, :3]
    mask = np.all((xyz >= min_bound) & (xyz <= max_bound), axis=1)
    result = points[mask]

    logger.debug(
        "Crop: %d -> %d points (bounds min=%s max=%s)",
        len(points),
        len(result),
        min_bound,
        max_bound,
    )
    return result


# ---------------------------------------------------------------------------
# Random subsampling
# ---------------------------------------------------------------------------


def random_subsample(
    points: np.ndarray, max_points: int = 50_000
) -> np.ndarray:
    """randomly subsample to at most max_points"""
    if max_points <= 0:
        raise ValueError(f"max_points must be positive, got {max_points}")

    if len(points) <= max_points:
        return points.copy()

    rng = np.random.default_rng()
    indices = rng.choice(len(points), size=max_points, replace=False)
    indices.sort()  # preserve original ordering
    result = points[indices]

    logger.debug(
        "Random subsample: %d -> %d points", len(points), len(result)
    )
    return result


# ---------------------------------------------------------------------------
# Open3D conversions
# ---------------------------------------------------------------------------


def to_open3d(points: np.ndarray) -> "o3d.geometry.PointCloud":
    """convert (N, 3|4) numpy array to Open3D PointCloud"""
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for point cloud conversion but is not "
            "installed."
        ) from exc

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

    logger.debug("Converted %d points to Open3D PointCloud", len(points))
    return pcd


def from_open3d(pcd: "o3d.geometry.PointCloud") -> np.ndarray:
    """convert Open3D PointCloud to (N, 3) numpy array"""
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for point cloud conversion but is not "
            "installed."
        ) from exc

    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError(
            f"Expected open3d.geometry.PointCloud, got {type(pcd).__name__}"
        )

    points = np.asarray(pcd.points)
    logger.debug("Converted Open3D PointCloud to %d points", len(points))
    return points
