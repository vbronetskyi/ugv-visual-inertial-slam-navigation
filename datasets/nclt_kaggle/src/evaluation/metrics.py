"""SLAM and place recognition evaluation metrics: ATE, RPE, Recall@K, precision-recall"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------


def pose_error(
    pose_est: np.ndarray,
    pose_gt: np.ndarray,
) -> tuple[float, float]:
    """return (translation_error_m, rotation_error_deg) between two (4,4) SE3 poses"""
    pose_est = np.asarray(pose_est, dtype=np.float64)
    pose_gt = np.asarray(pose_gt, dtype=np.float64)

    if pose_est.shape != (4, 4):
        raise ValueError(
            f"pose_est must have shape (4, 4), got {pose_est.shape}"
        )
    if pose_gt.shape != (4, 4):
        raise ValueError(
            f"pose_gt must have shape (4, 4), got {pose_gt.shape}"
        )

    # relative transform: error = gt^{-1} @ est
    R_est = pose_est[:3, :3]
    t_est = pose_est[:3, 3]
    R_gt = pose_gt[:3, :3]
    t_gt = pose_gt[:3, 3]

    # translation error
    trans_err = float(np.linalg.norm(t_est - t_gt))

    # rotation error
    R_diff = R_gt.T @ R_est
    # clamp trace to valid acos range
    cos_angle = (np.trace(R_diff) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    rot_err_rad = float(np.arccos(cos_angle))
    rot_err_deg = float(np.degrees(rot_err_rad))

    return trans_err, rot_err_deg


# ---------------------------------------------------------------------------
# Umeyama alignment
# ---------------------------------------------------------------------------


def umeyama_alignment(
    estimated: np.ndarray,
    ground_truth: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Umeyama alignment (with scale) of two (N,3) point sets, returns (R, t, s)"""
    estimated = np.asarray(estimated, dtype=np.float64)
    ground_truth = np.asarray(ground_truth, dtype=np.float64)

    if estimated.ndim != 2 or estimated.shape[1] != 3:
        raise ValueError(
            f"estimated must have shape (N, 3), got {estimated.shape}"
        )
    if ground_truth.ndim != 2 or ground_truth.shape[1] != 3:
        raise ValueError(
            f"ground_truth must have shape (N, 3), got {ground_truth.shape}"
        )
    if estimated.shape[0] != ground_truth.shape[0]:
        raise ValueError(
            f"Point set sizes differ: estimated has {estimated.shape[0]} "
            f"points, ground_truth has {ground_truth.shape[0]} points"
        )

    n = estimated.shape[0]
    if n < 3:
        raise ValueError(
            f"At least 3 points are required for Umeyama alignment, got {n}"
        )

    # centroids
    mu_est = estimated.mean(axis=0)
    mu_gt = ground_truth.mean(axis=0)

    # centred coordinates
    est_c = estimated - mu_est
    gt_c = ground_truth - mu_gt

    # variances
    sigma_est_sq = np.mean(np.sum(est_c ** 2, axis=1))

    # cross-covariance
    sigma = (gt_c.T @ est_c) / n

    U, D, Vt = np.linalg.svd(sigma)

    # ensure proper rotation (det = +1)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    s = float(np.trace(np.diag(D) @ S) / sigma_est_sq)
    t = mu_gt - s * R @ mu_est

    return R, t, s


# ---------------------------------------------------------------------------
# Absolute Trajectory Error (ATE)
# ---------------------------------------------------------------------------


def compute_ate(
    estimated: np.ndarray,
    ground_truth: np.ndarray,
) -> dict[str, float]:
    """compute ATE (rmse/mean/median/std/max) after Umeyama alignment of (N,4,4) pose arrays"""
    estimated = np.asarray(estimated, dtype=np.float64)
    ground_truth = np.asarray(ground_truth, dtype=np.float64)

    # --- input validation ---------------------------------------------------
    if estimated.size == 0 or ground_truth.size == 0:
        raise ValueError("Input pose arrays must not be empty.")

    if estimated.ndim != 3 or estimated.shape[1:] != (4, 4):
        raise ValueError(
            f"estimated must have shape (N, 4, 4), got {estimated.shape}"
        )
    if ground_truth.ndim != 3 or ground_truth.shape[1:] != (4, 4):
        raise ValueError(
            f"ground_truth must have shape (N, 4, 4), got {ground_truth.shape}"
        )
    if estimated.shape[0] != ground_truth.shape[0]:
        raise ValueError(
            f"Trajectory lengths differ: estimated has {estimated.shape[0]} "
            f"poses, ground_truth has {ground_truth.shape[0]} poses"
        )

    n = estimated.shape[0]
    if n < 3:
        raise ValueError(
            f"At least 3 poses are required for ATE computation, got {n}"
        )

    # extract positions (N, 3)
    est_pos = estimated[:, :3, 3]
    gt_pos = ground_truth[:, :3, 3]

    # align estimated to ground truth
    R, t, s = umeyama_alignment(est_pos, gt_pos)
    est_pos_aligned = (s * (R @ est_pos.T)).T + t

    # per-pose translational errors
    errors = np.linalg.norm(est_pos_aligned - gt_pos, axis=1)

    rmse = float(np.sqrt(np.mean(errors ** 2)))
    mean = float(np.mean(errors))
    median = float(np.median(errors))
    std = float(np.std(errors))
    max_err = float(np.max(errors))

    logger.debug(
        "ATE (n=%d): RMSE=%.4f, mean=%.4f, median=%.4f, std=%.4f, max=%.4f",
        n,
        rmse,
        mean,
        median,
        std,
        max_err,
    )

    return {
        "rmse": rmse,
        "mean": mean,
        "median": median,
        "std": std,
        "max": max_err,
    }


# ---------------------------------------------------------------------------
# Relative Pose Error (RPE)
# ---------------------------------------------------------------------------


def compute_rpe(
    estimated: np.ndarray,
    ground_truth: np.ndarray,
    delta: int = 1,
) -> dict[str, float]:
    """compute RPE (trans/rot rmse/mean) for (N,4,4) pose arrays with given step delta"""
    estimated = np.asarray(estimated, dtype=np.float64)
    ground_truth = np.asarray(ground_truth, dtype=np.float64)

    # --- input validation ---------------------------------------------------
    if estimated.size == 0 or ground_truth.size == 0:
        raise ValueError("Input pose arrays must not be empty.")

    if estimated.ndim != 3 or estimated.shape[1:] != (4, 4):
        raise ValueError(
            f"estimated must have shape (N, 4, 4), got {estimated.shape}"
        )
    if ground_truth.ndim != 3 or ground_truth.shape[1:] != (4, 4):
        raise ValueError(
            f"ground_truth must have shape (N, 4, 4), got {ground_truth.shape}"
        )
    if estimated.shape[0] != ground_truth.shape[0]:
        raise ValueError(
            f"Trajectory lengths differ: estimated has {estimated.shape[0]} "
            f"poses, ground_truth has {ground_truth.shape[0]} poses"
        )
    if not isinstance(delta, int) or delta < 1:
        raise ValueError(f"delta must be a positive integer, got {delta}")

    n = estimated.shape[0]
    if n <= delta:
        raise ValueError(
            f"Trajectory length ({n}) must be greater than delta ({delta})"
        )

    trans_errors: list[float] = []
    rot_errors: list[float] = []

    for i in range(n - delta):
        # rel_est = est[i]^{-1} @ est[i+delta]
        est_inv = np.linalg.inv(estimated[i])
        gt_inv = np.linalg.inv(ground_truth[i])

        rel_est = est_inv @ estimated[i + delta]
        rel_gt = gt_inv @ ground_truth[i + delta]

        # error between relative transforms
        t_err, r_err = pose_error(rel_est, rel_gt)
        trans_errors.append(t_err)
        rot_errors.append(r_err)

    trans_errors_arr = np.array(trans_errors)
    rot_errors_arr = np.array(rot_errors)

    result = {
        "trans_rmse": float(np.sqrt(np.mean(trans_errors_arr ** 2))),
        "trans_mean": float(np.mean(trans_errors_arr)),
        "rot_rmse": float(np.sqrt(np.mean(rot_errors_arr ** 2))),
        "rot_mean": float(np.mean(rot_errors_arr)),
    }

    logger.debug(
        "RPE (n=%d, delta=%d): trans_rmse=%.4f, rot_rmse=%.4f deg",
        n,
        delta,
        result["trans_rmse"],
        result["rot_rmse"],
    )

    return result


# ---------------------------------------------------------------------------
# Place Recognition – Recall@K
# ---------------------------------------------------------------------------


def recall_at_k(
    descriptors_query: np.ndarray,
    descriptors_db: np.ndarray,
    positions_query: np.ndarray,
    positions_db: np.ndarray,
    k_values: list[int] | None = None,
    threshold: float = 10.0,
) -> dict[str, float]:
    """compute Recall@K: fraction of queries with correct match in top-K"""
    if k_values is None:
        k_values = [1, 5, 10]

    descriptors_query = np.asarray(descriptors_query, dtype=np.float64)
    descriptors_db = np.asarray(descriptors_db, dtype=np.float64)
    positions_query = np.asarray(positions_query, dtype=np.float64)
    positions_db = np.asarray(positions_db, dtype=np.float64)

    # --- input validation ---------------------------------------------------
    if descriptors_query.size == 0 or descriptors_db.size == 0:
        raise ValueError("Descriptor arrays must not be empty.")
    if positions_query.size == 0 or positions_db.size == 0:
        raise ValueError("Position arrays must not be empty.")

    if descriptors_query.ndim != 2 or descriptors_db.ndim != 2:
        raise ValueError(
            "Descriptors must be 2-D arrays (num_samples, descriptor_dim)."
        )
    if descriptors_query.shape[1] != descriptors_db.shape[1]:
        raise ValueError(
            f"Descriptor dimensions differ: query has "
            f"{descriptors_query.shape[1]}, db has {descriptors_db.shape[1]}"
        )
    if descriptors_query.shape[0] != positions_query.shape[0]:
        raise ValueError(
            f"Number of query descriptors ({descriptors_query.shape[0]}) does "
            f"not match number of query positions ({positions_query.shape[0]})"
        )
    if descriptors_db.shape[0] != positions_db.shape[0]:
        raise ValueError(
            f"Number of db descriptors ({descriptors_db.shape[0]}) does not "
            f"match number of db positions ({positions_db.shape[0]})"
        )

    num_queries = descriptors_query.shape[0]
    num_db = descriptors_db.shape[0]

    # pairwise distances
    desc_dists = cdist(descriptors_query, descriptors_db, metric="euclidean")
    pos_dists = cdist(positions_query, positions_db, metric="euclidean")

    # sort by descriptor distance
    sorted_indices = np.argsort(desc_dists, axis=1)

    results: dict[str, float] = {}
    max_k = max(k_values)

    for k in k_values:
        effective_k = min(k, num_db)
        correct = 0
        for q in range(num_queries):
            top_k_indices = sorted_indices[q, :effective_k]
            # check if any match within threshold
            min_pos_dist = np.min(pos_dists[q, top_k_indices])
            if min_pos_dist <= threshold:
                correct += 1
        recall = correct / num_queries
        results[f"recall@{k}"] = recall

    logger.debug(
        "Recall@K (Q=%d, DB=%d, threshold=%.1f m): %s",
        num_queries,
        num_db,
        threshold,
        ", ".join(f"{k}={v:.4f}" for k, v in results.items()),
    )

    return results


# ---------------------------------------------------------------------------
# Precision-Recall Curve for Loop Closure
# ---------------------------------------------------------------------------


def precision_recall_curve(
    similarities: np.ndarray,
    labels: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """compute (precision, recall, thresholds) arrays by sweeping similarity thresholds"""
    similarities = np.asarray(similarities, dtype=np.float64).ravel()
    labels = np.asarray(labels, dtype=np.float64).ravel()

    # --- input validation ---------------------------------------------------
    if similarities.size == 0:
        raise ValueError("similarities array must not be empty.")
    if labels.size == 0:
        raise ValueError("labels array must not be empty.")
    if similarities.shape[0] != labels.shape[0]:
        raise ValueError(
            f"Length mismatch: similarities has {similarities.shape[0]} "
            f"elements, labels has {labels.shape[0]}"
        )

    unique_labels = np.unique(labels)
    if not np.all(np.isin(unique_labels, [0.0, 1.0])):
        raise ValueError(
            f"labels must contain only 0 and 1, found unique values: "
            f"{unique_labels.tolist()}"
        )

    total_positives = np.sum(labels == 1)
    if total_positives == 0:
        logger.warning(
            "No positive labels found; precision-recall curve is undefined."
        )
        return (
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )

    # sort by descending similarity
    sorted_indices = np.argsort(-similarities)
    sorted_labels = labels[sorted_indices]
    sorted_similarities = similarities[sorted_indices]

    # unique thresholds only
    unique_thresholds, first_indices = np.unique(
        sorted_similarities[::-1], return_index=True
    )
    # descending order
    unique_thresholds = unique_thresholds[::-1]

    precisions: list[float] = []
    recalls: list[float] = []
    thresholds_out: list[float] = []

    for thresh in unique_thresholds:
        predicted_positive = similarities >= thresh
        tp = np.sum(predicted_positive & (labels == 1))
        fp = np.sum(predicted_positive & (labels == 0))

        precision = tp / (tp + fp) if (tp + fp) > 0 else 1.0
        recall = tp / total_positives

        precisions.append(float(precision))
        recalls.append(float(recall))
        thresholds_out.append(float(thresh))

    precision_arr = np.array(precisions, dtype=np.float64)
    recall_arr = np.array(recalls, dtype=np.float64)
    thresholds_arr = np.array(thresholds_out, dtype=np.float64)

    logger.debug(
        "Precision-recall curve: %d thresholds, %d positives out of %d total",
        len(thresholds_out),
        int(total_positives),
        len(labels),
    )

    return precision_arr, recall_arr, thresholds_arr
