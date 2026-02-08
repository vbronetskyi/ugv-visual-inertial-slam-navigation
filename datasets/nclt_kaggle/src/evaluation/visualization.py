"""SLAM evaluation visualization: trajectories, errors, recall, precision-recall"""

from __future__ import annotations

import logging
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)


def plot_trajectory_2d(
    trajectories: dict[str, np.ndarray],
    output_path: str | Path | None = None,
    title: str = "Trajectory Comparison",
    figsize: tuple[int, int] = (10, 10),
) -> plt.Figure:
    """plot 2D bird's-eye-view trajectory comparison"""
    fig, ax = plt.subplots(1, 1, figsize=figsize)

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    for i, (label, poses) in enumerate(trajectories.items()):
        positions = poses[:, :3, 3]
        color = colors[i % len(colors)]
        ax.plot(positions[:, 0], positions[:, 1], label=label, color=color, linewidth=1.5)
        ax.scatter(positions[0, 0], positions[0, 1], marker="o", color=color, s=100, zorder=5)
        ax.scatter(positions[-1, 0], positions[-1, 1], marker="x", color=color, s=100, zorder=5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        logger.info("Saved trajectory plot to %s", output_path)

    return fig


def plot_trajectory_3d(
    trajectories: dict[str, np.ndarray],
    output_path: str | Path | None = None,
    title: str = "3D Trajectory",
) -> plt.Figure:
    """plot 3D trajectory comparison"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    for label, poses in trajectories.items():
        positions = poses[:, :3, 3]
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label=label)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)
    ax.legend()

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        logger.info("Saved 3D trajectory plot to %s", output_path)

    return fig


def plot_error_over_time(
    errors: np.ndarray,
    timestamps: np.ndarray | None = None,
    output_path: str | Path | None = None,
    title: str = "Trajectory Error Over Time",
    ylabel: str = "Error (m)",
) -> plt.Figure:
    """plot per-frame error over time"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 4))

    x = timestamps if timestamps is not None else np.arange(len(errors))
    xlabel = "Time (s)" if timestamps is not None else "Frame"

    ax.plot(x, errors, linewidth=0.8)
    ax.axhline(y=np.mean(errors), color="r", linestyle="--", alpha=0.7, label=f"Mean: {np.mean(errors):.3f}")
    ax.fill_between(x, 0, errors, alpha=0.2)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")

    return fig


def plot_recall_at_k(
    recall_values: dict[str, float],
    output_path: str | Path | None = None,
    title: str = "Place Recognition Recall@K",
) -> plt.Figure:
    """plot recall@K bar chart"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 5))

    labels = list(recall_values.keys())
    values = list(recall_values.values())

    bars = ax.bar(labels, values, color="#1f77b4", alpha=0.8, edgecolor="black")

    for bar, val in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{val:.3f}",
            ha="center",
            fontsize=11,
        )

    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Recall")
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")

    return fig


def plot_precision_recall(
    precision: np.ndarray,
    recall: np.ndarray,
    output_path: str | Path | None = None,
    title: str = "Precision-Recall Curve (Loop Closure)",
) -> plt.Figure:
    """plot precision-recall curve for loop closure"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))

    ax.plot(recall, precision, linewidth=2)
    ax.fill_between(recall, 0, precision, alpha=0.15)

    # compute AUC
    auc = np.trapz(precision, recall)
    ax.set_xlabel("Recall")
    ax.set_ylabel("Precision")
    ax.set_title(f"{title} (AUC={auc:.3f})")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")

    return fig


def plot_loop_closures_on_map(
    poses: np.ndarray,
    loop_pairs: list[tuple[int, int]],
    output_path: str | Path | None = None,
    title: str = "Detected Loop Closures",
) -> plt.Figure:
    """plot loop closure detections on 2D trajectory map"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    positions = poses[:, :3, 3]
    ax.plot(positions[:, 0], positions[:, 1], "b-", alpha=0.5, linewidth=1)

    for src, tgt in loop_pairs:
        ax.plot(
            [positions[src, 0], positions[tgt, 0]],
            [positions[src, 1], positions[tgt, 1]],
            "r-",
            alpha=0.3,
            linewidth=0.5,
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{title} ({len(loop_pairs)} closures)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=150, bbox_inches="tight")

    return fig
