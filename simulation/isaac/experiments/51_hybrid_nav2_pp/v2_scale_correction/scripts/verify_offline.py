#!/usr/bin/env python3
"""Offline scale-correction verification on exp 51 recorded data.

Replays raw VIO + GT (as encoder proxy, ratio 0.9999 per exp 42).
Applies rolling-window scale correction, reports corrected ATE.

Inputs:
  vio_final_camera.txt   timestamp tx ty tz qx qy qz qw  (SLAM camera frame)
  groundtruth.csv        timestamp,x,y,z,yaw           (world frame)

Output:
  scale_correction_result.png  scale & error time series
  scale_correction_result.csv  per-frame (t, gt_x, gt_y, vio_x, vio_y, corr_x, corr_y, scale)
"""
import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

VIO_FILE = "/workspace/simulation/isaac/experiments/51_hybrid_nav2_pp/logs/vio_final_camera.txt"
GT_FILE  = "/root/bags/husky_real/isaac_slam_1776444185/groundtruth.csv"
OUT_DIR  = "/workspace/simulation/isaac/experiments/51_hybrid_nav2_pp/v2_scale_correction/results"


def load_vio(path):
    data = np.loadtxt(path)
    return data  # (N, 8): t, tx, ty, tz, qx, qy, qz, qw


def load_gt(path):
    gt = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(0, 1, 2, 4))
    return gt  # (N, 4): t, x, y, yaw


def vio_camera_to_nav(vio, gt0_x, gt0_y, gt0_yaw):
    """Transform raw SLAM camera poses to nav frame using SE(3) alignment.

    Replicates _slam_se3_to_nav from tf_wall_clock_relay.py.
    Returns (nav_x, nav_y, nav_yaw) arrays.
    """
    T_FLU_from_cam = np.array([
        [0,  0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0,  0, 0, 1],
    ], dtype=float)
    R_nav = Rotation.from_euler('z', gt0_yaw).as_matrix()
    T_nav_origin = np.eye(4)
    T_nav_origin[:3, :3] = R_nav
    T_nav_origin[:3, 3] = [gt0_x, gt0_y, 0.0]

    t0 = vio[0]
    R0 = Rotation.from_quat([t0[4], t0[5], t0[6], t0[7]]).as_matrix()
    T_slam0 = np.eye(4)
    T_slam0[:3, :3] = R0
    T_slam0[:3, 3] = t0[1:4]
    T_nav_slam = T_nav_origin @ T_FLU_from_cam @ np.linalg.inv(T_slam0)

    nav_x = np.zeros(len(vio))
    nav_y = np.zeros(len(vio))
    nav_yaw = np.zeros(len(vio))
    for i, row in enumerate(vio):
        Ri = Rotation.from_quat([row[4], row[5], row[6], row[7]]).as_matrix()
        Ti = np.eye(4)
        Ti[:3, :3] = Ri
        Ti[:3, 3] = row[1:4]
        Tn = T_nav_slam @ Ti
        nav_x[i] = Tn[0, 3]
        nav_y[i] = Tn[1, 3]
        nav_yaw[i] = math.atan2(Tn[1, 0], Tn[0, 0])
    return nav_x, nav_y, nav_yaw


class ScaleCorrector:
    """Rolling-window scale correction using encoder distance vs VIO distance."""
    def __init__(self, window_seconds=30.0, alpha=0.1,
                 min_vio_dist=1.0, scale_min=0.3, scale_max=3.0):
        self.window = window_seconds
        self.alpha = alpha
        self.min_vio_dist = min_vio_dist
        self.scale_min = scale_min
        self.scale_max = scale_max
        self.vio_positions = []
        self.enc_distances = []
        self.current_scale = 1.0
        self.enc_cumulative = 0.0
        self.prev_enc_x = None
        self.prev_enc_y = None

    def update_vio(self, t, x, y):
        self.vio_positions.append((t, x, y))
        cutoff = t - self.window
        while self.vio_positions and self.vio_positions[0][0] < cutoff:
            self.vio_positions.pop(0)

    def update_encoder(self, t, enc_x, enc_y):
        if self.prev_enc_x is not None:
            dx = enc_x - self.prev_enc_x
            dy = enc_y - self.prev_enc_y
            self.enc_cumulative += math.sqrt(dx * dx + dy * dy)
        self.prev_enc_x = enc_x
        self.prev_enc_y = enc_y
        self.enc_distances.append((t, self.enc_cumulative))
        cutoff = t - self.window
        while self.enc_distances and self.enc_distances[0][0] < cutoff:
            self.enc_distances.pop(0)

    def get_scale(self):
        if len(self.vio_positions) < 10 or len(self.enc_distances) < 10:
            return 1.0
        vio_dist = 0.0
        for i in range(1, len(self.vio_positions)):
            dx = self.vio_positions[i][1] - self.vio_positions[i - 1][1]
            dy = self.vio_positions[i][2] - self.vio_positions[i - 1][2]
            vio_dist += math.sqrt(dx * dx + dy * dy)
        enc_dist = self.enc_distances[-1][1] - self.enc_distances[0][1]
        if vio_dist < self.min_vio_dist:
            return self.current_scale
        raw = enc_dist / vio_dist
        self.current_scale = self.current_scale * (1 - self.alpha) + raw * self.alpha
        self.current_scale = max(self.scale_min, min(self.scale_max, self.current_scale))
        return self.current_scale


def umeyama_ate(traj_est, traj_gt):
    """Umeyama-aligned ATE. traj_est/traj_gt: (N,2) XY arrays."""
    mu_e = traj_est.mean(axis=0)
    mu_g = traj_gt.mean(axis=0)
    e = traj_est - mu_e
    g = traj_gt - mu_g
    H = e.T @ g
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    var_e = (e ** 2).sum() / len(e)
    scale = S.sum() / (var_e * len(e))
    aligned = scale * (traj_est @ R.T) + (mu_g - scale * mu_e @ R.T)
    err = np.linalg.norm(aligned - traj_gt, axis=1)
    return err, scale


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    vio = load_vio(VIO_FILE)
    gt = load_gt(GT_FILE)
    print(f"VIO: {len(vio)} frames, {vio[0,0]:.1f}..{vio[-1,0]:.1f}s")
    print(f"GT:  {len(gt)} frames, {gt[0,0]:.1f}..{gt[-1,0]:.1f}s")

    # Clip VIO to GT timespan (so encoder_dist stays valid)
    mask = (vio[:, 0] >= gt[0, 0]) & (vio[:, 0] <= gt[-1, 0])
    vio = vio[mask]
    print(f"VIO clipped to GT span: {len(vio)} frames")

    # Match VIO frames to GT by timestamp (nearest neighbour)
    vio_ts = vio[:, 0]
    gt_ts = gt[:, 0]
    idx = np.searchsorted(gt_ts, vio_ts)
    idx = np.clip(idx, 0, len(gt) - 1)
    gt_matched = gt[idx]  # (N, 4)

    # Transform VIO to nav frame using first GT pose as alignment
    gt0 = gt_matched[0]
    nav_x, nav_y, _ = vio_camera_to_nav(vio, gt0[1], gt0[2], gt0[3])

    # Apply scale correction using GT as encoder proxy (0.9999 ratio, exp 42)
    # Strategy: scale each frame-to-frame VIO delta, accumulate corrected path
    # Rationale: scaling absolute displacement from origin amplifies noise when
    # scale changes; scaling deltas only affects instantaneous motion
    corrector = ScaleCorrector(window_seconds=60.0, alpha=0.05,
                               min_vio_dist=2.0, scale_min=0.3, scale_max=3.0)
    corr_x = np.zeros(len(vio))
    corr_y = np.zeros(len(vio))
    scales = np.zeros(len(vio))
    corr_x[0] = nav_x[0]
    corr_y[0] = nav_y[0]
    scales[0] = 1.0
    for i in range(len(vio)):
        t = vio_ts[i]
        corrector.update_vio(t, nav_x[i], nav_y[i])
        corrector.update_encoder(t, gt_matched[i, 1], gt_matched[i, 2])
        s = corrector.get_scale()
        scales[i] = s
        if i > 0:
            dx = nav_x[i] - nav_x[i - 1]
            dy = nav_y[i] - nav_y[i - 1]
            corr_x[i] = corr_x[i - 1] + dx * s
            corr_y[i] = corr_y[i - 1] + dy * s

    # Metrics: direct (aligned to GT origin, not Umeyama)
    gt_xy = gt_matched[:, 1:3]
    nav_xy = np.column_stack([nav_x, nav_y])
    corr_xy = np.column_stack([corr_x, corr_y])

    raw_err = np.linalg.norm(nav_xy - gt_xy, axis=1)
    corr_err = np.linalg.norm(corr_xy - gt_xy, axis=1)

    # Umeyama (ORB-SLAM3 convention)
    raw_ate, raw_scale = umeyama_ate(nav_xy, gt_xy)
    corr_ate, corr_scale = umeyama_ate(corr_xy, gt_xy)

    print()
    print("=== DIRECT ERR (spawn-aligned, measures drift from origin) ===")
    print(f"  RAW:       mean={raw_err.mean():.2f}m  max={raw_err.max():.2f}m  median={np.median(raw_err):.2f}m")
    print(f"  CORRECTED: mean={corr_err.mean():.2f}m  max={corr_err.max():.2f}m  median={np.median(corr_err):.2f}m")
    print()
    print("=== UMEYAMA ATE (optimal alignment, ignores origin drift) ===")
    print(f"  RAW:       RMSE={np.sqrt((raw_ate**2).mean()):.2f}m  max={raw_ate.max():.2f}m  scale={raw_scale:.3f}")
    print(f"  CORRECTED: RMSE={np.sqrt((corr_ate**2).mean()):.2f}m  max={corr_ate.max():.2f}m  scale={corr_scale:.3f}")
    print()
    print(f"=== SCALE evolution ===")
    for pct in (10, 25, 50, 75, 100):
        k = min(int(len(scales) * pct / 100) - 1, len(scales) - 1)
        print(f"  {pct:3d}% (t={vio_ts[k]:.0f}s): scale={scales[k]:.3f}")

    # Save CSV
    out_csv = os.path.join(OUT_DIR, "scale_correction_result.csv")
    with open(out_csv, "w") as f:
        f.write("t,gt_x,gt_y,vio_x,vio_y,corr_x,corr_y,scale,raw_err,corr_err\n")
        for i in range(len(vio)):
            f.write(f"{vio_ts[i]:.3f},{gt_matched[i,1]:.3f},{gt_matched[i,2]:.3f},"
                    f"{nav_x[i]:.3f},{nav_y[i]:.3f},{corr_x[i]:.3f},{corr_y[i]:.3f},"
                    f"{scales[i]:.4f},{raw_err[i]:.3f},{corr_err[i]:.3f}\n")
    print(f"\nSaved {out_csv}")

    # Plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        ax = axes[0, 0]
        ax.plot(gt_matched[:, 1], gt_matched[:, 2], 'k-', lw=2, label='GT', alpha=0.7)
        ax.plot(nav_x, nav_y, 'r-', lw=1, label=f'VIO raw (Ume RMSE {np.sqrt((raw_ate**2).mean()):.2f}m)', alpha=0.7)
        ax.plot(corr_x, corr_y, 'b-', lw=1, label=f'VIO corrected (Ume RMSE {np.sqrt((corr_ate**2).mean()):.2f}m)', alpha=0.7)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Trajectory (spawn-aligned)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        ax = axes[0, 1]
        ax.plot(vio_ts - vio_ts[0], scales, 'b-', lw=1)
        ax.axhline(1.0, color='k', ls='--', alpha=0.5, label='ideal')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('scale factor')
        ax.set_title('Scale correction over time')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.plot(vio_ts - vio_ts[0], raw_err, 'r-', lw=1, label=f'raw (mean {raw_err.mean():.2f})')
        ax.plot(vio_ts - vio_ts[0], corr_err, 'b-', lw=1, label=f'corrected (mean {corr_err.mean():.2f})')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('err vs GT [m]')
        ax.set_title('Position error (direct, spawn-aligned)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.plot(vio_ts - vio_ts[0], raw_ate, 'r-', lw=1, label=f'raw (RMSE {np.sqrt((raw_ate**2).mean()):.2f})')
        ax.plot(vio_ts - vio_ts[0], corr_ate, 'b-', lw=1, label=f'corrected (RMSE {np.sqrt((corr_ate**2).mean()):.2f})')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('ATE [m]')
        ax.set_title('Umeyama-aligned ATE')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        out_png = os.path.join(OUT_DIR, "scale_correction_result.png")
        plt.savefig(out_png, dpi=100)
        print(f"Saved {out_png}")
    except ImportError:
        print("matplotlib not available, skipping plot")


if __name__ == '__main__':
    main()
