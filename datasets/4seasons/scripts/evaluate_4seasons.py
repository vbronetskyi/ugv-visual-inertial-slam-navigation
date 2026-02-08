#!/usr/bin/env python3
"""Evaluate ORB-SLAM3 trajectory on 4Seasons dataset against GNSSPoses ground truth

Usage:
  python3 evaluate_4seasons.py \
      --traj kf_4seasons_office_loop_1.txt \
      --gt /workspace/data/4seasons/office_loop_1_euroc/gt_tum.txt \
      --output /workspace/datasets/robotcar/results/4seasons/office_loop_1/
"""

import argparse
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_tum_trajectory(path):
    """load TUM trajectory, returns (timestamps, positions, quaternions)"""
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            data.append([float(p) for p in parts[:8]])
    data = np.array(data)
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def load_orbslam_trajectory(path):
    """load ORB-SLAM3 output, auto-detect ns vs s timestamps"""
    ts, pos, quat = load_tum_trajectory(path)
    # auto-detect nanosecond timestamps (>1e15 means nanoseconds)
    if len(ts) > 0 and ts[0] > 1e15:
        ts = ts / 1e9
    return ts, pos, quat


def associate_trajectories(ts_est, ts_gt, max_diff=0.05):
    """associate est and GT trajectories by timestamp, returns matched index arrays"""
    idx_est = []
    idx_gt = []
    gt_idx = 0

    for i, te in enumerate(ts_est):
        while gt_idx < len(ts_gt) - 1 and ts_gt[gt_idx + 1] <= te:
            gt_idx += 1
        # check neighbors
        best_j = gt_idx
        best_diff = abs(te - ts_gt[gt_idx])
        if gt_idx + 1 < len(ts_gt):
            d = abs(te - ts_gt[gt_idx + 1])
            if d < best_diff:
                best_j = gt_idx + 1
                best_diff = d
        if best_diff <= max_diff:
            idx_est.append(i)
            idx_gt.append(best_j)

    return np.array(idx_est), np.array(idx_gt)


def umeyama_alignment(src, dst, with_scale=True):
    """Umeyama Sim3/SE3 alignment, returns (R, t, scale)"""
    assert src.shape == dst.shape
    n, d = src.shape

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    var_src = np.sum(src_c ** 2) / n

    H = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(H)

    D = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[d - 1, d - 1] = -1

    R = U @ D @ Vt

    if with_scale:
        s = np.trace(np.diag(S) @ D) / var_src
    else:
        s = 1.0

    t = mu_dst - s * R @ mu_src
    return R, t, s


def compute_ate(pos_est, pos_gt, R, t, s):
    """Compute Absolute Trajectory Error after alignment"""
    pos_aligned = s * (R @ pos_est.T).T + t
    errors = np.linalg.norm(pos_aligned - pos_gt, axis=1)
    return errors, pos_aligned


def compute_rpe(pos_est, pos_gt, delta=1):
    """Compute Relative Pose Error"""
    errors = []
    for i in range(len(pos_est) - delta):
        dp_est = pos_est[i + delta] - pos_est[i]
        dp_gt = pos_gt[i + delta] - pos_gt[i]
        errors.append(np.linalg.norm(dp_gt - dp_est))
    return np.array(errors)


def main():
    parser = argparse.ArgumentParser(description="Evaluate ORB-SLAM3 on 4Seasons")
    parser.add_argument("--traj", required=True, help="ORB-SLAM3 trajectory (TUM format)")
    parser.add_argument("--gt", required=True, help="Ground truth (TUM format)")
    parser.add_argument("--output", "-o", required=True, help="Output directory")
    parser.add_argument("--max-diff", type=float, default=0.05,
                        help="Max timestamp difference for association (seconds)")
    parser.add_argument("--total-frames", type=int, default=0,
                        help="Total frames in sequence (for tracking rate calculation)")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # load trajectories
    print("Loading trajectories...")
    ts_est, pos_est, quat_est = load_orbslam_trajectory(args.traj)
    ts_gt, pos_gt, quat_gt = load_tum_trajectory(args.gt)

    print(f"  Estimated: {len(ts_est)} poses, t=[{ts_est[0]:.3f}, {ts_est[-1]:.3f}]")
    print(f"  GT:        {len(ts_gt)} poses, t=[{ts_gt[0]:.3f}, {ts_gt[-1]:.3f}]")

    # associate
    print("Associating trajectories...")
    idx_e, idx_g = associate_trajectories(ts_est, ts_gt, max_diff=args.max_diff)
    print(f"  Matched: {len(idx_e)} pairs")

    if len(idx_e) < 3:
        print("ERROR: Not enough matches for alignment!")
        sys.exit(1)

    pos_e = pos_est[idx_e]
    pos_g = pos_gt[idx_g]
    ts_matched = ts_est[idx_e]

    # sim3 alignment
    print("Aligning (Sim3)...")
    R, t, s = umeyama_alignment(pos_e, pos_g, with_scale=True)
    ate_errors, pos_aligned = compute_ate(pos_e, pos_g, R, t, s)

    # SE3 alignment (no scale)
    R_se3, t_se3, _ = umeyama_alignment(pos_e, pos_g, with_scale=False)
    ate_se3, pos_aligned_se3 = compute_ate(pos_e, pos_g, R_se3, t_se3, 1.0)

    # rpe
    rpe_errors = compute_rpe(pos_aligned, pos_g)

    # tracking rate
    tracking_rate = len(ts_est) / args.total_frames * 100 if args.total_frames > 0 else -1

    # results
    results = {
        "dataset": "4Seasons",
        "method": "ORB-SLAM3 Stereo-Inertial",
        "num_estimated": len(ts_est),
        "num_gt": len(ts_gt),
        "num_matched": len(idx_e),
        "tracking_rate_pct": round(tracking_rate, 1) if tracking_rate > 0 else "N/A",
        "sim3_scale": round(s, 6),
        "ate_sim3": {
            "rmse": round(float(np.sqrt(np.mean(ate_errors**2))), 4),
            "mean": round(float(np.mean(ate_errors)), 4),
            "median": round(float(np.median(ate_errors)), 4),
            "std": round(float(np.std(ate_errors)), 4),
            "max": round(float(np.max(ate_errors)), 4),
        },
        "ate_se3": {
            "rmse": round(float(np.sqrt(np.mean(ate_se3**2))), 4),
            "mean": round(float(np.mean(ate_se3)), 4),
            "median": round(float(np.median(ate_se3)), 4),
        },
        "rpe": {
            "rmse": round(float(np.sqrt(np.mean(rpe_errors**2))), 4),
            "mean": round(float(np.mean(rpe_errors)), 4),
        },
    }

    # print results
    print(f"\n{'='*50}")
    print(f"Results (Sim3 alignment):")
    print(f"  Scale:      {s:.6f}")
    print(f"  ATE RMSE:   {results['ate_sim3']['rmse']:.4f} m")
    print(f"  ATE Mean:   {results['ate_sim3']['mean']:.4f} m")
    print(f"  ATE Median: {results['ate_sim3']['median']:.4f} m")
    print(f"  ATE Max:    {results['ate_sim3']['max']:.4f} m")
    print(f"  RPE RMSE:   {results['rpe']['rmse']:.4f} m")
    if tracking_rate > 0:
        print(f"  Tracking:   {tracking_rate:.1f}%")
    print(f"{'='*50}")

    # save JSON
    json_path = os.path.join(args.output, "eval_results.json")
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {json_path}")

    # plot
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # 1. Trajectory comparison (XY)
    ax = axes[0]
    ax.plot(pos_g[:, 0], pos_g[:, 1], 'b-', linewidth=1, alpha=0.7, label='GT (GNSS)')
    ax.plot(pos_aligned[:, 0], pos_aligned[:, 1], 'r-', linewidth=1, alpha=0.7,
            label=f'ORB-SLAM3 SI (Sim3, s={s:.3f})')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Comparison (XY)')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 2. ATE heatmap
    ax = axes[1]
    sc = ax.scatter(pos_aligned[:, 0], pos_aligned[:, 1], c=ate_errors,
                    cmap='hot', s=2, vmin=0)
    plt.colorbar(sc, ax=ax, label='ATE (m)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'ATE Heatmap (RMSE={results["ate_sim3"]["rmse"]:.2f}m)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 3. ATE over time
    ax = axes[2]
    t_rel = ts_matched - ts_matched[0]
    ax.plot(t_rel, ate_errors, 'r-', linewidth=0.5, alpha=0.7)
    ax.axhline(results['ate_sim3']['rmse'], color='b', linestyle='--', alpha=0.5,
               label=f'RMSE={results["ate_sim3"]["rmse"]:.2f}m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('ATE (m)')
    ax.set_title('ATE Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(args.output, "trajectory_comparison.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"Plot saved to {plot_path}")

    # save summary
    summary_path = os.path.join(args.output, "summary.txt")
    with open(summary_path, 'w') as f:
        f.write(f"4Seasons ORB-SLAM3 Stereo-Inertial Evaluation\n")
        f.write(f"{'='*50}\n")
        f.write(f"Trajectory: {args.traj}\n")
        f.write(f"Ground Truth: {args.gt}\n")
        f.write(f"Estimated poses: {len(ts_est)}\n")
        f.write(f"GT poses: {len(ts_gt)}\n")
        f.write(f"Matched pairs: {len(idx_e)}\n")
        if tracking_rate > 0:
            f.write(f"Tracking rate: {tracking_rate:.1f}%\n")
        f.write(f"\nSim3 alignment (scale={s:.6f}):\n")
        f.write(f"  ATE RMSE:   {results['ate_sim3']['rmse']:.4f} m\n")
        f.write(f"  ATE Mean:   {results['ate_sim3']['mean']:.4f} m\n")
        f.write(f"  ATE Median: {results['ate_sim3']['median']:.4f} m\n")
        f.write(f"  ATE Max:    {results['ate_sim3']['max']:.4f} m\n")
        f.write(f"\nSE3 alignment (no scale):\n")
        f.write(f"  ATE RMSE:   {results['ate_se3']['rmse']:.4f} m\n")
        f.write(f"\nRPE:\n")
        f.write(f"  RMSE: {results['rpe']['rmse']:.4f} m\n")


if __name__ == "__main__":
    main()
