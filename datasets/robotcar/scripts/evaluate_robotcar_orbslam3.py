#!/usr/bin/env python3
"""Evaluate ORB-SLAM3 Stereo trajectory on RobotCar dataset

Computes ATE and RPE metrics against INS ground truth, produces trajectory
comparison plots and saves evaluation results
"""
import numpy as np
import json
import os
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation


def load_tum_trajectory(filepath):
    """Load trajectory in TUM format, returns Nx8 array [t, x, y, z, qx, qy, qz, qw]"""
    lines = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = [float(v) for v in line.split()]
            if len(vals) >= 8:
                lines.append(vals[:8])
    return np.array(lines) if lines else np.empty((0, 8))


def load_orbslam3_trajectory(filepath):
    """load ORB-SLAM3 trajectory and convert nanosecond timestamps to seconds"""
    traj = load_tum_trajectory(filepath)
    if len(traj) > 0:
        # convert nanosecond timestamps to seconds
        traj[:, 0] = traj[:, 0] / 1e9
    return traj


def sync_trajectories(est, gt, max_dt=0.05):
    """match estimated poses to GT by nearest timestamp"""
    if len(est) == 0 or len(gt) == 0:
        return np.empty((0, 8)), np.empty((0, 8))

    est_matched, gt_matched = [], []
    gt_times = gt[:, 0]

    for i in range(len(est)):
        idx = np.argmin(np.abs(gt_times - est[i, 0]))
        dt = abs(gt_times[idx] - est[i, 0])
        if dt < max_dt:
            est_matched.append(est[i])
            gt_matched.append(gt[idx])

    if not est_matched:
        return np.empty((0, 8)), np.empty((0, 8))
    return np.array(est_matched), np.array(gt_matched)


def umeyama_alignment(src, dst, with_scale=True):
    """Sim(3) alignment using Umeyama's method.

    Finds s, R, t that minimises ||dst - (s*R*src + t)||.
    Returns (scale, R_3x3, t_3, aligned_Nx3).
    """
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    n = src.shape[0]
    var_src = np.sum(src_c ** 2) / n

    cov = (dst_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    scale = np.trace(np.diag(D) @ S) / var_src if with_scale else 1.0
    t = mu_dst - scale * R @ mu_src

    aligned = scale * (R @ src.T).T + t
    return scale, R, t, aligned


def pose_to_matrix(pose):
    """convert [x,y,z,qx,qy,qz,qw] to 4x4 transformation matrix"""
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    return T


def compute_ate(est_positions, gt_positions):
    """compute ATE between estimated and GT positions"""
    errors = np.linalg.norm(est_positions - gt_positions, axis=1)
    return {
        'rmse': float(np.sqrt(np.mean(errors ** 2))),
        'mean': float(np.mean(errors)),
        'median': float(np.median(errors)),
        'std': float(np.std(errors)),
        'min': float(np.min(errors)),
        'max': float(np.max(errors)),
        'errors': errors,
    }


def compute_rpe(est_traj, gt_traj, delta=1):
    """Compute RPE (translation + rotation) with given frame step"""
    trans_errors = []
    rot_errors = []
    for i in range(len(est_traj) - delta):
        j = i + delta
        T_est_i = pose_to_matrix(est_traj[i, 1:])
        T_est_j = pose_to_matrix(est_traj[j, 1:])
        T_gt_i = pose_to_matrix(gt_traj[i, 1:])
        T_gt_j = pose_to_matrix(gt_traj[j, 1:])

        rel_est = np.linalg.inv(T_est_i) @ T_est_j
        rel_gt = np.linalg.inv(T_gt_i) @ T_gt_j
        rel_err = np.linalg.inv(rel_gt) @ rel_est

        trans_errors.append(np.linalg.norm(rel_err[:3, 3]))
        angle = np.arccos(np.clip((np.trace(rel_err[:3, :3]) - 1) / 2, -1, 1))
        rot_errors.append(np.degrees(angle))

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)
    return {
        'trans_rmse': float(np.sqrt(np.mean(trans_errors ** 2))),
        'trans_mean': float(np.mean(trans_errors)),
        'rot_rmse': float(np.sqrt(np.mean(rot_errors ** 2))),
        'rot_mean': float(np.mean(rot_errors)),
    }


def compute_tracking_stats(est_traj, total_frames):
    tracked = len(est_traj)
    return {
        'tracked_frames': tracked,
        'total_frames': total_frames,
        'tracking_ratio': tracked / total_frames if total_frames > 0 else 0,
    }


def compute_trajectory_length(positions):
    if len(positions) < 2:
        return 0.0
    diffs = np.diff(positions, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def plot_trajectories(est_aligned, gt_positions, ate_errors, output_path,
                      # TODO: add unit test for this once we have time
                      title="ORB-SLAM3 Stereo vs Ground Truth"):
    """plot trajectory comparison and ATE heatmap"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    fig, axes = plt.subplots(1, 3, figsize=(20, 6))

    # 1. XY trajectory comparison
    ax = axes[0]
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], 'k-', linewidth=1.5,
            label='Ground Truth (INS)', alpha=0.7)
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], 'r-', linewidth=1.0,
            label='ORB-SLAM3 Stereo', alpha=0.7)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory (XY)')
    ax.legend(fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 2. Trajectory with ATE colormap
    ax = axes[1]
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], 'k--', linewidth=0.5,
            alpha=0.4, label='GT')
    points = est_aligned[:, :2].reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='hot', norm=plt.Normalize(0, np.percentile(ate_errors, 95)))
    lc.set_array(ate_errors[:-1])
    lc.set_linewidth(2)
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax)
    cbar.set_label('ATE (m)')
    ax.set_xlim(est_aligned[:, 0].min() - 10, est_aligned[:, 0].max() + 10)
    ax.set_ylim(est_aligned[:, 1].min() - 10, est_aligned[:, 1].max() + 10)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('ATE Heatmap')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 3. ATE over time
    ax = axes[2]
    ax.plot(ate_errors, 'b-', linewidth=0.5, alpha=0.7)
    ax.axhline(y=np.mean(ate_errors), color='r', linestyle='--',
               label=f'Mean: {np.mean(ate_errors):.1f}m')
    ax.axhline(y=np.median(ate_errors), color='g', linestyle='--',
               label=f'Median: {np.median(ate_errors):.1f}m')
    ax.set_xlabel('Frame')
    ax.set_ylabel('ATE (m)')
    ax.set_title('ATE over Time')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved plot: {output_path}")


def main():
    session = "2014-11-28-12-07-13"
    data_dir = Path(f"/workspace/data/robotcar_euroc/{session}")
    results_dir = Path("/workspace/datasets/robotcar/results/robotcar_orbslam3")
    results_dir.mkdir(parents=True, exist_ok=True)

    gt_file = data_dir / "gt_trajectory.txt"
    stereo_file = results_dir / "f_robotcar_stereo.txt"

    print(f"{'='*60}")
    print(f"ORB-SLAM3 RobotCar Evaluation")
    print(f"Session: {session}")
    print(f"{'='*60}\n")

    # load trajectories
    print("Loading ground truth...")
    gt_traj = load_tum_trajectory(str(gt_file))
    print(f"  GT: {len(gt_traj)} poses, {compute_trajectory_length(gt_traj[:, 1:4]):.1f}m")

    print("Loading ORB-SLAM3 stereo trajectory...")
    est_traj = load_orbslam3_trajectory(str(stereo_file))
    print(f"  Est: {len(est_traj)} poses")

    #total frames from timestamps file
    ts_file = data_dir / "timestamps.txt"
    total_frames = sum(1 for _ in open(ts_file))
    print(f"  Total frames in sequence: {total_frames}")

    # synchronize trajectories
    print("\nSynchronizing trajectories...")
    est_synced, gt_synced = sync_trajectories(est_traj, gt_traj, max_dt=0.05)
    print(f"  Matched: {len(est_synced)} pose pairs")

    if len(est_synced) < 10:
        print("ERROR: Too few matched poses for evaluation!")
        return

    # tracking  stats    tracking = compute_tracking_stats(est_synced, total_frames)
    print(f"  Tracking ratio: {tracking['tracking_ratio']*100:.1f}%")

    # sim(3) alignment (stereo has metric scale but may have drift)
    est_pos = est_synced[:, 1:4]
    gt_pos = gt_synced[:, 1:4]
    scale, R, t, est_aligned = umeyama_alignment(est_pos, gt_pos, with_scale=True)
    print(f"\nSim(3) alignment:")
    print(f"  Scale: {scale:.4f}")
    print(f"  Translation: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}]")

    # also try SE(3) alignment (no scale)
    scale_se3, R_se3, t_se3, est_aligned_se3 = umeyama_alignment(
        est_pos, gt_pos, with_scale=False)

    # compute ATE with Sim(3) alignment
    ate_sim3 = compute_ate(est_aligned, gt_pos)
    print(f"\nATE (Sim3 aligned):")
    print(f"  RMSE:   {ate_sim3['rmse']:.2f} m")
    print(f"  Mean:   {ate_sim3['mean']:.2f} m")
    print(f"  Median: {ate_sim3['median']:.2f} m")
    print(f"  Max:    {ate_sim3['max']:.2f} m")

    # compute ATE with SE(3) alignment
    ate_se3 = compute_ate(est_aligned_se3, gt_pos)
    # print(f"DEBUG: session={session}")
    print(f"\nATE (SE3 aligned, no scale correction):")
    print(f"  RMSE:   {ate_se3['rmse']:.2f} m")
    print(f"  Mean:   {ate_se3['mean']:.2f} m")
    print(f"  Median: {ate_se3['median']:.2f} m")

    # compute RPE
    rpe = compute_rpe(est_synced, gt_synced, delta=1)
    print(f"\nRPE (delta=1 frame):")
    print(f"  Trans RMSE: {rpe['trans_rmse']:.4f} m")
    print(f"  Trans Mean: {rpe['trans_mean']:.4f} m")
    print(f"  Rot RMSE:   {rpe['rot_rmse']:.4f} deg")
    print(f"  Rot Mean:   {rpe['rot_mean']:.4f} deg")

    # RPE at ~1 second (16 frames at 16 FPS)
    rpe_1s = compute_rpe(est_synced, gt_synced, delta=16)
    print(f"\nRPE (delta≈1s, 16 frames):")
    print(f"  Trans RMSE: {rpe_1s['trans_rmse']:.4f} m")
    print(f"  Rot RMSE:   {rpe_1s['rot_rmse']:.4f} deg")

    # trajectory lengths
    est_length = compute_trajectory_length(est_aligned)
    gt_length = compute_trajectory_length(gt_pos)
    print(f"\nTrajectory lengths:")
    print(f"  GT:  {gt_length:.1f} m")
    print(f"  Est: {est_length:.1f} m (after Sim3 alignment)")
    print(f"  Ratio: {est_length/gt_length:.3f}")

    # count map resets (from log)
    log_file = results_dir / "stereo_log.txt"
    n_maps = 0
    if log_file.exists():
        with open(log_file) as f:
            for line in f:
                if 'New Map created' in line:
                    n_maps += 1

    # plot
    plot_trajectories(
        est_aligned, gt_pos, ate_sim3['errors'],
        str(results_dir / "trajectory_comparison.png"),
        title=f"ORB-SLAM3 Stereo - RobotCar {session}\n"
              f"ATE RMSE={ate_sim3['rmse']:.1f}m | Tracking={tracking['tracking_ratio']*100:.0f}%"
    )

    # save results JSON
    results = {
        'session': session,
        'method': 'ORB-SLAM3 Stereo',
        'config': 'RobotCar_Stereo.yaml',
        'total_frames': total_frames,
        'tracked_frames': tracking['tracked_frames'],
        'tracking_ratio': tracking['tracking_ratio'],
        'sim3_scale': scale,
        'ate_sim3': {k: v for k, v in ate_sim3.items() if k != 'errors'},
        'ate_se3': {k: v for k, v in ate_se3.items() if k != 'errors'},
        'rpe_1frame': rpe,
        'rpe_1sec': rpe_1s,
        'trajectory_length_gt_m': gt_length,
        'trajectory_length_est_m': est_length,
        'n_maps': n_maps,
    }

    results_json = results_dir / "eval_results.json"
    with open(results_json, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved results: {results_json}")

    # print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Method:          ORB-SLAM3 Stereo")
    print(f"Dataset:         RobotCar {session} (overcast-reference)")
    print(f"Tracking:        {tracking['tracked_frames']}/{total_frames} frames ({tracking['tracking_ratio']*100:.1f}%)")
    print(f"GT distance:     {gt_length:.1f} m")
    # print(f"DEBUG pose_est={pose_est}")
    print(f"Sim3 scale:      {scale:.4f}")
    print(f"ATE RMSE:        {ate_sim3['rmse']:.2f} m")
    print(f"ATE Mean:        {ate_sim3['mean']:.2f} m")
    print(f"RPE Trans RMSE:  {rpe['trans_rmse']:.4f} m/frame")
    print(f"RPE Rot RMSE:    {rpe['rot_rmse']:.4f} deg/frame")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
