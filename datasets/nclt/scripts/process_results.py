#!/usr/bin/env python3
"""Generate metrics and plots from custom ICP results"""
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.ground_truth_loader import GroundTruthLoader

RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_baseline'
PLOTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_loop_closure' / 'plots'
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

print("="*80)
print("CUSTOM ICP BASELINE RESULTS (Subsampled Data)")
print("="*80)
print("NOTE: Using every 10th scan (500 scans) from preliminary evaluation")
print("="*80)

# load trajectoriesprint("\nLoading trajectories...")
custom_traj = np.loadtxt(RESULTS_DIR / 'custom_icp_trajectory.txt')
gt_traj = np.loadtxt(RESULTS_DIR / 'gt_trajectory.txt')

# reorder quaternion: TUM (qx,qy,qz,qw) -> internal (qw,qx,qy,qz)
custom_traj_converted = np.column_stack([
    custom_traj[:, 0],  # timestamp
    custom_traj[:, 1:4],  # xyz
    custom_traj[:, 7],  # qw
    custom_traj[:, 4:7],  # qx qy qz
])

gt_traj_converted = np.column_stack([
    gt_traj[:, 0],
    gt_traj[:, 1:4],
    gt_traj[:, 7],
    gt_traj[:, 4:7],
])

print(f"Custom ICP: {len(custom_traj_converted)} poses")
print(f"Ground Truth: {len(gt_traj_converted)} poses")


def compute_ate(traj_est, traj_gt):
    pos_est = traj_est[:, 1:4]
    pos_gt = traj_gt[:, 1:4]

    errors = np.linalg.norm(pos_est - pos_gt, axis=1)

    return {
        'mean': errors.mean(),
        'rmse': np.sqrt((errors**2).mean()),
        'std': errors.std(),
        'median': np.median(errors),
        'min': errors.min(),
        'max': errors.max(),
        'errors': errors
    }


def compute_rpe(traj_est, traj_gt, delta=1):
    n = len(traj_est)
    trans_errors = []
    rot_errors = []

    for i in range(n - delta):
        # ground truth relative motion
        pos_gt_i = traj_gt[i, 1:4]
        pos_gt_j = traj_gt[i + delta, 1:4]
        quat_gt_i = traj_gt[i, [4, 5, 6, 7]]  # qw, qx, qy, qz
        quat_gt_j = traj_gt[i + delta, [4, 5, 6, 7]]

        R_gt_i = Rotation.from_quat([quat_gt_i[1], quat_gt_i[2], quat_gt_i[3], quat_gt_i[0]]).as_matrix()
        R_gt_j = Rotation.from_quat([quat_gt_j[1], quat_gt_j[2], quat_gt_j[3], quat_gt_j[0]]).as_matrix()

        T_gt_i = np.eye(4)
        T_gt_i[:3, :3] = R_gt_i
        T_gt_i[:3, 3] = pos_gt_i

        T_gt_j = np.eye(4)
        T_gt_j[:3, :3] = R_gt_j
        T_gt_j[:3, 3] = pos_gt_j

        T_gt_rel = np.linalg.inv(T_gt_i) @ T_gt_j

        # estimated relative motion
        pos_est_i = traj_est[i, 1:4]
        pos_est_j = traj_est[i + delta, 1:4]
        quat_est_i = traj_est[i, [4, 5, 6, 7]]
        quat_est_j = traj_est[i + delta, [4, 5, 6, 7]]

        R_est_i = Rotation.from_quat([quat_est_i[1], quat_est_i[2], quat_est_i[3], quat_est_i[0]]).as_matrix()
        R_est_j = Rotation.from_quat([quat_est_j[1], quat_est_j[2], quat_est_j[3], quat_est_j[0]]).as_matrix()

        T_est_i = np.eye(4)
        T_est_i[:3, :3] = R_est_i
        T_est_i[:3, 3] = pos_est_i

        T_est_j = np.eye(4)
        T_est_j[:3, :3] = R_est_j
        T_est_j[:3, 3] = pos_est_j

        T_est_rel = np.linalg.inv(T_est_i) @ T_est_j

        # relative error
        T_error = np.linalg.inv(T_gt_rel) @ T_est_rel

        # translation error
        trans_error = np.linalg.norm(T_error[:3, 3])
        trans_errors.append(trans_error)

        # rotation error (in degrees)
        R_error = T_error[:3, :3]
        rot_error = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))
        rot_error_deg = np.degrees(rot_error)
        rot_errors.append(rot_error_deg)

    trans_errors = np.array(trans_errors)
    rot_errors = np.array(rot_errors)

    return {
        'trans_rmse': np.sqrt((trans_errors**2).mean()),
        'trans_mean': trans_errors.mean(),
        'trans_median': np.median(trans_errors),
        'rot_rmse': np.sqrt((rot_errors**2).mean()),
        'rot_mean': rot_errors.mean(),
        'rot_median': np.median(rot_errors),
        'trans_errors': trans_errors,
        'rot_errors': rot_errors
    }


# compute metricsprint("\nComputing metrics...")
ate = compute_ate(custom_traj_converted, gt_traj_converted)
rpe = compute_rpe(custom_traj_converted, gt_traj_converted, delta=1)

custom_traj_length = np.linalg.norm(np.diff(custom_traj_converted[:, 1:4], axis=0), axis=1).sum()
gt_traj_length = np.linalg.norm(np.diff(gt_traj_converted[:, 1:4], axis=0), axis=1).sum()

print("\n" + "="*80)
print("EVALUATION METRICS - CUSTOM ICP+IMU")
print("="*80)

print(f"\n{'Metric':<25} {'Value':<15}")
print("-" * 40)
print(f"{'ATE RMSE (m)':<25} {ate['rmse']:<15.3f}")
print(f"{'ATE Mean (m)':<25} {ate['mean']:<15.3f}")
print(f"{'ATE Median (m)':<25} {ate['median']:<15.3f}")
print(f"{'ATE Std (m)':<25} {ate['std']:<15.3f}")
print(f"{'ATE Min (m)':<25} {ate['min']:<15.3f}")
print(f"{'ATE Max (m)':<25} {ate['max']:<15.3f}")
print()
print(f"{'RPE Trans RMSE (m)':<25} {rpe['trans_rmse']:<15.4f}")
print(f"{'RPE Trans Mean (m)':<25} {rpe['trans_mean']:<15.4f}")
print(f"{'RPE Trans Median (m)':<25} {rpe['trans_median']:<15.4f}")
print()
print(f"{'RPE Rot RMSE (deg)':<25} {rpe['rot_rmse']:<15.4f}")
print(f"{'RPE Rot Mean (deg)':<25} {rpe['rot_mean']:<15.4f}")
print(f"{'RPE Rot Median (deg)':<25} {rpe['rot_median']:<15.4f}")
print()
print(f"{'Traj Length Est (m)':<25} {custom_traj_length:<15.1f}")
print(f"{'Traj Length GT (m)':<25} {gt_traj_length:<15.1f}")
print(f"{'Traj Length Error (%)':<25} {100*(custom_traj_length-gt_traj_length)/gt_traj_length:<15.1f}")
print()
print(f"{'Num Poses':<25} {len(custom_traj_converted):<15}")
print(f"{'Processing Time (s)':<25} {'~15':<15}  # (estimated for 500 scans)")
print(f"{'Processing Rate (sc/s)':<25} {'~33':<15}  # (from previous run)")
print(f"{'Data Subsampling':<25} {'Every 10th':<15}")
print("-" * 40)

print("\n" + "="*80)
print("GENERATING PLOTS")
print("="*80)

# 1. Trajectory comparison
plt.figure(figsize=(14, 12))
plt.plot(gt_traj_converted[:, 1], gt_traj_converted[:, 2], 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
plt.plot(custom_traj_converted[:, 1], custom_traj_converted[:, 2], 'b-', linewidth=1.5, label='Custom ICP+IMU', alpha=0.8)

plt.xlabel('X (m)', fontsize=12)
plt.ylabel('Y (m)', fontsize=12)
plt.title('Trajectory Comparison - Custom ICP+IMU (Every 2nd Scan)', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()

traj_plot = PLOTS_DIR / 'trajectory_comparison.png'
plt.savefig(traj_plot, dpi=150)
print(f"✓ Saved: {traj_plot}")
plt.close()

# 2. ATE over time
plt.figure(figsize=(14, 6))
time_axis = custom_traj_converted[:, 0] - custom_traj_converted[0, 0]
plt.plot(time_axis, ate['errors'], 'b-', linewidth=1.5, label='Custom ICP+IMU', alpha=0.8)

plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('ATE (m)', fontsize=12)
plt.title('Absolute Trajectory Error Over Time', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()

ate_plot = PLOTS_DIR / 'ate_over_time.png'
plt.savefig(ate_plot, dpi=150)
print(f"✓ Saved: {ate_plot}")
plt.close()

# 3. Error distribution
plt.figure(figsize=(12, 5))
plt.hist(ate['errors'], bins=50, color='blue', alpha=0.7, edgecolor='black')
plt.axvline(ate['mean'], color='red', linestyle='--', linewidth=2, label=f"Mean: {ate['mean']:.2f}m")
plt.axvline(ate['median'], color='green', linestyle='--', linewidth=2, label=f"Median: {ate['median']:.2f}m")
plt.xlabel('Error (m)', fontsize=12)
plt.ylabel('Frequency', fontsize=12)
plt.title('Custom ICP+IMU Error Distribution', fontsize=13, fontweight='bold')
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.tight_layout()

error_dist_plot = PLOTS_DIR / 'error_distribution.png'
plt.savefig(error_dist_plot, dpi=150)
print(f"✓ Saved: {error_dist_plot}")
plt.close()

# 4. RPE over time
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

# translation error. RPE has one fewer sample than ATE (pairwise over frames)
time_rpe = time_axis[:-1]
ax1.plot(time_rpe, rpe['trans_errors'], 'b-', linewidth=1, alpha=0.7)
ax1.axhline(rpe['trans_mean'], color='red', linestyle='--', linewidth=2, label=f"Mean: {rpe['trans_mean']:.4f}m")
ax1.set_ylabel('Translation Error (m)', fontsize=11)
ax1.set_title('Relative Pose Error - Translation', fontsize=13, fontweight='bold')
ax1.legend(fontsize=10)
ax1.grid(True, alpha=0.3)

ax2.plot(time_rpe, rpe['rot_errors'], 'r-', linewidth=1, alpha=0.7)
ax2.axhline(rpe['rot_mean'], color='blue', linestyle='--', linewidth=2, label=f"Mean: {rpe['rot_mean']:.4f}°")
ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('Rotation Error (deg)', fontsize=11)
ax2.set_title('Relative Pose Error - Rotation', fontsize=13, fontweight='bold')
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
rpe_plot = PLOTS_DIR / 'rpe_over_time.png'
plt.savefig(rpe_plot, dpi=150)
print(f"✓ Saved: {rpe_plot}")
plt.close()

print("\n" + "="*80)
print("COMPLETE!")
print("="*80)
print(f"\nResults directory: {RESULTS_DIR}")
print(f"Plots directory: {PLOTS_DIR}")
