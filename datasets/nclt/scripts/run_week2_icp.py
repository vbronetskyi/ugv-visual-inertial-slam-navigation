#!/usr/bin/env python3
"""
Dense Custom ICP Evaluation - Every 2nd scan with improved settings
Processes 6,486 scans from NCLT session 2012-04-29
"""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
import open3d as o3d
from scipy.spatial.transform import Rotation
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.ground_truth_loader import GroundTruthLoader

SESSION = '2012-04-29'
SUBSAMPLE = 2  # every 2nd scan -> 6,486 scans
# TODO: try subsample=1 once i can run overnight
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_loop_closure'
PLOTS_DIR = RESULTS_DIR / 'plots'

PLOTS_DIR.mkdir(parents=True, exist_ok=True)

print("  DENSE CUSTOM ICP EVALUATION")
print(f"Session: {SESSION}")
print(f"Sampling: every {SUBSAMPLE}nd scan (~6,486 scans)")
print(f"Settings: 0.3m voxel, 100 iterations, 1.5m threshold")
print(f"Results: {RESULTS_DIR}")


class CustomICP:
    """Point-to-plane ICP odometry with 0.3m voxel downsampling"""

    def __init__(self):
        self.poses = []
        self.timestamps = []
        self.current_pose = np.eye(4)
        self.prev_pcd = None
        self.prev_timestamp = None

    def register_scan(self, points: np.ndarray, timestamp: int):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=0.3)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )

        if self.prev_pcd is None:
            self.poses.append(self.current_pose.copy())
            self.timestamps.append(timestamp)
            self.prev_pcd = pcd
            self.prev_timestamp = timestamp
            return self.current_pose

        threshold = 1.5
        trans_init = np.eye(4)  # could use constant velocity model here

        reg_result = o3d.pipelines.registration.registration_icp(
            pcd, self.prev_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
# FAST_THRESH = 15  # default 20 too strict for fisheye
                max_iteration=100,  # More iterations
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )

        self.current_pose = self.current_pose @ reg_result.transformation
        self.poses.append(self.current_pose.copy())
        self.timestamps.append(timestamp)
        self.prev_pcd = pcd
        self.prev_timestamp = timestamp

        return self.current_pose

    def get_trajectory(self):
        """Nx8 TUM trajectory: [timestamp, x, y, z, qx, qy, qz, qw]"""
        traj = []
        for timestamp, pose in zip(self.timestamps, self.poses):
            pos = pose[:3, 3]
            rot = Rotation.from_matrix(pose[:3, :3])
            quat = rot.as_quat()  # [qx, qy, qz, qw]

            traj.append([
                timestamp / 1e6,  # Convert to seconds
                pos[0], pos[1], pos[2],
                quat[0], quat[1], quat[2], quat[3]  # TUM format: qx, qy, qz, qw
            ])

        return np.array(traj)


def load_velodyne_scans(session, subsample=1):
    print(f"\nLoading Velodyne scans for {session} (subsample={subsample})...")
    loader = VelodyneLoader()

    files = loader.get_velodyne_sync_files(session)
    total_scans = len(files)
    print(f"Total scans available: {total_scans}")

    # apply subsampling
    files = files[::subsample]
    print(f"Scans to process: {len(files)}")

    scans = []
    timestamps = []
    for file_path in tqdm(files, desc="Loading scans"):
        points = loader.load_velodyne_sync(file_path)
        timestamp = int(Path(file_path).stem)
        scans.append(points)
        timestamps.append(timestamp)

    return scans, timestamps


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
    # NOTE: not thread-safe but we run single threaded anyway
    n = len(traj_est)
    trans_errors = []
    rot_errors = []

    for i in range(n - delta):
        # ground truth relative motion
        pos_gt_i = traj_gt[i, 1:4]
        pos_gt_j = traj_gt[i + delta, 1:4]
        # TUM format: qx, qy, qz, qw
        quat_gt_i = traj_gt[i, [7, 4, 5, 6]]  # [qw, qx, qy, qz]
        quat_gt_j = traj_gt[i + delta, [7, 4, 5, 6]]

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
        quat_est_i = traj_est[i, [7, 4, 5, 6]]  # [qw, qx, qy, qz]
        quat_est_j = traj_est[i + delta, [7, 4, 5, 6]]

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


# ============================================================================
# MAIN EXECUTION
# ============================================================================

print("\n" + "="*80)
print("LOADING GROUND TRUTH")
gt_loader = GroundTruthLoader()
gt_df = gt_loader.load_ground_truth(SESSION)
# convert to numpy array: utime (in seconds), x, y, z, qx, qy, qz, qw
gt_all = np.column_stack([
    gt_df['utime'].values / 1e6,  # Convert to seconds
    gt_df['x'].values,
    gt_df['y'].values,
    gt_df['z'].values,
    gt_df['qx'].values,
    gt_df['qy'].values,
    gt_df['qz'].values,
    gt_df['qw'].values,
])
print(f"Ground truth poses: {len(gt_all)}")

scans, timestamps = load_velodyne_scans(SESSION, subsample=SUBSAMPLE)
print(f"\nLoaded {len(scans)} scans")

print("\n" + "="*80)
print('RUNNING CUSTOM ICP')

odometry = CustomICP()
start_time = time.time()

for i, (scan, timestamp) in enumerate(tqdm(
    zip(scans, timestamps),
    total=len(scans),
    desc="Custom ICP"
)):
    pose = odometry.register_scan(scan, timestamp)

    if (i + 1) % 500 == 0:
        pos = pose[:3, 3]
        elapsed = time.time() - start_time
        rate = (i + 1) / elapsed
        print(f"\n  Progress: {i+1}/{len(scans)} scans | "
              f"Rate: {rate:.1f} scans/s | "
              f"Position: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")

end_time = time.time()
processing_time = end_time - start_time
processing_rate = len(scans) / processing_time

print(f"\nCustom ICP complete!")
print(f"Processing time: {processing_time:.1f}s")
print(f"Processing rate: {processing_rate:.1f} scans/s")

custom_traj = odometry.get_trajectory()
print(f"Trajectory: {len(custom_traj)} poses")

print("\n" + "="*80)
print("SYNCHRONIZING WITH GROUND TRUTH")

gt_synced = []
for i, timestamp in enumerate(custom_traj[:, 0]):
    # find closest ground truth pose (within 200ms)
    time_diffs = np.abs(gt_all[:, 0] - timestamp)
    min_idx = np.argmin(time_diffs)

    if time_diffs[min_idx] < 0.2:  # 200ms tolerance
        gt_synced.append(gt_all[min_idx])

gt_traj = np.array(gt_synced)
print(f"Synchronized: {len(gt_traj)} ground truth poses")

# trim to same length
min_len = min(len(custom_traj), len(gt_traj))
custom_traj = custom_traj[:min_len]
gt_traj = gt_traj[:min_len]

print(f"Final trajectory length: {min_len} poses")

print("\n" + "="*80)
print('COMPUTING METRICS')

# convert to consistent format for metrics (timestamp, x, y, z, qw, qx, qy, qz)
custom_traj_metrics = np.column_stack([
    custom_traj[:, 0],  # timestamp
    custom_traj[:, 1:4],  # xyz
    custom_traj[:, 7],  # qw
    custom_traj[:, 4:7],  # qx, qy, qz
])

gt_traj_metrics = np.column_stack([
    gt_traj[:, 0],
    gt_traj[:, 1:4],
    gt_traj[:, 7],
    gt_traj[:, 4:7],
])

ate = compute_ate(custom_traj_metrics, gt_traj_metrics)
rpe = compute_rpe(custom_traj, gt_traj, delta=1)

custom_traj_length = np.linalg.norm(np.diff(custom_traj[:, 1:4], axis=0), axis=1).sum()
gt_traj_length = np.linalg.norm(np.diff(gt_traj[:, 1:4], axis=0), axis=1).sum()

print("\n" + "="*80)
print("EVALUATION METRICS - CUSTOM ICP (DENSE)")

print(f"\n{'Metric':<30} {'Value':<15}")
print(f"{'ATE RMSE (m)':<30} {ate['rmse']:<15.3f}")
print(f"{'ATE Mean (m)':<30} {ate['mean']:<15.3f}")
print(f"{'ATE Median (m)':<30} {ate['median']:<15.3f}")
print(f"{'ATE Std (m)':<30} {ate['std']:<15.3f}")
print(f"{'ATE Min (m)':<30} {ate['min']:<15.3f}")
print(f"{'ATE Max (m)':<30} {ate['max']:<15.3f}")
print()
print(f"{'RPE Trans RMSE (m/frame)':<30} {rpe['trans_rmse']:<15.4f}")
print(f"{'RPE Trans Mean (m/frame)':<30} {rpe['trans_mean']:<15.4f}")
print(f"{'RPE Trans Median (m/frame)':<30} {rpe['trans_median']:<15.4f}")
print()
print(f"{'RPE Rot RMSE (deg/frame)':<30} {rpe['rot_rmse']:<15.4f}")
print(f"{'RPE Rot Mean (deg/frame)':<30} {rpe['rot_mean']:<15.4f}")
print(f"{'RPE Rot Median (deg/frame)':<30} {rpe['rot_median']:<15.4f}")
print()
print(f"{'Traj Length Est (m)':<30} {custom_traj_length:<15.1f}")
print(f"{'Traj Length GT (m)':<30} {gt_traj_length:<15.1f}")
print(f"{'Traj Length Error (%)':<30} {100*(custom_traj_length-gt_traj_length)/gt_traj_length:<15.1f}")
print()
print(f"{'Num Poses':<30} {len(custom_traj):<15}")
print(f"{'Processing Time (s)':<30} {processing_time:<15.1f}")
print(f"{'Processing Rate (scans/s)':<30} {processing_rate:<15.1f}")
print(f"{'Data Subsampling':<30} {'Every 2nd':<15}")

print("\n" + "="*80)
print("SAVING TRAJECTORIES")

np.savetxt(RESULTS_DIR / 'custom_icp_dense_trajectory.txt', custom_traj,
           fmt='%.6f', header='timestamp x y z qx qy qz qw')
np.savetxt(RESULTS_DIR / 'gt_dense_trajectory.txt', gt_traj,
           fmt='%.6f', header='timestamp x y z qx qy qz qw')

print(f"✓ Saved: {RESULTS_DIR / 'custom_icp_dense_trajectory.txt'}")
print(f"✓ Saved: {RESULTS_DIR / 'gt_dense_trajectory.txt'}")

print("\n" + "="*80)
print("GENERATING PLOTS")

# 1. Trajectory comparison
plt.figure(figsize=(14, 12))
plt.plot(gt_traj[:, 1], gt_traj[:, 2], 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
plt.plot(custom_traj[:, 1], custom_traj[:, 2], 'b-', linewidth=1.5, label='Custom ICP (Every 2nd Scan)', alpha=0.8)

plt.xlabel('X (m)', fontsize=12)
plt.ylabel('Y (m)', fontsize=12)
plt.title('Trajectory Comparison - Dense Custom ICP (Every 2nd Scan)', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()

traj_plot = PLOTS_DIR / 'trajectory_comparison_dense.png'
plt.savefig(traj_plot, dpi=150)
print(f"✓ Saved: {traj_plot}")
plt.close()

# 2. ATE over time
plt.figure(figsize=(14, 6))
time_axis = custom_traj_metrics[:, 0] - custom_traj_metrics[0, 0]
plt.plot(time_axis, ate['errors'], 'b-', linewidth=1.5, label='Custom ICP (Dense)', alpha=0.8)

plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('ATE (m)', fontsize=12)
plt.title('Absolute Trajectory Error Over Time - Dense Sampling', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()

ate_plot = PLOTS_DIR / 'ate_over_time_dense.png'
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
plt.title('Error Distribution - Dense Custom ICP', fontsize=13, fontweight='bold')
plt.legend(fontsize=10)
plt.grid(True, alpha=0.3)
plt.tight_layout()

error_dist_plot = PLOTS_DIR / 'error_distribution_dense.png'
plt.savefig(error_dist_plot, dpi=150)
print(f"✓ Saved: {error_dist_plot}")
plt.close()

# 4. RPE over time
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

# translation error. RPE has n-1 samples (pairwise delta)
time_rpe = time_axis[:-1]
ax1.plot(time_rpe, rpe['trans_errors'], 'b-', linewidth=1, alpha=0.7)
ax1.axhline(rpe['trans_mean'], color='red', linestyle='--', linewidth=2, label=f"Mean: {rpe['trans_mean']:.4f}m")
ax1.set_ylabel('Translation Error (m)', fontsize=11)
ax1.set_title('Relative Pose Error - Translation (Dense)', fontsize=13, fontweight='bold')
ax1.legend(fontsize=10)
ax1.grid(True, alpha=0.3)

ax2.plot(time_rpe, rpe['rot_errors'], 'r-', linewidth=1, alpha=0.7)
ax2.axhline(rpe['rot_mean'], color='blue', linestyle='--', linewidth=2, label=f"Mean: {rpe['rot_mean']:.4f}°")
ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('Rotation Error (deg)', fontsize=11)
ax2.set_title('Relative Pose Error - Rotation (Dense)', fontsize=13, fontweight='bold')
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
rpe_plot = PLOTS_DIR / 'rpe_over_time_dense.png'
plt.savefig(rpe_plot, dpi=150)
print(f"✓ Saved: {rpe_plot}")
plt.close()

print("\n" + "="*80)
print("EVALUATION COMPLETE!")
print(f"\nResults saved to: {RESULTS_DIR}")
print(f"Plots saved to: {PLOTS_DIR}")
print("\nGenerated files:")
print(f"  - custom_icp_dense_trajectory.txt")
print(f"  - gt_dense_trajectory.txt")
print(f"  - trajectory_comparison_dense.png")
print(f"  - ate_over_time_dense.png")
print(f"  - error_distribution_dense.png")
print(f"  - rpe_over_time_dense.png")
