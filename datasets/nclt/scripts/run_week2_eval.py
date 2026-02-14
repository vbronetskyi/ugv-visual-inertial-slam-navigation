#!/usr/bin/env python3
"""KISS-ICP vs Custom ICP evaluation on NCLT data"""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.ground_truth_loader import GroundTruthLoader

SESSION = '2012-04-29'
# LC_SC_THRESH = 0.4  # was 0.5, gave too many false LCs
SUBSAMPLE = 10  # every Nth scan; 10 -> ~2800 scans from 28k, enough for smoke test
MAX_SCANS = 500  # cap for quick testing
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_baseline'
PLOTS_DIR = RESULTS_DIR / 'plots'

PLOTS_DIR.mkdir(parents=True, exist_ok=True)

print('  SLAM Evaluation: KISS-ICP vs Custom ICP')
print(f"Session: {SESSION}")
print(f"Subsample rate: every {SUBSAMPLE}th scan")
print(f"Max scans: {MAX_SCANS}")
print(f"Results: {RESULTS_DIR}")


class CustomICP:
    def __init__(self):
        self.poses = []
        self.timestamps = []
        self.current_pose = np.eye(4)

    def register_scan(self, points: np.ndarray, timestamp: int):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=0.5)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )

        if len(self.poses) == 0:
            self.poses.append(self.current_pose.copy())
            self.timestamps.append(timestamp)
            self.prev_pcd = pcd
            return self.current_pose

        threshold = 2.0
        trans_init = np.eye(4)

        reg_result = o3d.pipelines.registration.registration_icp(
            pcd, self.prev_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )

        self.current_pose = self.current_pose @ reg_result.transformation
        self.poses.append(self.current_pose.copy())
        self.timestamps.append(timestamp)
        self.prev_pcd = pcd

        return self.current_pose

    def get_trajectory(self):
        """Nx7 trajectory: [timestamp, x, y, z, qw, qx, qy, qz]"""
        from scipy.spatial.transform import Rotation

        traj = []
        for timestamp, pose in zip(self.timestamps, self.poses):
            pos = pose[:3, 3]
            rot = Rotation.from_matrix(pose[:3, :3])
            quat = rot.as_quat()  # [qx, qy, qz, qw]

            traj.append([
                timestamp / 1e6,  # Convert to seconds
                pos[0], pos[1], pos[2],
                quat[3], quat[0], quat[1], quat[2]  # [qw, qx, qy, qz]
            ])

        return np.array(traj)


def load_velodyne_scans(session, subsample=1, max_scans=None):
    print(f"\nLoading Velodyne scans for {session}...")
    loader = VelodyneLoader()

    files = loader.get_velodyne_sync_files(session)
    files = files[::subsample]  # Subsample

    if max_scans:
        files = files[:max_scans]

    print(f"Selected {len(files)} scans (from {len(loader.get_velodyne_sync_files(session))} total)")

    scans = []
    timestamps = []

    for i, filepath in enumerate(tqdm(files, desc="Loading scans")):
        points = loader.load_velodyne_sync(filepath)
        timestamp = loader.get_timestamp_from_filename(filepath)

        scans.append(points)
        timestamps.append(timestamp)

    return scans, timestamps, files


def run_custom_icp(scans, timestamps):
    print("\n" + "="*60)
    print("Running Custom ICP Odometry")

    icp = CustomICP()

    for i, (points, timestamp) in enumerate(tqdm(
        zip(scans, timestamps),
        total=len(scans),
        desc="Custom ICP"
    )):
        pose = icp.register_scan(points, timestamp)

        if (i + 1) % 100 == 0:
            pos = pose[:3, 3]
            print(f"  Scan {i+1}/{len(scans)}: position = [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

    trajectory = icp.get_trajectory()
    print(f"\nCustom ICP complete: {len(trajectory)} poses")
    print(f"Trajectory length: {np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum():.1f} m")

    return trajectory


def load_ground_truth(session, timestamps):
    print("\n" + "="*60)
    print("Loading Ground Truth")

    loader = GroundTruthLoader()
    gt_full = loader.load_ground_truth(session)

    print(f"Full GT: {len(gt_full)} poses")

    # synchronize with scan timestamps
    gt_synced = []
    for timestamp in timestamps:
        pose = loader.get_pose_at_timestamp(gt_full, timestamp, tolerance_us=200000)
        if pose:
            gt_synced.append([
                timestamp / 1e6,  # seconds
                pose['position'][0], pose['position'][1], pose['position'][2],
                pose['quaternion'][0], pose['quaternion'][1],
                pose['quaternion'][2], pose['quaternion'][3]
            ])

    gt_synced = np.array(gt_synced)
    print(f"Synchronized GT: {len(gt_synced)} poses")

    return gt_synced


def try_kiss_icp(scans, timestamps):
    print("\n" + "="*60)
    print("Attempting KISS-ICP")

    try:
        # try to import kiss_icp
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.config import KISSConfig

        print("KISS-ICP found, running...")

        # create config with proper defaults
        config = KISSConfig()
        config.data.deskew = False  # Disable deskewing for simplicity
        config.mapping.voxel_size = 1.0  # Set voxel size explicitly

        odometry = KissICP(config=config)

        # debug: check what attributes the odometry object has
        print(f"\nDEBUG: KissICP attributes: {[attr for attr in dir(odometry) if not attr.startswith('_')]}")

        poses = []
        for i, (points, timestamp) in enumerate(tqdm(
            zip(scans, timestamps),
            total=len(scans),
            desc="KISS-ICP"
        )):
            # register frame (returns downsampled points, map points - we don't need these)
            _ = odometry.register_frame(points[:, :3], timestamps=np.array([timestamp / 1e6]))

            # get the current pose - try different possible attributes
            if hasattr(odometry, 'current_pose'):
                pose = odometry.current_pose
            elif hasattr(odometry, 'last_pose'):
                pose = odometry.last_pose
            elif hasattr(odometry, 'pose'):
                pose = odometry.pose
            else:
                print(f'ERROR: Cannot find pose attribute in KissICP object')
                break

            poses.append(pose)

            if (i + 1) % 100 == 0:
                pos = pose[:3, 3]
                print(f"  Scan {i+1}/{len(scans)}: position = [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        # convert to trajectory format
        from scipy.spatial.transform import Rotation
        trajectory = []
        for timestamp, pose in zip(timestamps, poses):
            pos = pose[:3, 3]
            rot = Rotation.from_matrix(pose[:3, :3])
            quat = rot.as_quat()  # [qx, qy, qz, qw]

            trajectory.append([
                timestamp / 1e6,
                pos[0], pos[1], pos[2],
                quat[3], quat[0], quat[1], quat[2]
            ])

        trajectory = np.array(trajectory)
        print(f"\nKISS-ICP complete: {len(trajectory)} poses")
        print(f"Trajectory length: {np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum():.1f} m")

        return trajectory

    except ImportError as e:
        print(f"KISS-ICP not available: {e}")
        return None
    except Exception as e:
        print(f"KISS-ICP failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def compute_ate(traj_est, traj_gt):
    """Compute Absolute Trajectory Error"""
    pos_est = traj_est[:, 1:4]
    pos_gt = traj_gt[:, 1:4]

    errors = np.linalg.norm(pos_est - pos_gt, axis=1)

    ate_mean = errors.mean()
    ate_rmse = np.sqrt((errors**2).mean())
    ate_std = errors.std()

    return {
        'mean': ate_mean,
        'rmse': ate_rmse,
        'std': ate_std,
        'errors': errors
    }


def plot_results(gt_traj, custom_traj, kiss_traj=None):
    # TODO: check with prof if ATE RMSE or ATE mean is what the thesis needs
    print("\n" + "="*60)
    print("Generating Plots")

    # 1. Trajectory comparison (bird's eye view)
    plt.figure(figsize=(12, 10))
    plt.plot(gt_traj[:, 1], gt_traj[:, 2], 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
    plt.plot(custom_traj[:, 1], custom_traj[:, 2], 'b-', linewidth=1.5, label='Custom ICP', alpha=0.8)
    if kiss_traj is not None:
        plt.plot(kiss_traj[:, 1], kiss_traj[:, 2], 'r-', linewidth=1.5, label='KISS-ICP', alpha=0.8)

    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.title(f'Trajectory Comparison - {SESSION}', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()

    traj_plot = PLOTS_DIR / 'trajectory_comparison.png'
    plt.savefig(traj_plot, dpi=150)
    print(f"Saved: {traj_plot}")
    plt.close()

    # 2. ATE over time
    custom_ate = compute_ate(custom_traj, gt_traj)

    plt.figure(figsize=(12, 6))
    time_axis = custom_traj[:, 0] - custom_traj[0, 0]  # Relative time
    plt.plot(time_axis, custom_ate['errors'], 'b-', linewidth=1.5, label='Custom ICP', alpha=0.8)

    if kiss_traj is not None:
        kiss_ate = compute_ate(kiss_traj, gt_traj)
        plt.plot(time_axis, kiss_ate['errors'], 'r-', linewidth=1.5, label='KISS-ICP', alpha=0.8)

    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('ATE (m)', fontsize=12)
    plt.title('Absolute Trajectory Error Over Time', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    ate_plot = PLOTS_DIR / 'ate_over_time.png'
    plt.savefig(ate_plot, dpi=150)
    print(f"Saved: {ate_plot}")
    plt.close()

    # 3. Error distribution histogram
    fig, axes = plt.subplots(1, 2 if kiss_traj is not None else 1, figsize=(14, 5))

    if kiss_traj is None:
        axes = [axes]

    # custom ICP
    axes[0].hist(custom_ate['errors'], bins=50, color='blue', alpha=0.7, edgecolor='black')
    axes[0].axvline(custom_ate['mean'], color='red', linestyle='--', linewidth=2, label=f"Mean: {custom_ate['mean']:.2f}m")
    axes[0].set_xlabel('Error (m)', fontsize=12)
    axes[0].set_ylabel('Frequency', fontsize=12)
    axes[0].set_title('Custom ICP Error Distribution', fontsize=13, fontweight='bold')
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    # KISS-ICP
    if kiss_traj is not None:
        axes[1].hist(kiss_ate['errors'], bins=50, color='red', alpha=0.7, edgecolor='black')
        axes[1].axvline(kiss_ate['mean'], color='blue', linestyle='--', linewidth=2, label=f"Mean: {kiss_ate['mean']:.2f}m")
        axes[1].set_xlabel('Error (m)', fontsize=12)
        axes[1].set_ylabel('Frequency', fontsize=12)
        axes[1].set_title('KISS-ICP Error Distribution', fontsize=13, fontweight='bold')
        axes[1].legend(fontsize=10)
        axes[1].grid(True, alpha=0.3)

    plt.tight_layout()

    error_dist_plot = PLOTS_DIR / 'error_distribution.png'
    plt.savefig(error_dist_plot, dpi=150)
    print(f"Saved: {error_dist_plot}")
    plt.close()

    return custom_ate, kiss_ate if kiss_traj is not None else None


def save_trajectories(gt_traj, custom_traj, kiss_traj=None):
    print("\n" + "="*60)
    print("Saving Trajectories (TUM format)")

    def save_tum(traj, filename):
        # TUM format: timestamp x y z qx qy qz qw
        tum_traj = np.column_stack([
            traj[:, 0],  # timestamp
            traj[:, 1:4],  # xyz
            traj[:, 5:8],  # qx qy qz
            traj[:, 4]  # qw
        ])
        np.savetxt(filename, tum_traj, fmt='%.6f')
        print(f"Saved: {filename}")

    save_tum(gt_traj, RESULTS_DIR / 'gt_trajectory.txt')
    save_tum(custom_traj, RESULTS_DIR / 'custom_icp_trajectory.txt')
    if kiss_traj is not None:
        save_tum(kiss_traj, RESULTS_DIR / 'kiss_icp_trajectory.txt')


def main():

    # 1. Load data
    scans, timestamps, files = load_velodyne_scans(SESSION, SUBSAMPLE, MAX_SCANS)
    gt_traj = load_ground_truth(SESSION, timestamps)

    # 2. Run Custom ICP
    custom_traj = run_custom_icp(scans, timestamps)

    # 3. Try KISS-ICP
    kiss_traj = try_kiss_icp(scans, timestamps)

    # 4. Save trajectories
    save_trajectories(gt_traj, custom_traj, kiss_traj)

    # 5. Compute metrics
    print("\n" + "="*60)
    print("EVALUATION RESULTS")

    custom_ate = compute_ate(custom_traj, gt_traj)
    print(f"\nCustom ICP:")
    print(f"  ATE Mean: {custom_ate['mean']:.3f} m")
    print(f"  ATE RMSE: {custom_ate['rmse']:.3f} m")
    print(f"  ATE Std:  {custom_ate['std']:.3f} m")

    if kiss_traj is not None:
        kiss_ate = compute_ate(kiss_traj, gt_traj)
        print(f"\nKISS-ICP:")
        print(f"  ATE Mean: {kiss_ate['mean']:.3f} m")
        print(f"  ATE RMSE: {kiss_ate['rmse']:.3f} m")
        print(f"  ATE Std:  {kiss_ate['std']:.3f} m")

    # 6. Generate plots
    custom_ate, kiss_ate = plot_results(gt_traj, custom_traj, kiss_traj)

    # 7. List generated files
    print("\n" + "="*60)
    print("GENERATED FILES")

    for item in sorted(PLOTS_DIR.glob('*.png')):
        print(f"  {item}")

    for item in sorted(RESULTS_DIR.glob('*.txt')):
        print(f"  {item}")

    print("\n" + "="*60)
    print("EVALUATION COMPLETE!")


if __name__ == '__main__':
    main()
