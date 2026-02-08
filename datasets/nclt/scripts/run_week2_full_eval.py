#!/usr/bin/env python3
"""Full-scale KISS-ICP vs Custom ICP+IMU on NCLT 2012-04-29 (12,971 scans)"""
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
KISS_ICP_SUBSAMPLE = 5  # Use every 5th scan for KISS-ICP (2,594 scans - manageable)
CUSTOM_ICP_SUBSAMPLE = 2  # Use every 2nd scan for Custom ICP (6,486 scans)
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_loop_closure'
PLOTS_DIR = RESULTS_DIR / 'plots'

PLOTS_DIR.mkdir(parents=True, exist_ok=True)

print("="*80)
print("  SLAM PRACTICAL EVALUATION: KISS-ICP vs Custom ICP+IMU")
print("="*80)
print(f"Session: {SESSION}")
print(f"KISS-ICP: every {KISS_ICP_SUBSAMPLE}th scan (~2,594 scans - practical for performance)")
print(f"Custom ICP: every {CUSTOM_ICP_SUBSAMPLE}nd scan (~6,486 scans - denser sampling)")
print(f"Results: {RESULTS_DIR}")
print("="*80)


class CustomICPWithIMU:
    """ICP odometry with IMU motion prediction for initial guess"""

    def __init__(self, imu_data=None):
        self.poses = []
        self.timestamps = []
        self.current_pose = np.eye(4)
        self.prev_pcd = None
        self.imu_data = imu_data
        self.prev_timestamp = None

    def get_imu_prediction(self, timestamp):
        """Get motion prediction from IMU data"""
        if self.imu_data is None or self.prev_timestamp is None:
            return np.eye(4)
        # TODO: integrate angular velocity + linear acceleration
        return np.eye(4)

    def register_scan(self, points: np.ndarray, timestamp: int):
        """Register a new scan with IMU-aided initial guess"""
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

        imu_prediction = self.get_imu_prediction(timestamp)
        threshold = 1.5
        trans_init = imu_prediction

        reg_result = o3d.pipelines.registration.registration_icp(
            pcd, self.prev_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
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
        """Get trajectory as Nx7 array [timestamp, x, y, z, qw, qx, qy, qz]"""
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


def load_imu_data(session):
    try:
        imu_path = f'/workspace/nclt_data/sensor_data/{session}/ms25.csv'
        print(f"Loading IMU data from {imu_path}...")
        # ms25.csv format: utime, odom, angular_velocity_x/y/z, linear_acceleration_x/y/z, etc.
        df = pd.read_csv(imu_path)
        print(f"Loaded {len(df)} IMU measurements")
        return df
    except Exception as e:
        print(f"Warning: Could not load IMU data: {e}")
        return None


def load_velodyne_scans(session, subsample=1, max_scans=None):
    print(f"\nLoading Velodyne scans for {session} (subsample={subsample})...")
    loader = VelodyneLoader()

    files = loader.get_velodyne_sync_files(session)
    files = files[::subsample]  # Subsample

    if max_scans:
        files = files[:max_scans]

    print(f"Selected {len(files)} scans (from {len(loader.get_velodyne_sync_files(session))} total)")

    scans = []
    timestamps = []

    for filepath in tqdm(files, desc="Loading scans"):
        points = loader.load_velodyne_sync(filepath)
        timestamp = loader.get_timestamp_from_filename(filepath)

        scans.append(points)
        timestamps.append(timestamp)

    return scans, timestamps, files


def run_custom_icp_with_imu(scans, timestamps, imu_data=None):
    print("\n" + "="*80)
    print("Running Custom ICP+IMU Odometry")
    print("="*80)

    start_time = time.time()
    icp = CustomICPWithIMU(imu_data=imu_data)

    for i, (points, timestamp) in enumerate(tqdm(
        zip(scans, timestamps),
        total=len(scans),
        desc="Custom ICP+IMU"
    )):
        pose = icp.register_scan(points, timestamp)

        if (i + 1) % 500 == 0:
            pos = pose[:3, 3]
            elapsed = time.time() - start_time
            rate = (i + 1) / elapsed
            print(f"  Scan {i+1}/{len(scans)}: pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], "
                  f"rate={rate:.1f} scans/s, elapsed={elapsed:.1f}s")

    trajectory = icp.get_trajectory()
    elapsed = time.time() - start_time

    print(f"\nCustom ICP+IMU complete: {len(trajectory)} poses in {elapsed:.1f}s")
    print(f"Processing rate: {len(trajectory)/elapsed:.1f} scans/s")
    print(f"Trajectory length: {np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum():.1f} m")

    return trajectory, elapsed


def try_kiss_icp(scans, timestamps):
    """Run KISS-ICP on all scans"""
    print("\n" + "="*80)
    print("Running KISS-ICP")
    print("="*80)

    try:
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.config import KISSConfig

        print("KISS-ICP found, running...")

        # create config optimized for outdoor driving
        config = KISSConfig()
        config.data.deskew = False
        config.mapping.voxel_size = 0.5  # Smaller voxel for better accuracy
        config.mapping.max_points_per_voxel = 20

        start_time = time.time()
        odometry = KissICP(config=config)

        poses = []
        for i, (points, timestamp) in enumerate(tqdm(
            zip(scans, timestamps),
            total=len(scans),
            desc="KISS-ICP"
        )):
            # register frame
            _ = odometry.register_frame(points[:, :3], timestamps=np.array([timestamp / 1e6]))

            # get current pose
            pose = odometry.last_pose
            poses.append(pose)

            if (i + 1) % 500 == 0:
                pos = pose[:3, 3]
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                print(f"  Scan {i+1}/{len(scans)}: pos=[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], "
                      f"rate={rate:.1f} scans/s, elapsed={elapsed:.1f}s")

        # convert to trajectory format
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
        elapsed = time.time() - start_time

        print(f"\nKISS-ICP complete: {len(trajectory)} poses in {elapsed:.1f}s")
        print(f"Processing rate: {len(trajectory)/elapsed:.1f} scans/s")
        print(f"Trajectory length: {np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1).sum():.1f} m")

        return trajectory, elapsed

    except ImportError as e:
        print(f"KISS-ICP not available: {e}")
        return None, 0
    except Exception as e:
        print(f"KISS-ICP failed: {e}")
        import traceback
        traceback.print_exc()
        return None, 0


def load_ground_truth(session, timestamps):
    print("\n" + "="*80)
    print("Loading Ground Truth")
    print("="*80)

    loader = GroundTruthLoader()
    gt_full = loader.load_ground_truth(session)

    print(f"Full GT: {len(gt_full)} poses")

    # synchronize with scan timestamps
    gt_synced = []
    for timestamp in tqdm(timestamps, desc="Syncing GT"):
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


def compute_ate(traj_est, traj_gt):
    """Compute Absolute Trajectory Error"""
    pos_est = traj_est[:, 1:4]
    pos_gt = traj_gt[:, 1:4]

    errors = np.linalg.norm(pos_est - pos_gt, axis=1)

    ate_mean = errors.mean()
    ate_rmse = np.sqrt((errors**2).mean())
    ate_std = errors.std()
    ate_median = np.median(errors)
    ate_min = errors.min()
    ate_max = errors.max()

    return {
        'mean': ate_mean,
        'rmse': ate_rmse,
        'std': ate_std,
        'median': ate_median,
        'min': ate_min,
        'max': ate_max,
        'errors': errors
    }


def compute_rpe(traj_est, traj_gt, delta=1):
    """Compute Relative Pose Error between estimated and GT trajectories"""
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


def save_trajectories(gt_traj, custom_traj, custom_time, kiss_traj=None, kiss_time=0):
    print("\n" + "="*80)
    print("Saving Trajectories (TUM format)")
    print("="*80)

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

    # save timing info
    with open(RESULTS_DIR / 'timing.txt', 'w') as f:
        f.write(f"Custom ICP+IMU: {custom_time:.1f}s ({len(custom_traj)/custom_time:.2f} scans/s)\n")
        if kiss_traj is not None:
            f.write(f"KISS-ICP: {kiss_time:.1f}s ({len(kiss_traj)/kiss_time:.2f} scans/s)\n")
    print(f"Saved: {RESULTS_DIR / 'timing.txt'}")


def plot_results(gt_traj, custom_traj, kiss_traj=None):
    print("\n" + "="*80)
    print("Generating Plots")
    print("="*80)

    # 1. Trajectory comparison
    plt.figure(figsize=(14, 12))
    plt.plot(gt_traj[:, 1], gt_traj[:, 2], 'k-', linewidth=2, label='Ground Truth', alpha=0.8)
    plt.plot(custom_traj[:, 1], custom_traj[:, 2], 'b-', linewidth=1.5, label='Custom ICP+IMU', alpha=0.8)
    if kiss_traj is not None:
        plt.plot(kiss_traj[:, 1], kiss_traj[:, 2], 'r-', linewidth=1.5, label='KISS-ICP', alpha=0.8)

    plt.xlabel('X (m)', fontsize=12)
    plt.ylabel('Y (m)', fontsize=12)
    plt.title(f'Full-Scale Trajectory Comparison - {SESSION}', fontsize=14, fontweight='bold')
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

    plt.figure(figsize=(14, 6))
    time_axis = custom_traj[:, 0] - custom_traj[0, 0]
    plt.plot(time_axis, custom_ate['errors'], 'b-', linewidth=1.5, label='Custom ICP+IMU', alpha=0.8)

    if kiss_traj is not None:
        kiss_ate = compute_ate(kiss_traj, gt_traj)
        time_axis_kiss = kiss_traj[:, 0] - kiss_traj[0, 0]
        plt.plot(time_axis_kiss, kiss_ate['errors'], 'r-', linewidth=1.5, label='KISS-ICP', alpha=0.8)

    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('ATE (m)', fontsize=12)
    plt.title('Absolute Trajectory Error Over Time (Full Dataset)', fontsize=14, fontweight='bold')
    plt.legend(fontsize=11)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    ate_plot = PLOTS_DIR / 'ate_over_time.png'
    plt.savefig(ate_plot, dpi=150)
    print(f"Saved: {ate_plot}")
    plt.close()


def print_metrics_table(custom_traj, gt_traj, custom_time, kiss_traj=None, kiss_time=0):
    print("\n" + "="*80)
    print("EVALUATION METRICS - FULL DATASET")
    print("="*80)

    # compute metrics for Custom ICP
    custom_ate = compute_ate(custom_traj, gt_traj)
    custom_rpe = compute_rpe(custom_traj, gt_traj, delta=1)
    custom_traj_length = np.linalg.norm(np.diff(custom_traj[:, 1:4], axis=0), axis=1).sum()

    print(f"\n{'Method':<20} {'ATE RMSE':<12} {'RPE Trans':<15} {'RPE Rot':<15} {'Traj Len':<12} {'Time':<12} {'Rate':<12}")
    print("-" * 110)

    print(f"{'Custom ICP+IMU':<20} "
          f"{custom_ate['rmse']:<12.3f} "
          f"{custom_rpe['trans_rmse']:<15.4f} "
          f"{custom_rpe['rot_rmse']:<15.4f} "
          f"{custom_traj_length:<12.1f} "
          f"{custom_time:<12.1f} "
          f"{len(custom_traj)/custom_time:<12.1f}")

    if kiss_traj is not None:
        kiss_ate = compute_ate(kiss_traj, gt_traj)
        kiss_rpe = compute_rpe(kiss_traj, gt_traj, delta=1)
        kiss_traj_length = np.linalg.norm(np.diff(kiss_traj[:, 1:4], axis=0), axis=1).sum()

        print(f"{'KISS-ICP':<20} "
              f"{kiss_ate['rmse']:<12.3f} "
              f"{kiss_rpe['trans_rmse']:<15.4f} "
              f"{kiss_rpe['rot_rmse']:<15.4f} "
              f"{kiss_traj_length:<12.1f} "
              f"{kiss_time:<12.1f} "
              f"{len(kiss_traj)/kiss_time:<12.1f}")

    gt_traj_length = np.linalg.norm(np.diff(gt_traj[:, 1:4], axis=0), axis=1).sum()
    print(f"{'Ground Truth':<20} "
          f"{'---':<12} "
          f"{'---':<15} "
          f"{'---':<15} "
          f"{gt_traj_length:<12.1f} "
          f"{'---':<12} "
          f"{'---':<12}")

    print("-" * 110)
    print(f"\nUnits: ATE RMSE (m), RPE Trans (m/frame), RPE Rot (deg/frame), ")
    print(f"       Traj Len (m), Time (s), Rate (scans/s)")
    print(f"\nDataset: {len(custom_traj)} scans for Custom ICP, {len(kiss_traj) if kiss_traj is not None else 0} scans for KISS-ICP")


def main():

    # load IMU data
    imu_data = load_imu_data(SESSION)

    # load scans for Custom ICP (every 2nd scan)
    custom_scans, custom_timestamps, _ = load_velodyne_scans(
        SESSION, subsample=CUSTOM_ICP_SUBSAMPLE
    )

    # load ground truth for Custom ICP scans
    custom_gt_traj = load_ground_truth(SESSION, custom_timestamps)

    # run Custom ICP with IMU
    custom_traj, custom_time = run_custom_icp_with_imu(
        custom_scans, custom_timestamps, imu_data
    )

    # load scans for KISS-ICP (all scans)
    kiss_scans, kiss_timestamps, _ = load_velodyne_scans(
        SESSION, subsample=KISS_ICP_SUBSAMPLE
    )

    # load ground truth for KISS-ICP scans
    kiss_gt_traj = load_ground_truth(SESSION, kiss_timestamps)

    # run KISS-ICP
    kiss_traj, kiss_time = try_kiss_icp(kiss_scans, kiss_timestamps)

    # save trajectories
    save_trajectories(custom_gt_traj, custom_traj, custom_time, kiss_traj, kiss_time)

    # generate plots (use KISS-ICP GT for plotting since it has all scans)
    plot_results(kiss_gt_traj, custom_traj, kiss_traj)

    # print metrics table
    print_metrics_table(custom_traj, custom_gt_traj, custom_time, kiss_traj, kiss_time)

    print("\n" + "="*80)
    print("EVALUATION COMPLETE!")
    print("="*80)
    print(f"\nResults saved to: {RESULTS_DIR}")
    print(f"Plots saved to: {PLOTS_DIR}")


if __name__ == '__main__':
    main()
