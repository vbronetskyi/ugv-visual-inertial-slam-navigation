#!/usr/bin/env python3
"""Week 0: visual SLAM baseline on NCLT Ladybug3.

tries ORB-SLAM3 mono first; if not available falls back to a simple OpenCV VO
pipeline (ORB features + essential matrix). compared against GT trajectory.

only spring (2012-04-29) and summer (2012-08-04) have camera images,
winter/autumn are LiDAR-only.
"""
import os
import sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
from scipy.spatial.transform import Rotation
import time
import glob

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.ground_truth_loader import GroundTruthLoader
from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.sensor_loader import SensorLoader
from evaluation.metrics import compute_ate, compute_rpe, sync_trajectories
from slam.imu_fusion import OdometryPredictor, LocalMap, GPSLoopClosureDetector, remove_ground
from slam.loop_closure import PoseGraphOptimizer2D, pose_to_2d, rel2d

import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parent.parent
NCLT_DATA = Path('/workspace/nclt_data')


# lidar pipeline (reusable for all sessions)
def run_lidar_pipeline(session, subsample=2):
    """Run best LiDAR pipeline (Week 3) on a session, returns dict with trajectories and metrics"""
    print(f"\n{'='*80}")
    print(f"  LIDAR PIPELINE: {session}")
    print(f"{'='*80}")

    # --- load data ---
    gt_loader = GroundTruthLoader()
    gt_df = gt_loader.load_ground_truth(session)
    gt_all = np.column_stack([
        gt_df['utime'].values / 1e6, gt_df['x'].values, gt_df['y'].values,
        gt_df['z'].values, gt_df['qx'].values, gt_df['qy'].values,
        gt_df['qz'].values, gt_df['qw'].values])
    print(f"GT: {len(gt_all)} poses")

    sensor = SensorLoader()

    # wheel odometry
    try:
        odom_df = sensor.load_odometry_mu(session, hz=100)
        print(f"Odometry: {len(odom_df)} samples (100Hz)")
        has_odom = True
    except Exception as e:
        print(f"Warning: No 100Hz odometry for {session}, trying 10Hz: {e}")
        try:
            odom_df = sensor.load_odometry_mu(session, hz=10)
            print(f"Odometry: {len(odom_df)} samples (10Hz)")
            has_odom = True
        except Exception as e2:
            print(f"Warning: No odometry available: {e2}")
            has_odom = False

    # GPS
    try:
        gps_df = sensor.load_gps_rtk(session)
        print(f"GPS RTK: {len(gps_df)} samples")
        has_gps = len(gps_df) > 100
    except Exception as e:
        print(f"Warning: No GPS RTK for {session}: {e}")
        has_gps = False

    # scans
    loader = VelodyneLoader()
    all_files = loader.get_velodyne_sync_files(session)
    if len(all_files) == 0:
        print(f"ERROR: No velodyne files for {session}")
        return None
    files = all_files[::subsample]
    print(f"Scans: {len(files)} (every {subsample}nd of {len(all_files)})")

    # ICP params
    ICP_VOXEL = 0.3
    ICP_THRESHOLD = 1.5
    ICP_MAX_ITER = 80
    LOCALMAP_SIZE = 20
    LOCALMAP_VOXEL = 0.5
    GROUND_DIST = 0.25

    # GPS loop closure params
    GPS_LC_MIN_GAP = 200
    GPS_LC_RADIUS = 15.0
    GPS_LC_ICP_FITNESS = 0.25
    GPS_LC_ICP_DIST = 2.0
    GPS_LC_DEDUP = 50

    # pose graph params
    PG_ODOM_W = 1.0
    PG_LC_W = 10.0
    PG_DAMPING = 1e-3

    # init components
    odom_pred = OdometryPredictor(odom_df) if has_odom else None
    local_map = LocalMap(LOCALMAP_SIZE, LOCALMAP_VOXEL)
    gps_lc = GPSLoopClosureDetector(gps_df, GPS_LC_MIN_GAP, GPS_LC_RADIUS, GPS_LC_DEDUP) if has_gps else None

    # --- Step 1: Odometry-aided ICP ---
    print(f"\n  Step 1: ICP Odometry ({len(files)} scans)...")
    poses_4x4 = []
    timestamps_us = []
    pcd_cache = {}

    current_pose = np.eye(4)
    prev_pcd = None
    prev_timestamp = None
    t0 = time.time()

    for idx in tqdm(range(len(files)), desc=f"ICP {session}"):
        fp = files[idx]
        pts = loader.load_velodyne_sync(fp)
        ts = int(Path(fp).stem)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
        pcd = pcd.voxel_down_sample(ICP_VOXEL)
        pcd = remove_ground(pcd, GROUND_DIST)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

        if idx % 10 == 0:
            pcd_cache[idx] = pcd

        if prev_pcd is not None:
            if has_odom:
                T_odom_rel = odom_pred.get_relative_transform(prev_timestamp, ts)
            else:
                T_odom_rel = np.eye(4)

            map_pcd = local_map.get_map_pcd()

            if map_pcd is not None and len(map_pcd.points) > 500:
                T_pred = current_pose @ T_odom_rel
                reg = o3d.pipelines.registration.registration_icp(
                    pcd, map_pcd, ICP_THRESHOLD, T_pred,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=ICP_MAX_ITER,
                        relative_fitness=1e-6, relative_rmse=1e-6))
                current_pose = reg.transformation
            else:
                reg = o3d.pipelines.registration.registration_icp(
                    pcd, prev_pcd, ICP_THRESHOLD, T_odom_rel,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=ICP_MAX_ITER,
                        relative_fitness=1e-6, relative_rmse=1e-6))
                current_pose = current_pose @ reg.transformation

        poses_4x4.append(current_pose.copy())
        timestamps_us.append(ts)
        local_map.add_scan(np.asarray(pcd.points), current_pose)
        prev_pcd = pcd
        prev_timestamp = ts

        if (idx+1) % 1000 == 0:
            el = time.time()-t0
            print(f"    [{idx+1}/{len(files)}] {(idx+1)/el:.1f} sc/s")

    t_odom = time.time()-t0
    print(f"  ICP done: {len(poses_4x4)} poses in {t_odom:.1f}s")

    # --- Step 2: GPS loop closure ---
    verified_lc = []
    t_lc = 0
    if gps_lc is not None:
        print(f"  Step 2: GPS loop closure detection...")
        t0 = time.time()
        gps_candidates = gps_lc.find_candidates(timestamps_us)
        print(f"    GPS candidates: {len(gps_candidates)}")

        for q, c, gps_dist in tqdm(gps_candidates, desc="ICP verify"):
            pcd_q = _get_cached_pcd(pcd_cache, q, files, loader, GROUND_DIST)
            pcd_c = _get_cached_pcd(pcd_cache, c, files, loader, GROUND_DIST)
            if pcd_q is None or pcd_c is None:
                continue

            T_q, T_c = poses_4x4[q], poses_4x4[c]
            T_init = np.linalg.inv(T_c) @ T_q
            if np.linalg.norm(T_init[:3, 3]) > 100:
                T_init = np.eye(4)

            reg = o3d.pipelines.registration.registration_icp(
                pcd_q, pcd_c, GPS_LC_ICP_DIST, T_init,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=50, relative_fitness=1e-6, relative_rmse=1e-6))

            if reg.fitness >= GPS_LC_ICP_FITNESS:
                verified_lc.append((q, c, reg.transformation, reg.fitness,
                                   reg.inlier_rmse, gps_dist))

        t_lc = time.time()-t0
        print(f"    Verified: {len(verified_lc)} / {len(gps_candidates)} in {t_lc:.1f}s")
    else:
        print(f"  Step 2: Skipped (no GPS)")

    # --- Step 3: Pose graph optimization ---
    print(f"  Step 3: Pose graph optimization...")
    poses_2d = np.array([pose_to_2d(p) for p in poses_4x4])
    z_vals = [p[2,3] for p in poses_4x4]

    odom_edges = []
    for i in range(len(poses_2d)-1):
        dx, dy, dt = rel2d(*poses_2d[i], *poses_2d[i+1])
        odom_edges.append((i, i+1, dx, dy, dt))

    lc_edges = []
    for q, c, T, fit, rmse, gd in verified_lc:
        dx, dy = T[0,3], T[1,3]
        dt = np.arctan2(T[1,0], T[0,0])
        lc_edges.append((c, q, dx, dy, dt))

    if len(lc_edges) > 0:
        opt = PoseGraphOptimizer2D(PG_ODOM_W, PG_LC_W, PG_DAMPING)
        opt_2d = opt.optimize(poses_2d, odom_edges, lc_edges)
    else:
        print("    No loop closures. Using odometry-only.")
        opt_2d = poses_2d.copy()

    # --- Build trajectories ---
    odom_traj = []
    for ts, pose in zip(timestamps_us, poses_4x4):
        p = pose[:3,3]; q = Rotation.from_matrix(pose[:3,:3]).as_quat()
        odom_traj.append([ts/1e6, p[0], p[1], p[2], q[0], q[1], q[2], q[3]])
    odom_traj = np.array(odom_traj)

    opt_traj = []
    for i, (p2d, ts) in enumerate(zip(opt_2d, timestamps_us)):
        x, y, th = p2d; z = z_vals[i] if i < len(z_vals) else 0
        opt_traj.append([ts/1e6, x, y, z, 0, 0, np.sin(th/2), np.cos(th/2)])
    opt_traj = np.array(opt_traj)

    # sync with GT
    opt_traj_s, gt_traj_s = sync_trajectories(opt_traj, gt_all)
    odom_traj_s, gt_traj_o = sync_trajectories(odom_traj, gt_all)

    ml = min(len(odom_traj_s), len(opt_traj_s))
    odom_traj_s = odom_traj_s[:ml]
    opt_traj_s = opt_traj_s[:ml]
    gt_traj_s = gt_traj_s[:ml]

    # compute metrics
    ate_odom = compute_ate(odom_traj_s, gt_traj_s)
    ate_opt = compute_ate(opt_traj_s, gt_traj_s)
    rpe_odom = compute_rpe(odom_traj_s, gt_traj_s)
    rpe_opt = compute_rpe(opt_traj_s, gt_traj_s)

    traj_len = np.linalg.norm(np.diff(opt_traj_s[:, 1:4], axis=0), axis=1).sum()
    gt_len = np.linalg.norm(np.diff(gt_traj_s[:, 1:4], axis=0), axis=1).sum()

    total_time = t_odom + t_lc

    result = {
        'session': session,
        'odom_traj': odom_traj_s,
        'opt_traj': opt_traj_s,
        'gt_traj': gt_traj_s,
        'ate_odom': ate_odom,
        'ate_opt': ate_opt,
        'rpe_odom': rpe_odom,
        'rpe_opt': rpe_opt,
        'traj_len': traj_len,
        'gt_len': gt_len,
        'n_loop_closures': len(lc_edges),
        'n_poses': len(opt_traj_s),
        'total_time': total_time,
    }

    print(f"\n  Results for {session}:")
    print(f"    ATE RMSE: {ate_opt['rmse']:.1f}m (odom-only: {ate_odom['rmse']:.1f}m)")
    print(f"    RPE Trans: {rpe_opt['trans_rmse']:.4f} m/frame")
    print(f"    RPE Rot:   {rpe_opt['rot_rmse']:.2f} deg/frame")
    print(f"    Traj len:  {traj_len:.1f}m (GT: {gt_len:.1f}m)")
    print(f"    LC:        {len(lc_edges)} verified")
    print(f"    Time:      {total_time:.1f}s")

    return result


def _get_cached_pcd(cache, idx, files, loader, ground_dist):
    """Get cached or freshly loaded point cloud"""
    if idx in cache:
        return cache[idx]
    nearest = min(cache.keys(), key=lambda k: abs(k-idx)) if cache else None
    if nearest is not None and abs(nearest-idx) <= 5:
        return cache[nearest]
    try:
        pts = loader.load_velodyne_sync(files[idx])
        p = o3d.geometry.PointCloud()
        p.points = o3d.utility.Vector3dVector(pts[:,:3])
        p = p.voxel_down_sample(0.3)
        p = remove_ground(p, ground_dist)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))
        return p
    except Exception:
        return None


# cross-season comparison
def generate_comparison(results, output_dir):
    """Generate cross-season comparison table and plots"""
    output_dir = Path(output_dir)
    plots_dir = output_dir / 'plots'
    plots_dir.mkdir(parents=True, exist_ok=True)

    sessions = list(results.keys())
    season_names = {
        '2012-01-08': 'Winter',
        '2012-04-29': 'Spring',
        '2012-08-04': 'Summer',
        '2012-10-28': 'Autumn',
    }

    # --- Comparison table ---
    table_lines = []
    table_lines.append("=" * 100)
    table_lines.append("CROSS-SEASON COMPARISON: LiDAR SLAM Pipeline (Odom-Aided ICP + Local Map + GPS LC)")
    table_lines.append("=" * 100)
    table_lines.append("")

    hdr = f"{'Session':<14} {'Season':<8} {'ATE RMSE':>10} {'ATE Mean':>10} {'RPE Trans':>10} {'RPE Rot':>10} {'Traj Len':>10} {'GT Len':>10} {'LC':>5} {'Time':>8}"
    table_lines.append(hdr)
    table_lines.append("-" * 100)

    for s in sessions:
        r = results[s]
        line = (f"{s:<14} {season_names.get(s, '?'):<8} "
                f"{r['ate_opt']['rmse']:>10.1f} {r['ate_opt']['mean']:>10.1f} "
                f"{r['rpe_opt']['trans_rmse']:>10.4f} {r['rpe_opt']['rot_rmse']:>10.2f} "
                f"{r['traj_len']:>10.1f} {r['gt_len']:>10.1f} "
                f"{r['n_loop_closures']:>5} {r['total_time']:>7.0f}s")
        table_lines.append(line)

    table_lines.append("-" * 100)

    # summary stats
    ate_vals = [results[s]['ate_opt']['rmse'] for s in sessions]
    table_lines.append(f"\nBest ATE RMSE:  {min(ate_vals):.1f}m ({sessions[np.argmin(ate_vals)]})")
    table_lines.append(f"Worst ATE RMSE: {max(ate_vals):.1f}m ({sessions[np.argmax(ate_vals)]})")
    table_lines.append(f"Mean ATE RMSE:  {np.mean(ate_vals):.1f}m")

    table_text = "\n".join(table_lines)
    print(f"\n{table_text}")

    with open(output_dir / 'comparison_table.txt', 'w') as f:
        f.write(table_text + "\n")

    # --- Plot 1: All trajectories (subplots) ---
    n = len(sessions)
    fig, axes = plt.subplots(2, 2, figsize=(20, 20))
    axes = axes.flatten()

    for i, s in enumerate(sessions):
        ax = axes[i]
        r = results[s]
        ax.plot(r['gt_traj'][:, 1], r['gt_traj'][:, 2], 'k-', lw=2,
                label='Ground Truth', alpha=0.8)
        ax.plot(r['opt_traj'][:, 1], r['opt_traj'][:, 2], 'r-', lw=1.5,
                label=f'Estimated (ATE={r["ate_opt"]["rmse"]:.0f}m)', alpha=0.7)
        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_title(f'{s} ({season_names.get(s, "?")})', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    for i in range(len(sessions), 4):
        axes[i].axis('off')

    plt.suptitle('Cross-Season Trajectory Comparison', fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(plots_dir / 'all_trajectories.png', dpi=150)
    print(f"  Saved: all_trajectories.png")
    plt.close()

    # --- Plot 2: ATE over time (all sessions) ---
    fig, ax = plt.subplots(figsize=(16, 8))
    colors = ['blue', 'green', 'red', 'orange']
    for i, s in enumerate(sessions):
        r = results[s]
        t = r['opt_traj'][:, 0] - r['opt_traj'][0, 0]
        ax.plot(t, r['ate_opt']['errors'], '-', color=colors[i % len(colors)],
                lw=1.5, label=f"{s} ({season_names.get(s, '?')}, RMSE={r['ate_opt']['rmse']:.0f}m)",
                alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('ATE (m)', fontsize=12)
    ax.set_title('ATE Over Time - All Sessions', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(plots_dir / 'ate_comparison.png', dpi=150)
    print(f"  Saved: ate_comparison.png")
    plt.close()

    # --- Plot 3: Bar chart ---
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    labels = [f"{s}\n{season_names.get(s, '?')}" for s in sessions]
    x = np.arange(len(sessions))

    # ATE RMSE
    ate_rmse = [results[s]['ate_opt']['rmse'] for s in sessions]
    bars = axes[0].bar(x, ate_rmse, color=colors[:len(sessions)], alpha=0.8, edgecolor='black')
    axes[0].set_ylabel('ATE RMSE (m)', fontsize=12)
    axes[0].set_title('Absolute Trajectory Error', fontsize=13, fontweight='bold')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, fontsize=9)
    axes[0].grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, ate_rmse):
        axes[0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                    f'{val:.0f}m', ha='center', fontsize=10, fontweight='bold')

    # RPE Trans
    rpe_trans = [results[s]['rpe_opt']['trans_rmse'] for s in sessions]
    bars = axes[1].bar(x, rpe_trans, color=colors[:len(sessions)], alpha=0.8, edgecolor='black')
    axes[1].set_ylabel('RPE Trans RMSE (m/frame)', fontsize=12)
    axes[1].set_title('Relative Pose Error (Translation)', fontsize=13, fontweight='bold')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, fontsize=9)
    axes[1].grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, rpe_trans):
        axes[1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{val:.3f}', ha='center', fontsize=10, fontweight='bold')

    # trajectory length error %
    len_err = [abs(results[s]['traj_len'] - results[s]['gt_len']) / results[s]['gt_len'] * 100
               for s in sessions]
    bars = axes[2].bar(x, len_err, color=colors[:len(sessions)], alpha=0.8, edgecolor='black')
    axes[2].set_ylabel('Trajectory Length Error (%)', fontsize=12)
    axes[2].set_title('Trajectory Length Accuracy', fontsize=13, fontweight='bold')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, fontsize=9)
    axes[2].grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, len_err):
        axes[2].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{val:.1f}%', ha='center', fontsize=10, fontweight='bold')

    plt.suptitle('Cross-Season Performance Comparison', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(plots_dir / 'seasonal_bar_chart.png', dpi=150)
    print(f"  Saved: seasonal_bar_chart.png")
    plt.close()

    return table_text


if __name__ == '__main__':
    SESSIONS = ['2012-01-08', '2012-04-29', '2012-08-04', '2012-10-28']
    SUBSAMPLE = 2
    OUTPUT_DIR = PROJECT_ROOT / 'results' / 'week0_seasonal'
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    results = {}

    for session in SESSIONS:
        print(f"\n{'#'*80}")
        print(f"# SESSION: {session}")
        print(f"{'#'*80}")

        session_dir = OUTPUT_DIR / session
        session_dir.mkdir(parents=True, exist_ok=True)

        try:
            result = run_lidar_pipeline(session, subsample=SUBSAMPLE)
            if result is not None:
                results[session] = result

                np.savetxt(session_dir / 'odom_trajectory.txt',
                          result['odom_traj'], fmt='%.6f',
                          header='timestamp x y z qx qy qz qw')
                np.savetxt(session_dir / 'optimized_trajectory.txt',
                          result['opt_traj'], fmt='%.6f',
                          header='timestamp x y z qx qy qz qw')
                np.savetxt(session_dir / 'gt_trajectory.txt',
                          result['gt_traj'], fmt='%.6f',
                          header='timestamp x y z qx qy qz qw')

                # per-session plot
                fig, ax = plt.subplots(figsize=(14, 12))
                ax.plot(result['gt_traj'][:, 1], result['gt_traj'][:, 2],
                        'k-', lw=2, label='Ground Truth', alpha=0.8)
                ax.plot(result['opt_traj'][:, 1], result['opt_traj'][:, 2],
                        'r-', lw=1.5, label=f'Estimated (ATE={result["ate_opt"]["rmse"]:.0f}m)',
                        alpha=0.7)
                ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
                ax.set_title(f'{session} Trajectory', fontsize=14, fontweight='bold')
                ax.legend(fontsize=11); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
                plt.tight_layout()
                plt.savefig(session_dir / 'trajectory.png', dpi=150)
                plt.close()
                print(f"  Saved: {session_dir / 'trajectory.png'}")

        except Exception as e:
            print(f"\n  ERROR on {session}: {e}")
            import traceback
            traceback.print_exc()
            continue

    # --- Generate comparison ---
    if len(results) >= 2:
        print(f"\n{'#'*80}")
        print(f"# CROSS-SEASON COMPARISON")
        print(f"{'#'*80}")
        table = generate_comparison(results, OUTPUT_DIR)
    else:
        print(f"\nOnly {len(results)} session(s) completed. Need at least 2 for comparison.")

    print(f"\n{'='*80}")
    print("WEEK 0 COMPLETE")
    print(f"{'='*80}")
    print(f"Results: {OUTPUT_DIR}")
