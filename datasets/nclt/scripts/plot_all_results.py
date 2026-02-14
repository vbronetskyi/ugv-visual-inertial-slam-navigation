#!/usr/bin/env python3
"""Plot all SLAM results on NCLT (trajectories, ATE bars, tracking rates)"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from pathlib import Path
import json

RESULTS = Path('/workspace/datasets/nclt/results')
OUT_DIR = RESULTS / 'summary_plots'
OUT_DIR.mkdir(exist_ok=True)


def load_tum(path, skip_header=True):
    lines = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    vals = [float(p) for p in parts[:8]]
                    lines.append(vals)
                except ValueError:
                    continue
    if not lines:
        return None
    arr = np.array(lines)
    return arr


def load_orbslam3(path):
    """Load ORB-SLAM3 format (same as TUM but timestamps may be in nanoseconds)."""
    arr = load_tum(path)
    if arr is None:
        return None
    # convert nanosecond timestamps to seconds
    if arr[0, 0] > 1e15:
        arr[:, 0] = arr[:, 0] / 1e9
    elif arr[0, 0] > 1e9:
        arr[:, 0] = arr[:, 0] / 1e6
    return arr


def align_to_gt_2d(est_xyz, gt_xyz):
    """Align estimated trajectory to GT using Umeyama (Sim3) in 2D (x,y)."""
    # use only x, y
    src = est_xyz[:, :2].T  # 2xN
    tgt = gt_xyz[:, :2].T   # 2xN

    n = src.shape[1]
    if n < 3:
        return est_xyz

    mu_src = src.mean(axis=1, keepdims=True)
    mu_tgt = tgt.mean(axis=1, keepdims=True)

    src_c = src - mu_src
    tgt_c = tgt - mu_tgt

    cov = tgt_c @ src_c.T / n

    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[1, 1] = -1

    R = U @ S @ Vt

    var_src = np.sum(src_c ** 2) / n
    scale = np.sum(D * np.diag(S)) / var_src if var_src > 1e-10 else 1.0

    t = mu_tgt - scale * R @ mu_src

    aligned = np.copy(est_xyz)
    xy_aligned = scale * R @ est_xyz[:, :2].T + t
    aligned[:, 0] = xy_aligned[0]
    aligned[:, 1] = xy_aligned[1]

    return aligned


def sync_and_align(est, gt, max_diff_s=0.5):
    est_ts = est[:, 0]
    gt_ts = gt[:, 0]

    matched_est = []
    matched_gt = []

    for i, t in enumerate(est_ts):
        diffs = np.abs(gt_ts - t)
        j = np.argmin(diffs)
        if diffs[j] < max_diff_s:
            matched_est.append(est[i, 1:4])
            matched_gt.append(gt[j, 1:4])

    if len(matched_est) < 3:
        return None, None

    matched_est = np.array(matched_est)
    matched_gt = np.array(matched_gt)

    aligned = align_to_gt_2d(matched_est, matched_gt)
    return aligned, matched_gt


def compute_ate(est_xyz, gt_xyz):
    """Compute ATE RMSE"""
    diff = est_xyz - gt_xyz
    dists = np.linalg.norm(diff[:, :2], axis=1)  # 2D
    return np.sqrt(np.mean(dists**2)), np.mean(dists), np.median(dists)


# ============================================================
# LOAD ALL TRAJECTORIES
# ============================================================

print('Loading trajectories...')

trajs = {}

# --- Ground Truth (Spring 2012-04-29, full session) ---
gt_seasonal = load_tum(RESULTS / 'week0_seasonal/2012-04-29/gt_trajectory.txt')
gt_icp_dense = load_tum(RESULTS / 'week2_icp_loop_closure/gt_dense_trajectory.txt')
gt_synced = load_tum(RESULTS / 'week2_icp_loop_closure/gt_synced_trajectory.txt')

# use the most complete GT
gt = gt_seasonal if gt_seasonal is not None else gt_icp_dense
print(f"  GT: {len(gt)} poses")

# --- LiDAR ICP (Spring) ---
icp_dense = load_tum(RESULTS / 'week2_icp_loop_closure/icp_only_trajectory.txt')
if icp_dense is not None:
    aligned, gt_matched = sync_and_align(icp_dense, gt)
    if aligned is not None:
        trajs['LiDAR ICP\n(dense, 6.5k scans)'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#2196F3', 'n_poses': len(icp_dense),
            'method': 'lidar'
        }
        ate_rmse, ate_mean, ate_med = compute_ate(aligned, gt_matched)
        trajs['LiDAR ICP\n(dense, 6.5k scans)']['ate_rmse'] = ate_rmse
        print(f"  LiDAR ICP: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- LiDAR ICP + Loop Closure ---
icp_lc = load_tum(RESULTS / 'week2_icp_loop_closure/icp_lc_trajectory.txt')
if icp_lc is not None:
    aligned, gt_matched = sync_and_align(icp_lc, gt)
    if aligned is not None:
        trajs['LiDAR ICP+LC'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#00BCD4', 'n_poses': len(icp_lc),
            'method': 'lidar'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['LiDAR ICP+LC']['ate_rmse'] = ate_rmse
        print(f"  LiDAR ICP+LC: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- Seasonal optimized (Spring = LiDAR+Odom+GPS) ---
seasonal_opt = load_tum(RESULTS / 'week0_seasonal/2012-04-29/optimized_trajectory.txt')
if seasonal_opt is not None:
    aligned, gt_matched = sync_and_align(seasonal_opt, gt)
    if aligned is not None:
        trajs['LiDAR+Odom+GPS\n(optimized)'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#4CAF50', 'n_poses': len(seasonal_opt),
            'method': 'lidar'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['LiDAR+Odom+GPS\n(optimized)']['ate_rmse'] = ate_rmse
        print(f"  LiDAR+Odom+GPS: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- Week3 IMU+GPS optimized ---
w3_opt = load_tum(RESULTS / 'week3_imu_gps/optimized_trajectory.txt')
if w3_opt is not None:
    aligned, gt_matched = sync_and_align(w3_opt, gt)
    if aligned is not None:
        trajs['LiDAR+IMU+GPS\n(week3)'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#8BC34A', 'n_poses': len(w3_opt),
            'method': 'lidar'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['LiDAR+IMU+GPS\n(week3)']['ate_rmse'] = ate_rmse
        print(f"  LiDAR+IMU+GPS: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- DROID-SLAM (Spring, aligned) ---
droid = load_tum(RESULTS / 'week0_visual_slam/droid_slam/2012-04-29/trajectory_aligned.txt')
if droid is not None:
    aligned, gt_matched = sync_and_align(droid, gt, max_diff_s=2.0)
    if aligned is not None:
        trajs['DROID-SLAM'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#FF5722', 'n_poses': len(droid),
            'method': 'visual'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['DROID-SLAM']['ate_rmse'] = ate_rmse
        print(f"  DROID-SLAM: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- DPVO (Spring, aligned) ---
dpvo = load_tum(RESULTS / 'week0_visual_slam/dpvo/2012-04-29/trajectory_aligned.txt')
if dpvo is not None:
    aligned, gt_matched = sync_and_align(dpvo, gt, max_diff_s=2.0)
    if aligned is not None:
        trajs['DPVO'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#FF9800', 'n_poses': len(dpvo),
            'method': 'visual'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['DPVO']['ate_rmse'] = ate_rmse
        print(f"  DPVO: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- DPV-SLAM (Spring, aligned) ---
dpv = load_tum(RESULTS / 'week0_visual_slam/dpv_slam/2012-04-29/trajectory_aligned.txt')
if dpv is not None:
    aligned, gt_matched = sync_and_align(dpv, gt, max_diff_s=2.0)
    if aligned is not None:
        trajs['DPV-SLAM'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#F44336', 'n_poses': len(dpv),
            'method': 'visual'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['DPV-SLAM']['ate_rmse'] = ate_rmse
        print(f"  DPV-SLAM: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- ORB-SLAM3 mono (v2, spring 3k, best single-map segment) ---
orb_v2_3k = load_orbslam3(RESULTS / 'week0_orbslam3_v2/trajectory_spring_3k.txt')
if orb_v2_3k is not None:
    aligned, gt_matched = sync_and_align(orb_v2_3k, gt, max_diff_s=2.0)
    if aligned is not None:
        trajs['ORB-SLAM3 Mono\n(3k, single map)'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#9C27B0', 'n_poses': len(orb_v2_3k),
            'method': 'visual'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['ORB-SLAM3 Mono\n(3k, single map)']['ate_rmse'] = ate_rmse
        print(f"  ORB-SLAM3 Mono 3k: {len(aligned)} matched, ATE={ate_rmse:.1f}m")

# --- ORB-SLAM3 mono-inertial (B001: best VIO run, still massive drift) ---
orb_b001 = load_orbslam3(RESULTS / 'week0_orbslam3_proper/phase_b/B001_imu_v1/f_nclt.txt')
if orb_b001 is not None:
    aligned, gt_matched = sync_and_align(orb_b001, gt, max_diff_s=2.0)
    if aligned is not None:
        trajs['ORB-SLAM3 VIO\n(mono-inertial)'] = {
            'xy': aligned[:, :2], 'gt_xy': gt_matched[:, :2],
            'color': '#E91E63', 'n_poses': len(orb_b001),
            'method': 'visual'
        }
        ate_rmse, _, _ = compute_ate(aligned, gt_matched)
        trajs['ORB-SLAM3 VIO\n(mono-inertial)']['ate_rmse'] = ate_rmse
        print(f"  ORB-SLAM3 VIO: {len(aligned)} matched, ATE={ate_rmse:.1f}m")


print(f"\nTotal methods loaded: {len(trajs)}")

# ============================================================
# FIGURE 1: Bird's-eye trajectory comparison (ALL methods)
# ============================================================

print("\nPlotting Figure 1: All trajectories bird's-eye view...")

fig, axes = plt.subplots(2, 2, figsize=(20, 20))
fig.suptitle('NCLT Spring 2012-04-29: All SLAM Methods Comparison\n(Bird\'s-eye view, X-Y plane)',
             fontsize=18, fontweight='bold', y=0.98)

# panel 1: LiDAR methods + GT
ax = axes[0, 0]
ax.set_title('LiDAR-based Methods', fontsize=14, fontweight='bold')
ax.plot(gt[:, 1], gt[:, 2], 'k-', linewidth=2.5, alpha=0.4, label='Ground Truth', zorder=1)
for name, data in trajs.items():
    if data['method'] == 'lidar':
        ax.plot(data['xy'][:, 0], data['xy'][:, 1], '-',
                color=data['color'], linewidth=1.5, alpha=0.8,
                label=f"{name.replace(chr(10), ' ')} (ATE={data['ate_rmse']:.1f}m)", zorder=2)
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.legend(fontsize=9, loc='best')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# panel 2: Visual SLAM methods + GT
ax = axes[0, 1]
ax.set_title('Visual SLAM Methods (Deep Learning)', fontsize=14, fontweight='bold')
ax.plot(gt[:, 1], gt[:, 2], 'k-', linewidth=2.5, alpha=0.4, label='Ground Truth', zorder=1)
for name, data in trajs.items():
    if data['method'] == 'visual' and 'ORB' not in name:
        ax.plot(data['xy'][:, 0], data['xy'][:, 1], '-',
                color=data['color'], linewidth=1.5, alpha=0.8,
                label=f"{name.replace(chr(10), ' ')} (ATE={data['ate_rmse']:.1f}m)", zorder=2)
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.legend(fontsize=9, loc='best')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# panel 3: ORB-SLAM3 methods + GT
ax = axes[1, 0]
ax.set_title('ORB-SLAM3 Methods', fontsize=14, fontweight='bold')
ax.plot(gt[:, 1], gt[:, 2], 'k-', linewidth=2.5, alpha=0.4, label='Ground Truth', zorder=1)
for name, data in trajs.items():
    if 'ORB' in name:
        ax.plot(data['xy'][:, 0], data['xy'][:, 1], '-',
                color=data['color'], linewidth=1.5, alpha=0.8,
                label=f"{name.replace(chr(10), ' ')} (ATE={data['ate_rmse']:.1f}m)", zorder=2)
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.legend(fontsize=9, loc='best')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# panel 4: Best of each category
ax = axes[1, 1]
ax.set_title('Best Method per Category', fontsize=14, fontweight='bold')
ax.plot(gt[:, 1], gt[:, 2], 'k-', linewidth=3, alpha=0.5, label='Ground Truth', zorder=1)

# pick best from each category
best_picks = {}
for name, data in trajs.items():
    cat = data['method']
    if 'ORB' in name:
        cat = 'orbslam'
    if cat not in best_picks or data['ate_rmse'] < best_picks[cat][1]['ate_rmse']:
        best_picks[cat] = (name, data)

for name, data in best_picks.values():
    ax.plot(data['xy'][:, 0], data['xy'][:, 1], '-',
            color=data['color'], linewidth=2, alpha=0.9,
            label=f"{name.replace(chr(10), ' ')} (ATE={data['ate_rmse']:.1f}m)", zorder=2)

ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.legend(fontsize=10, loc='best')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(OUT_DIR / 'fig1_all_trajectories.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig1_all_trajectories.png'}")
plt.close()


# ============================================================
# FIGURE 2: ATE comparison bar chart
# ============================================================

print("Plotting Figure 2: ATE comparison...")

# collect all ATE values (including from JSON summaries)
ate_data = {}
for name, data in trajs.items():
    clean_name = name.replace('\n', ' ')
    ate_data[clean_name] = data['ate_rmse']

# add values from JSON files that may not have trajectories loaded
# seasonal results
seasonal_ates = {
    'LiDAR ICP+Odom+GPS (Winter)': 30.2,
    'LiDAR ICP+Odom+GPS (Spring)': 174.0,
    'LiDAR ICP+Odom+GPS (Summer)': 188.2,
    'LiDAR ICP+Odom+GPS (Autumn)': 151.1,
}

# deep SLAM summer
deep_summer = {
    'DROID-SLAM (Summer)': 142.3,
    'DPVO (Summer)': 183.4,
    'DPV-SLAM (Summer)': 193.6,
}

# ORB-SLAM3 v2 values
orb_v2_ates = {
    'ORB-SLAM3 Mono 3k (best)': 45.6,
    'ORB-SLAM3 Mono Full': 100.3,
}

all_ates = {**ate_data, **seasonal_ates, **deep_summer, **orb_v2_ates}

# sort by ATE
sorted_ates = sorted(all_ates.items(), key=lambda x: x[1])

# color mapping
def get_bar_color(name):
    # TODO proper logging instead of log() hack
    if 'Winter' in name or 'Summer' in name or 'Autumn' in name or 'Spring' in name:
        return '#4CAF50'  # seasonal green
    if 'LiDAR' in name:
        return '#2196F3'  # blue
    if 'DROID' in name:
        return '#FF5722'
    if 'DPVO' in name:
        return '#FF9800'
    if 'DPV-SLAM' in name:
        return '#F44336'
    if 'ORB' in name and 'VIO' in name:
        return '#E91E63'
    if 'ORB' in name:
        return '#9C27B0'
    return '#607D8B'

fig, ax = plt.subplots(figsize=(16, 10))

names = [x[0] for x in sorted_ates]
values = [x[1] for x in sorted_ates]
colors = [get_bar_color(n) for n in names]

bars = ax.barh(range(len(names)), values, color=colors, alpha=0.85, edgecolor='white')

# add value labels
for i, (v, bar) in enumerate(zip(values, bars)):
    if v > 500:
        ax.text(min(v, ax.get_xlim()[1]) - 5, i, f'{v:.0f}m',
                ha='right', va='center', fontsize=10, fontweight='bold', color='white')
    else:
        ax.text(v + 3, i, f'{v:.1f}m', ha='left', va='center', fontsize=10)

ax.set_yticks(range(len(names)))
ax.set_yticklabels(names, fontsize=10)
ax.set_xlabel('ATE RMSE (meters)', fontsize=13)
ax.set_title('Absolute Trajectory Error (RMSE), All Methods\nNCLT Dataset, Lower is Better',
             fontsize=15, fontweight='bold')
ax.grid(axis='x', alpha=0.3)
ax.invert_yaxis()

# legendfrom matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor='#2196F3', label='LiDAR ICP'),
    Patch(facecolor='#4CAF50', label='LiDAR+Odom+GPS (seasonal)'),
    Patch(facecolor='#FF5722', label='DROID-SLAM'),
    Patch(facecolor='#FF9800', label='DPVO'),
    Patch(facecolor='#F44336', label='DPV-SLAM'),
    Patch(facecolor='#9C27B0', label='ORB-SLAM3 Mono'),
    Patch(facecolor='#E91E63', label='ORB-SLAM3 VIO'),
]
ax.legend(handles=legend_elements, loc='lower right', fontsize=9)

plt.tight_layout()
fig.savefig(OUT_DIR / 'fig2_ate_comparison.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig2_ate_comparison.png'}")
plt.close()


# ============================================================
# FIGURE 3: Tracking rate & coverage comparison
# ============================================================

print('Plotting Figure 3: Tracking rates...')

tracking_data = [
    ('DPVO (Spring)', 100.0, 21510, '#FF9800'),
    ('ORB-SLAM3 Mono 3k', 99.4, 2981, '#9C27B0'),
    ('ORB-SLAM3 VIO f=386', 90.2, 2706, '#E91E63'),
    ('DPV-SLAM (Summer)', 90.6, 21858, '#F44336'),
    ('DPVO (Summer)', 88.0, 21238, '#FF9800'),
    ('ORB-SLAM3 Mono (full)', 24.0, 5038, '#9C27B0'),
    ('ORB-SLAM3 Mono (Summer)', 19.2, 959, '#9C27B0'),
    ('DROID-SLAM (Spring)', 8.6, 1847, '#FF5722'),
    ('DPV-SLAM (Spring)', 8.3, 1780, '#F44336'),
    ('DROID-SLAM (Summer)', 3.9, 933, '#FF5722'),
    ('ORB-SLAM3 VIO f=290', 79.7, 2390, '#E91E63'),
    ('ORB-SLAM3 Pinhole', 2.6, 79, '#9C27B0'),
]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

tracking_data_sorted = sorted(tracking_data, key=lambda x: x[1], reverse=True)
names_t = [x[0] for x in tracking_data_sorted]
rates = [x[1] for x in tracking_data_sorted]
colors_t = [x[3] for x in tracking_data_sorted]

bars = ax1.barh(range(len(names_t)), rates, color=colors_t, alpha=0.85, edgecolor='white')
for i, (r, bar) in enumerate(zip(rates, bars)):
    ax1.text(r + 1, i, f'{r:.1f}%', ha='left', va='center', fontsize=10)

ax1.set_yticks(range(len(names_t)))
ax1.set_yticklabels(names_t, fontsize=10)
ax1.set_xlabel('Tracking Rate (%)', fontsize=12)
ax1.set_title('Tracking Rate (% frames tracked)\nHigher is Better', fontsize=14, fontweight='bold')
ax1.set_xlim(0, 115)
ax1.grid(axis='x', alpha=0.3)
ax1.invert_yaxis()

# number of poses (log scale)
n_poses_data = sorted(tracking_data, key=lambda x: x[2], reverse=True)
names_p = [x[0] for x in n_poses_data]
n_poses = [x[2] for x in n_poses_data]
colors_p = [x[3] for x in n_poses_data]

bars = ax2.barh(range(len(names_p)), n_poses, color=colors_p, alpha=0.85, edgecolor='white')
for i, (n, bar) in enumerate(zip(n_poses, bars)):
    ax2.text(n * 1.1, i, f'{n:,}', ha='left', va='center', fontsize=10)

ax2.set_yticks(range(len(names_p)))
ax2.set_yticklabels(names_p, fontsize=10)
ax2.set_xlabel('Number of Estimated Poses', fontsize=12)
ax2.set_title('Output Poses Count\n(log scale)', fontsize=14, fontweight='bold')
ax2.set_xscale('log')
ax2.grid(axis='x', alpha=0.3)
ax2.invert_yaxis()

plt.tight_layout()
fig.savefig(OUT_DIR / 'fig3_tracking_rates.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig3_tracking_rates.png'}")
plt.close()


# ============================================================
# FIGURE 4: Seasonal comparison (LiDAR pipeline)
# ============================================================

print("Plotting Figure 4: Seasonal trajectories...")

sessions = ['2012-01-08', '2012-04-29', '2012-08-04', '2012-10-28']
season_names = ['Winter', 'Spring', 'Summer', 'Autumn']
season_ates = [30.2, 174.0, 188.2, 151.1]
season_colors = ['#2196F3', '#4CAF50', '#FF9800', '#795548']

fig, axes = plt.subplots(2, 2, figsize=(18, 18))
fig.suptitle('Seasonal LiDAR SLAM Results (ICP + Odom + GPS Loop Closure)\nNCLT Dataset, All 4 Seasons',
             fontsize=16, fontweight='bold', y=0.98)

for idx, (session, sname, ate, scolor) in enumerate(zip(sessions, season_names, season_ates, season_colors)):
    ax = axes[idx // 2, idx % 2]

    gt_path = RESULTS / f'week0_seasonal/{session}/gt_trajectory.txt'
    opt_path = RESULTS / f'week0_seasonal/{session}/optimized_trajectory.txt'
    odom_path = RESULTS / f'week0_seasonal/{session}/odom_trajectory.txt'

    if gt_path.exists():
        gt_s = load_tum(gt_path)
        if gt_s is not None:
            ax.plot(gt_s[:, 1], gt_s[:, 2], 'k-', linewidth=2, alpha=0.4, label='Ground Truth')

    if opt_path.exists():
        opt_s = load_tum(opt_path)
        if opt_s is not None and gt_s is not None:
            aligned, gt_m = sync_and_align(opt_s, gt_s)
            if aligned is not None:
                ax.plot(aligned[:, 0], aligned[:, 1], '-', color=scolor,
                       linewidth=1.5, alpha=0.8, label=f'Optimized (ATE={ate:.1f}m)')

    if odom_path.exists():
        odom_s = load_tum(odom_path)
        if odom_s is not None and gt_s is not None:
            aligned_o, gt_o = sync_and_align(odom_s, gt_s)
            if aligned_o is not None:
                ax.plot(aligned_o[:, 0], aligned_o[:, 1], '--', color='gray',
                       linewidth=1, alpha=0.5, label='Raw Odometry')

    ax.set_title(f'{sname} ({session}), ATE: {ate:.1f}m', fontsize=13, fontweight='bold')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.legend(fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(OUT_DIR / 'fig4_seasonal.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig4_seasonal.png'}")
plt.close()


# ============================================================
# FIGURE 5: ORB-SLAM3 scale drift visualization
# ============================================================

print("Plotting Figure 5: ORB-SLAM3 scale drift...")

fig, axes = plt.subplots(1, 2, figsize=(18, 8))
fig.suptitle('ORB-SLAM3 on NCLT: Scale Drift Problem\nMono-Inertial (VIO) Cannot Estimate Scale at 5fps + 47Hz MEMS IMU',
             fontsize=14, fontweight='bold')

# left: VIO trajectory (raw, showing severe scale drift)
if orb_b001 is not None:
    ax = axes[0]
    ax.plot(orb_b001[:, 1], orb_b001[:, 2], '-', color='#E91E63', linewidth=0.5, alpha=0.7)
    ax.set_title(f'ORB-SLAM3 VIO (raw trajectory)\n{len(orb_b001)} poses, scale drift up to 200km!',
                fontsize=12, fontweight='bold')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.grid(True, alpha=0.3)

    # add text showing the scale
    max_coord = max(np.max(np.abs(orb_b001[:, 1])), np.max(np.abs(orb_b001[:, 2])))
    ax.text(0.05, 0.95, f'Max coordinate: {max_coord:.0f}m\nGT trajectory: ~700m\nScale error: {max_coord/700:.0f}x',
            transform=ax.transAxes, fontsize=11, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# right: Mono trajectory (no IMU), stays small but no scale
orb_v2_full = load_orbslam3(RESULTS / 'week0_orbslam3_v2/trajectory_spring_full.txt')
if orb_v2_full is not None:
    ax = axes[1]
    ax.plot(orb_v2_full[:, 1], orb_v2_full[:, 2], '-', color='#9C27B0', linewidth=0.5, alpha=0.7)
    ax.set_title(f'ORB-SLAM3 Mono (raw, no IMU)\n{len(orb_v2_full)} poses, 24% tracking, no metric scale',
                fontsize=12, fontweight='bold')
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.grid(True, alpha=0.3)

    path_len = np.sum(np.linalg.norm(np.diff(orb_v2_full[:, 1:4], axis=0), axis=1))
    ax.text(0.05, 0.95, f'Est. path: {path_len:.1f}m\nGT path: ~3186m\nScale: {path_len/3186:.3f}x',
            transform=ax.transAxes, fontsize=11, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

plt.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(OUT_DIR / 'fig5_orbslam3_drift.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig5_orbslam3_drift.png'}")
plt.close()


# ============================================================
# FIGURE 6: Summary dashboard
# ============================================================

print("Plotting Figure 6: Summary dashboard...")

fig = plt.figure(figsize=(22, 14))

# create a nice summary table
summary_rows = [
    ['Method', 'Sensor', 'Session', 'ATE RMSE (m)', 'Tracking %', 'Status'],
    ['LiDAR ICP+Odom+GPS', 'Velodyne HDL-32E', 'Winter', '30.2', '100%', 'BEST'],
    ['ORB-SLAM3 Mono (3k)', 'Ladybug3 Cam5', 'Spring', '45.6', '99.4%', 'Single map only'],
    ['ORB-SLAM3 Mono (full)', 'Ladybug3 Cam5', 'Spring', '100.3', '24.0%', '35 maps, unreliable'],
    ['DROID-SLAM', 'Ladybug3 Cam5', 'Spring', '110.0', '8.6%', 'Doesn\'t follow road'],
    ['DPVO', 'Ladybug3 Cam5', 'Spring', '142.0', '100%', 'Doesn\'t follow road'],
    ['LiDAR ICP+Odom+GPS', 'Velodyne HDL-32E', 'Autumn', '151.1', '100%', 'Drift'],
    ['DPV-SLAM', 'Ladybug3 Cam5', 'Spring', '166.2', '8.3%', 'Doesn\'t follow road'],
    ['LiDAR ICP+Odom+GPS', 'Velodyne HDL-32E', 'Spring', '174.0', '100%', 'Drift'],
    ['LiDAR ICP+Odom+GPS', 'Velodyne HDL-32E', 'Summer', '188.2', '100%', 'Drift'],
    ['ORB-SLAM3 VIO', 'Cam5 + MS25 IMU', 'Spring', '>10000', '90.2%', 'severe scale drift'],
    ['ORB-SLAM3 Stereo', 'Cam4+5 / Cam5+1', 'Spring', 'N/A', '0.03%', 'FAILED (no overlap)'],
]

ax = fig.add_subplot(111)
ax.axis('off')

table = ax.table(cellText=summary_rows[1:], colLabels=summary_rows[0],
                 cellLoc='center', loc='center',
                 colWidths=[0.2, 0.15, 0.1, 0.12, 0.1, 0.25])

table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1, 2)

# header style
for j in range(len(summary_rows[0])):
    cell = table[0, j]
    cell.set_facecolor('#37474F')
    cell.set_text_props(color='white', fontweight='bold')

# row colors
for i in range(1, len(summary_rows)):
    row = summary_rows[i]
    for j in range(len(row)):
        cell = table[i, j]
        if 'BEST' in row[5]:
            cell.set_facecolor('#E8F5E9')
        elif 'severe scale drift' in row[5] or 'FAILED' in row[5]:
            cell.set_facecolor('#FFEBEE')
        elif 'Drift' in row[5] or 'unreliable' in row[5]:
            cell.set_facecolor('#FFF3E0')
        elif 'Doesn\'t follow' in row[5]:
            cell.set_facecolor('#FFF8E1')
        else:
            cell.set_facecolor('#F5F5F5')

ax.set_title('NCLT SLAM Experiments: Complete Results Summary\n'
             'Experiments 0.1-0.6: LiDAR ICP, ORB-SLAM3 (Mono/VIO/Stereo), DROID-SLAM, DPVO, DPV-SLAM\n',
             fontsize=16, fontweight='bold', pad=20)

# add footnotes
footnote = (
    "Key findings:\n"
    "• LiDAR ICP + Odom + GPS is the only reliable pipeline (Winter: 30.2m ATE)\n"
    "• ORB-SLAM3 Mono tracks well in single map (99.4%) but cannot maintain across session\n"
    "- ORB-SLAM3 VIO fails: 47 Hz MEMS IMU + 5 fps camera are insufficient for scale estimation\n"
    "• Deep SLAM methods (DROID/DPVO/DPV) track but produce trajectories that don't follow road shape\n"
    "• Ladybug3 cameras: 120° FOV fisheye, 5fps, cam0=sky-facing → only cam1/4/5 usable\n"
    "• Stereo impossible: 72° angle between adjacent cameras = zero overlap"
)
fig.text(0.05, 0.02, footnote, fontsize=10, verticalalignment='bottom',
         fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout(rect=[0, 0.15, 1, 1])
fig.savefig(OUT_DIR / 'fig6_summary_dashboard.png', dpi=150, bbox_inches='tight')
print(f"  Saved: {OUT_DIR / 'fig6_summary_dashboard.png'}")
plt.close()


print(f"\n{'='*60}")
print(f"All plots saved to: {OUT_DIR}")
print(f"{'='*60}")

# list all generated files
for f in sorted(OUT_DIR.glob('*.png')):
    size_kb = f.stat().st_size / 1024
    print(f"  {f.name} ({size_kb:.0f} KB)")
