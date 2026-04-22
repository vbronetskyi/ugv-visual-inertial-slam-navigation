#!/usr/bin/env python3
"""generator for results/final/*.png thesis figures

reads routes/_common/metrics.json + raw logs + csv, writes 15 figures into
results/final/.  all figures use consistent palette, fonts, labels.  run with:

    python3 scripts/analysis/make_final_plots.py

ran this ~20 times while tuning.  if you add a new figure, bump the number
prefix (07 onwards) and add a plot_NN function
"""
import os
import re
import json
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D


ROOT = Path('/workspace/simulation/isaac')
OUT = ROOT / 'results' / 'final'
OUT.mkdir(parents=True, exist_ok=True)

# consistent palette across all figures
COL_OURS   = '#1f77b4'   # blue
COL_STOCK  = '#d62728'   # red
COL_RGBD   = '#2ca02c'   # green
COL_GT     = '#555555'   # dark grey
COL_WP     = '#ff9f1c'   # orange
COL_ANCHOR = '#9467bd'   # purple
COL_OBST   = '#7f7f7f'   # grey

METHODS = [('our custom', 'ours',  COL_OURS),
           ('exp 74 stock', 'stock Nav2', COL_STOCK),
           ('exp 76 RGB-D', 'RGB-D-no-IMU', COL_RGBD)]

ROUTES = ['01_road', '02_north_forest', '03_south',
          '04_nw_se', '05_ne_sw', '06_nw_ne',
          '07_se_sw', '08_nw_sw', '09_se_ne']

ROUTE_LABELS = {
    '01_road':         'road',
    '02_north_forest': 'N-forest',
    '03_south':        'south',
    '04_nw_se':        'NW-SE',
    '05_ne_sw':        'NE-SW',
    '06_nw_ne':        'NW-NE',
    '07_se_sw':        'SE-SW',
    '08_nw_sw':        'NW-SW',
    '09_se_ne':        'SE-NE',
}


def load_metrics():
    return json.load(open(ROOT / 'routes' / '_common' / 'metrics.json'))


def parse_tf_slam_err(log_path):
    """pull (t_offset_s, err_m) pairs from tf_slam.log 'err=N.Nm' lines"""
    if not log_path or not os.path.exists(log_path):
        return np.array([]), np.array([])
    errs = []
    t0 = None
    pat = re.compile(r'\[(\d+\.\d+)\].*err=(\d+\.\d+)m')
    with open(log_path) as f:
        for line in f:
            m = pat.search(line)
            if not m: continue
            t = float(m.group(1))
            e = float(m.group(2))
            if t0 is None: t0 = t
            errs.append((t - t0, e))
    if not errs: return np.array([]), np.array([])
    arr = np.array(errs)
    return arr[:, 0], arr[:, 1]


# figure 07: headline WP coverage bars

def plot_07_headline(metrics):
    # 9 routes x 3 methods
    x = np.arange(len(ROUTES))
    width = 0.27
    fig, ax = plt.subplots(figsize=(13, 5.2))

    for i, (key, label, color) in enumerate(METHODS):
        vals = [metrics.get(r, {}).get(key, {}).get('cov_pct', 0) for r in ROUTES]
        ax.bar(x + (i-1)*width, vals, width, label=label, color=color, edgecolor='white')

    ax.set_xticks(x)
    ax.set_xticklabels([ROUTE_LABELS[r] for r in ROUTES])
    ax.set_ylabel('WP coverage (%)')
    ax.set_ylim(0, 110)
    ax.axhline(100, color='grey', lw=0.5, ls='--', alpha=0.5)
    ax.set_title('GT-verified WP coverage: ours vs stock Nav2 vs RGB-D-no-IMU')
    ax.legend(loc='upper right')
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '07_headline_9routes_3methods.png', dpi=130)
    plt.close()
    print('07_headline_9routes_3methods.png')


# figure 08: drift mean+p95

def plot_08_drift(metrics):
    x = np.arange(len(ROUTES))
    width = 0.27
    fig, ax = plt.subplots(figsize=(13, 5.2))

    for i, (key, label, color) in enumerate(METHODS):
        means = [metrics.get(r, {}).get(key, {}).get('drift_mean', 0) or 0 for r in ROUTES]
        p95s  = [metrics.get(r, {}).get(key, {}).get('drift_p95', 0) or 0 for r in ROUTES]
        ax.bar(x + (i-1)*width, means, width, color=color, alpha=0.85, label=f'{label} (mean)')
        ax.bar(x + (i-1)*width, [p - m for p, m in zip(p95s, means)], width,
               bottom=means, color=color, alpha=0.35, label=f'{label} (p95)' if i==0 else None)

    ax.set_xticks(x)
    ax.set_xticklabels([ROUTE_LABELS[r] for r in ROUTES])
    ax.set_ylabel('localisation drift (m)')
    ax.set_title('localisation drift vs GT: mean bar + p95 translucent top')
    handles = [patches.Patch(color=c, label=l) for _, l, c in METHODS]
    ax.legend(handles=handles, loc='upper left')
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '08_drift_comparison.png', dpi=130)
    plt.close()
    print('08_drift_comparison.png')


# figure 09: endpoint scatter

def plot_09_endpoint(metrics):
    fig, ax = plt.subplots(figsize=(9, 7.5))
    for key, label, color in METHODS:
        xs, ys, names = [], [], []
        for r in ROUTES:
            m = metrics.get(r, {}).get(key, {})
            if not m: continue
            xs.append(m.get('final_d', 100) or 100)
            ys.append(m.get('return_d', 100) or 100)
            names.append(ROUTE_LABELS[r])
        ax.scatter(xs, ys, c=color, s=130, label=label, alpha=0.85, edgecolors='white', linewidths=1)
        for x, y, n in zip(xs, ys, names):
            if x < 30 and y < 30:
                ax.annotate(n, (x, y), xytext=(5, 5), textcoords='offset points', fontsize=8)

    ax.axvline(10, color='orange', ls='--', alpha=0.5, label='10 m pass')
    ax.axhline(10, color='orange', ls='--', alpha=0.5)
    ax.axvline(5, color='green', ls=':', alpha=0.6, label='5 m strong')
    ax.axhline(5, color='green', ls=':', alpha=0.6)
    ax.set_xlabel('final_reach_error (m) — distance to turnaround at closest')
    ax.set_ylabel('return_error (m) — distance from spawn at end')
    ax.set_xscale('symlog', linthresh=1.0)
    ax.set_yscale('symlog', linthresh=1.0)
    ax.set_title('endpoint success: 9 routes x 3 methods')
    ax.legend(loc='upper left')
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '09_endpoint_success_scatter.png', dpi=130)
    plt.close()
    print('09_endpoint_success_scatter.png')


# figure 10: drift over time for route 09

def plot_10_drift_timeline():
    runs = {
        'ours (route 09)': ROOT / 'experiments' / '52_obstacles_v9' / 'teach' / 'tf_slam.log',
        'stock Nav2':      ROOT / 'experiments' / '74_pure_stock_nav2_baseline' / 'results' / 'run_09' / 'tf_slam.log',
        'RGB-D no IMU':    ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'tf_slam.log',
    }
    colors = {'ours (route 09)': COL_OURS, 'stock Nav2': COL_STOCK, 'RGB-D no IMU': COL_RGBD}
    fig, ax = plt.subplots(figsize=(13, 5))
    for name, path in runs.items():
        t, e = parse_tf_slam_err(path)
        if len(t) == 0: continue
        ax.plot(t, e, color=colors[name], lw=1.4, label=f'{name} (n={len(t)})', alpha=0.85)

    # overlay anchor events if we have them
    anchors = ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'anchor_matches.csv'
    if anchors.exists():
        t0 = None
        with open(anchors) as f:
            r = csv.DictReader(f)
            ats = []
            for row in r:
                ts = float(row['ts'])
                if t0 is None: t0 = ts
                if 'published' in row.get('outcome', ''):
                    ats.append(ts - t0)
        if ats:
            for a in ats[:200]:
                ax.axvline(a, color=COL_ANCHOR, alpha=0.08, lw=0.6)
            ax.plot([], [], color=COL_ANCHOR, alpha=0.7, lw=2, label='anchor fires (RGB-D run)')

    ax.set_xlabel('time (s)')
    ax.set_ylabel('drift vs GT (m)')
    ax.set_title('localisation drift over time - route 09 SE->NE')
    ax.legend(loc='upper left')
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '10_drift_over_time.png', dpi=130)
    plt.close()
    print('10_drift_over_time.png')


# figure 11: anchors on trajectory for route 09

def plot_11_anchors_on_traj():
    anchor_path = ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'anchor_matches.csv'
    # 76/run_09 has anchor_matches but no traj_gt, borrow 74/run_09 traj_gt (same route = same GT)
    traj_path = ROOT / 'experiments' / '74_pure_stock_nav2_baseline' / 'results' / 'run_09' / 'traj_gt.csv'
    if not (anchor_path.exists() and traj_path.exists()):
        print('11 skipped - missing data')
        return
    gt = np.loadtxt(traj_path, delimiter=',', skiprows=1)
    if gt.ndim == 1: return

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.plot(gt[:, 1], gt[:, 2], color=COL_GT, lw=1.5, label='GT trajectory', alpha=0.7)

    # anchor points
    with open(anchor_path) as f:
        r = csv.DictReader(f)
        pts = []
        for row in r:
            if 'published' not in row.get('outcome', ''): continue
            try:
                pts.append((float(row['anchor_x']), float(row['anchor_y']),
                            int(row['best_n_inliers'])))
            except (ValueError, KeyError):
                continue
    if pts:
        pts = np.array(pts)
        sc = ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], cmap='viridis',
                        s=40, alpha=0.75, edgecolors='white', linewidths=0.4)
        cb = plt.colorbar(sc, ax=ax)
        cb.set_label('PnP inlier count')

    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'anchor events on route 09 SE->NE (n={len(pts)} accepted)')
    ax.legend(loc='upper left')
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '11_anchors_on_trajectory.png', dpi=130)
    plt.close()
    print('11_anchors_on_trajectory.png')


# figure 12: obstacle detour 4-panel zoom for route 07_se_sw

def plot_12_obstacle_detour():
    # plans from 76 run, borrow 74 traj_gt (same route 09 = same GT path)
    plan_path = ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'plans' / 'plans_summary.csv'
    traj_path = ROOT / 'experiments' / '74_pure_stock_nav2_baseline' / 'results' / 'run_09' / 'traj_gt.csv'
    if not (plan_path.exists() and traj_path.exists()):
        print('12 skipped - no plans data')
        return
    gt = np.loadtxt(traj_path, delimiter=',', skiprows=1)
    plans = []
    with open(plan_path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                plans.append([float(row['wall_ts']), float(row['robot_x']), float(row['robot_y']),
                              float(row['goal_x']), float(row['goal_y'])])
            except (ValueError, KeyError):
                continue
    if not plans: return
    plans = np.array(plans)

    # pick 4 interesting moments: early, early-mid, mid, late
    t_min, t_max = plans[0, 0], plans[-1, 0]
    ticks = np.linspace(t_min, t_max, 6)[1:5]

    fig, axes = plt.subplots(1, 4, figsize=(16, 4.5))
    gt_t0 = gt[0, 0]
    plan_t0 = plans[0, 0]
    for ax, t_abs in zip(axes, ticks):
        t_rel = t_abs - plan_t0
        # trajectory up to this tick
        gt_rel = gt[:, 0]
        mask = gt_rel <= t_rel + 5
        ax.plot(gt[mask, 1], gt[mask, 2], color=COL_GT, lw=1.3)

        # most recent plan at this tick
        idx = int(np.argmin(np.abs(plans[:, 0] - t_abs)))
        p = plans[idx]
        ax.plot([p[1], p[3]], [p[2], p[4]], color=COL_OURS, lw=1.8, alpha=0.7)
        ax.scatter([p[1]], [p[2]], color=COL_OURS, s=80, zorder=5, label='robot')
        ax.scatter([p[3]], [p[4]], color=COL_WP, s=80, marker='*', zorder=5, label='goal')

        ax.set_aspect('equal')
        ax.set_title(f't={t_rel:.0f}s')
        ax.grid(alpha=0.3)
        # zoom in on robot
        ax.set_xlim(p[1] - 10, p[1] + 10)
        ax.set_ylim(p[2] - 10, p[2] + 10)
    axes[0].legend(loc='upper left', fontsize=8)
    fig.suptitle('detour sequence: robot + most-recent Nav2 plan at 4 timepoints')
    plt.tight_layout()
    plt.savefig(OUT / '12_obstacle_detour_zoom.png', dpi=130)
    plt.close()
    print('12_obstacle_detour_zoom.png')


# figure 13: coverage heatmap 9x3

def plot_13_heatmap(metrics):
    grid = np.zeros((len(ROUTES), 3))
    for i, r in enumerate(ROUTES):
        for j, (key, _, _) in enumerate(METHODS):
            grid[i, j] = metrics.get(r, {}).get(key, {}).get('cov_pct', 0) or 0
    fig, ax = plt.subplots(figsize=(6.5, 7))
    im = ax.imshow(grid, cmap='RdYlGn', vmin=0, vmax=100, aspect='auto')
    ax.set_yticks(range(len(ROUTES)))
    ax.set_yticklabels([ROUTE_LABELS[r] for r in ROUTES])
    ax.set_xticks(range(3))
    ax.set_xticklabels([lab for _, lab, _ in METHODS], rotation=15)
    for i in range(len(ROUTES)):
        for j in range(3):
            ax.text(j, i, f'{grid[i,j]:.0f}%', ha='center', va='center',
                    color='white' if grid[i,j] < 35 or grid[i,j] > 65 else 'black', fontsize=9)
    plt.colorbar(im, ax=ax, label='WP coverage (%)')
    ax.set_title('WP coverage heatmap: routes x methods')
    plt.tight_layout()
    plt.savefig(OUT / '13_coverage_heatmap.png', dpi=130)
    plt.close()
    print('13_coverage_heatmap.png')


# figure 14: imu fidelity ablation from exp 70

def plot_14_imu_ablation():
    variants = {
        'baseline': ROOT / 'experiments' / '70_imu_fidelity' / 'baseline' / 'drift_monitor.log',
        'run A (accel noise)': ROOT / 'experiments' / '70_imu_fidelity' / 'run_A_accel_noise' / 'drift_monitor.log',
    }
    fig, ax = plt.subplots(figsize=(12, 5))
    for name, path in variants.items():
        if not path.exists(): continue
        ts, errs = [], []
        t0 = None
        pat = re.compile(r'drift.*?(\d+\.\d+)m')
        with open(path) as f:
            for i, line in enumerate(f):
                m = pat.search(line)
                if not m: continue
                e = float(m.group(1))
                ts.append(i)
                errs.append(e)
        if ts:
            ax.plot(ts, errs, lw=1.3, alpha=0.85, label=f'{name} (n={len(ts)})')
    ax.set_xlabel('sample index')
    ax.set_ylabel('VIO-GT drift (m)')
    ax.set_title('IMU fidelity ablation - exp 70 on road route')
    ax.legend()
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '14_imu_fidelity_ablation.png', dpi=130)
    plt.close()
    print('14_imu_fidelity_ablation.png')


# figure 15: WP skip distribution from metrics (visited vs total per route x method)

def plot_15_wp_skips(metrics):
    fig, ax = plt.subplots(figsize=(13, 5))
    x = np.arange(len(ROUTES))
    width = 0.27
    for i, (key, label, color) in enumerate(METHODS):
        visited = [metrics.get(r, {}).get(key, {}).get('cov_visited', 0) or 0 for r in ROUTES]
        total   = [metrics.get(r, {}).get(key, {}).get('cov_total', 0) or 0 for r in ROUTES]
        missed  = [t - v for v, t in zip(visited, total)]
        ax.bar(x + (i-1)*width, visited, width, color=color, label=f'{label} visited')
        ax.bar(x + (i-1)*width, missed, width, bottom=visited,
               color=color, alpha=0.35, hatch='//', label=f'{label} missed' if i==0 else None)
    ax.set_xticks(x)
    ax.set_xticklabels([ROUTE_LABELS[r] for r in ROUTES])
    ax.set_ylabel('waypoints')
    ax.set_title('visited vs missed waypoints per route')
    handles = [patches.Patch(color=c, label=l) for _, l, c in METHODS]
    handles.append(patches.Patch(facecolor='white', edgecolor='grey', hatch='//', label='missed'))
    ax.legend(handles=handles, loc='upper right')
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '15_wp_skip_distribution.png', dpi=130)
    plt.close()
    print('15_wp_skip_distribution.png')


# figure 16: duration vs coverage scatter

def plot_16_duration_vs_coverage(metrics):
    fig, ax = plt.subplots(figsize=(10, 6.5))
    for key, label, color in METHODS:
        xs, ys, names = [], [], []
        for r in ROUTES:
            m = metrics.get(r, {}).get(key, {})
            if not m: continue
            xs.append(m.get('duration_s', 0) or 0)
            ys.append(m.get('cov_pct', 0) or 0)
            names.append(ROUTE_LABELS[r])
        ax.scatter(xs, ys, c=color, s=120, label=label, alpha=0.8, edgecolors='white', linewidths=1)
    ax.set_xlabel('run duration (s)')
    ax.set_ylabel('WP coverage (%)')
    ax.set_title('run duration vs WP coverage')
    ax.legend()
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '16_duration_vs_coverage.png', dpi=130)
    plt.close()
    print('16_duration_vs_coverage.png')


# figure 17: VIO vs anchor vs GT overlay on trajectory

def plot_17_anchor_vs_vio():
    # exp 74 has plot_scratch/ with our_vio.csv and our_anchor.csv
    base = ROOT / 'experiments' / '74_pure_stock_nav2_baseline' / 'results' / 'plot_scratch'
    vio_csv = base / 'our_vio.csv'
    anc_csv = base / 'our_anchor.csv'
    stk_csv = base / 'stock_nav_fused.csv'
    if not (vio_csv.exists() and anc_csv.exists()):
        print('17 skipped - plot_scratch missing')
        return
    def read_xy(p):
        arr = []
        with open(p) as f:
            rd = csv.reader(f)
            for row in rd:
                try:
                    arr.append((float(row[1]), float(row[2])))
                except (ValueError, IndexError):
                    continue
        return np.array(arr)
    vio = read_xy(vio_csv)
    anc = read_xy(anc_csv)
    stk = read_xy(stk_csv) if stk_csv.exists() else None

    fig, ax = plt.subplots(figsize=(10, 7))
    if vio.size: ax.plot(vio[:, 0], vio[:, 1], color='#ff7f0e', lw=1.3, label='pure VIO (drifts)')
    if anc.size: ax.plot(anc[:, 0], anc[:, 1], color=COL_OURS, lw=1.8, label='ours: VIO + anchor')
    if stk is not None and stk.size:
        ax.plot(stk[:, 0], stk[:, 1], color=COL_STOCK, lw=1.4, label='stock Nav2 fused', alpha=0.75)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('VIO alone vs anchor-corrected pose (route 09)')
    ax.legend(loc='upper left')
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '17_anchor_vs_vio_path.png', dpi=130)
    plt.close()
    print('17_anchor_vs_vio_path.png')


# figure 18: scene + all 9 routes overlay

def plot_18_scene_routes():
    scene = json.load(open(ROOT / 'routes' / '_common' / 'scene_obstacles.json'))
    routes_xy = json.load(open(ROOT / 'routes' / '_common' / 'routes.json'))

    fig, ax = plt.subplots(figsize=(14, 9))
    # scene elements
    for o in scene:
        t = o.get('type', '?')
        r = o.get('r', 0.5)
        x, y = o['x'], o['y']
        if t == 'tree':
            ax.add_patch(patches.Circle((x, y), r, color='#2d6a4f', alpha=0.5))
        elif t == 'roadside_tree':
            ax.add_patch(patches.Circle((x, y), r, color='#40916c', alpha=0.6))
        elif t == 'shrub':
            ax.add_patch(patches.Circle((x, y), r * 0.7, color='#95d5b2', alpha=0.3))
        elif t == 'house':
            ax.add_patch(patches.Rectangle((x - r, y - r), r*2, r*2, color='#6c757d', alpha=0.7))
        elif t == 'rock':
            ax.add_patch(patches.Circle((x, y), r, color='#adb5bd', alpha=0.7))
        elif t == 'barrel':
            ax.add_patch(patches.Circle((x, y), 0.35, color='#8ac926', alpha=0.8))

    cmap = plt.get_cmap('tab10')
    for i, r in enumerate(ROUTES):
        wps = routes_xy.get(r, [])
        if not wps: continue
        wps = np.array(wps)
        ax.plot(wps[:, 0], wps[:, 1], color=cmap(i), lw=2, alpha=0.85,
                label=ROUTE_LABELS[r])
        ax.scatter(wps[0, 0], wps[0, 1], color=cmap(i), s=80, marker='s', edgecolors='black', zorder=5)

    ax.set_aspect('equal')
    ax.set_xlim(-115, 85)
    ax.set_ylim(-50, 50)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('simulation scene 220x150 m with all 9 T&R routes')
    ax.legend(loc='upper left', ncol=3, fontsize=8)
    ax.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(OUT / '18_scene_obstacles_routes.png', dpi=130)
    plt.close()
    print('18_scene_obstacles_routes.png')


# figure 19: PnP inlier histogram

def plot_19_pnp_hist():
    anchor_path = ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'anchor_matches.csv'
    if not anchor_path.exists():
        print('19 skipped - no anchor log')
        return
    accepted, rejected = [], []
    with open(anchor_path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                n = int(row.get('best_n_inliers') or 0)
            except ValueError:
                continue
            oc = row.get('outcome', '')
            if 'published' in oc:
                accepted.append(n)
            else:
                rejected.append(n)

    fig, ax = plt.subplots(figsize=(9, 5))
    bins = np.arange(0, 160, 8)
    ax.hist(rejected, bins=bins, color='#c62828', alpha=0.55, label=f'rejected (n={len(rejected)})')
    ax.hist(accepted, bins=bins, color=COL_OURS, alpha=0.75, label=f'accepted (n={len(accepted)})')
    ax.axvline(10, color='black', ls='--', lw=1, alpha=0.6)
    ax.text(10.5, ax.get_ylim()[1] * 0.9, 'MIN_INLIERS = 10', fontsize=9)
    ax.set_xlabel('PnP inlier count')
    ax.set_ylabel('frequency')
    ax.set_title('PnP inlier distribution - route 09 RGB-D-no-IMU run')
    ax.legend()
    ax.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(OUT / '19_pnp_inlier_hist.png', dpi=130)
    plt.close()
    print('19_pnp_inlier_hist.png')


# figure 20: yaw error over time (from tf_slam.log yaw_err)

def plot_20_yaw_drift():
    runs = {
        'ours':   ROOT / 'experiments' / '52_obstacles_v9' / 'teach' / 'tf_slam.log',
        'stock':  ROOT / 'experiments' / '74_pure_stock_nav2_baseline' / 'results' / 'run_09' / 'tf_slam.log',
        'RGB-D':  ROOT / 'experiments' / '76_rgbd_no_imu_ours' / 'results' / 'run_09' / 'tf_slam.log',
    }
    colors = {'ours': COL_OURS, 'stock': COL_STOCK, 'RGB-D': COL_RGBD}
    pat = re.compile(r'\[(\d+\.\d+)\].*yaw_err=(-?\d+\.\d+)')
    fig, ax = plt.subplots(figsize=(13, 5))
    for name, path in runs.items():
        if not path.exists(): continue
        ts, ys = [], []
        t0 = None
        with open(path) as f:
            for line in f:
                m = pat.search(line)
                if not m: continue
                t = float(m.group(1))
                y = float(m.group(2))
                if t0 is None: t0 = t
                ts.append(t - t0)
                ys.append(y)
        if ts:
            ax.plot(ts, ys, lw=1.3, color=colors[name], label=f'{name} (n={len(ts)})', alpha=0.85)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('yaw error vs GT (deg)')
    ax.set_title('heading drift over time - route 09 SE->NE')
    ax.legend()
    ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '20_yaw_drift_timeline.png', dpi=130)
    plt.close()
    print('20_yaw_drift_timeline.png')


# figure 21: path efficiency ratio

def plot_21_path_efficiency(metrics):
    # ratio of actual path_m over route's GT-teach path length (we'll use our-custom teach_gt_length as baseline)
    # from metrics, 'ours teach' path as denominator if available; else use 'our custom' path as baseline per route
    baselines = {r: (metrics.get(r, {}).get('our custom', {}).get('path_m', 1.0) or 1.0) for r in ROUTES}
    x = np.arange(len(ROUTES))
    width = 0.27
    fig, ax = plt.subplots(figsize=(13, 5))
    for i, (key, label, color) in enumerate(METHODS):
        vals = []
        for r in ROUTES:
            p = (metrics.get(r, {}).get(key, {}).get('path_m') or 0) / (baselines[r] or 1.0)
            vals.append(p)
        ax.bar(x + (i-1)*width, vals, width, color=color, label=label, edgecolor='white')
    ax.axhline(1.0, color='black', ls='--', lw=1, alpha=0.6, label='baseline (ours = 1.0)')
    ax.set_xticks(x)
    ax.set_xticklabels([ROUTE_LABELS[r] for r in ROUTES])
    ax.set_ylabel('path length / ours-custom teach path length')
    ax.set_title('path efficiency: how much did each method wander vs our reference path')
    ax.legend(loc='upper right')
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT / '21_path_efficiency.png', dpi=130)
    plt.close()
    print('21_path_efficiency.png')


# run everything

def main():
    metrics = load_metrics()
    print(f'output dir: {OUT}\n')
    plot_07_headline(metrics)
    plot_08_drift(metrics)
    plot_09_endpoint(metrics)
    plot_10_drift_timeline()
    plot_11_anchors_on_traj()
    plot_12_obstacle_detour()
    plot_13_heatmap(metrics)
    plot_14_imu_ablation()
    plot_15_wp_skips(metrics)
    plot_16_duration_vs_coverage(metrics)
    plot_17_anchor_vs_vio()
    plot_18_scene_routes()
    plot_19_pnp_hist()
    plot_20_yaw_drift()
    plot_21_path_efficiency(metrics)
    print('\nall done')


if __name__ == '__main__':
    main()
