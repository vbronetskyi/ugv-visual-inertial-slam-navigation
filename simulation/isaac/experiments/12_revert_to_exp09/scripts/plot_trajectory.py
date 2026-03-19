#!/usr/bin/env python3
"""
General trajectory plotting script for navigation experiments.

Plots robot GT trajectory on the road map with waypoints, obstacles,
and key metrics (distance, time, drift).

usage:
  python3 plot_trajectory.py <trajectory.csv> [--title "Experiment Name"] [--output plot.png]
  python3 plot_trajectory.py <trajectory.csv> --slam-log <slam_log.txt>
  python3 plot_trajectory.py traj1.csv traj2.csv --labels "GT,SLAM" --title "Comparison"
"""
import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# road reference
ROAD_WPS = [
    (-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
    (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
    (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
    (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
    (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5),
]
ROAD_X = [p[0] for p in ROAD_WPS]
ROAD_Y = [p[1] for p in ROAD_WPS]

# obstacles
CONE_GROUPS = [
    [(-52, -5.5), (-50, -4.5), (-48, -5.0)],  # group 1
    [(13, -2.5), (15, -1.5), (17, -2.0)],       # group 2
    [(43, -1.0), (45, -0.5), (47, -1.0)],       # group 3
]
TENT_POS = (-20, 0.2)

SPAWN = (-95, -6)
DEST = (72, -5)

# colors for multiple trajectories
TRAJ_COLORS = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0', '#FF9800', '#00BCD4']
ROAD_COLOR = '#888888'


def load_trajectory(csv_path):
    """Load trajectory CSV. Returns dict with arrays."""
    times, phases = [], []
    gt_x, gt_y, gt_z, gt_yaw = [], [], [], []
    cmd_lin, cmd_ang = [], []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row['time']))
                phases.append(row['phase'])
                gt_x.append(float(row['gt_x']))
                gt_y.append(float(row['gt_y']))
                gt_z.append(float(row.get('gt_z', 0)))
                gt_yaw.append(float(row.get('gt_yaw', 0)))
                cmd_lin.append(float(row.get('cmd_lin', 0)))
                cmd_ang.append(float(row.get('cmd_ang', 0)))
            except (ValueError, KeyError):
                continue

    return {
        'time': np.array(times),
        'phase': phases,
        'x': np.array(gt_x),
        'y': np.array(gt_y),
        'z': np.array(gt_z),
        'yaw': np.array(gt_yaw),
        'cmd_lin': np.array(cmd_lin),
        'cmd_ang': np.array(cmd_ang),
    }


def compute_metrics(traj):
    """Compute key metrics from trajectory."""
    x, y, t = traj['x'], traj['y'], traj['time']
    if len(x) < 2:
        return {}

    # total distance traveled
    dx = np.diff(x)
    dy = np.diff(y)
    dists = np.sqrt(dx**2 + dy**2)
    total_dist = np.sum(dists)

    # max distance from start
    dist_from_start = np.sqrt((x - SPAWN[0])**2 + (y - SPAWN[1])**2)
    max_from_start = np.max(dist_from_start)

    # max X reached (forward progress)
    max_x = np.max(x)

    # closest to destination
    dist_to_dest = np.sqrt((x - DEST[0])**2 + (y - DEST[1])**2)
    min_to_dest = np.min(dist_to_dest)

    # duration
    duration = t[-1] - t[0]

    # average speed (when moving)
    moving = traj['cmd_lin'] > 0.05
    avg_speed = np.mean(traj['cmd_lin'][moving]) if np.any(moving) else 0

    # lateral drift from road center
    road_y_interp = np.interp(x, ROAD_X, ROAD_Y)
    lateral_err = y - road_y_interp
    max_lateral = np.max(np.abs(lateral_err))
    mean_lateral = np.mean(np.abs(lateral_err))

    # phase split
    outbound_mask = np.array([p == 'outbound' for p in traj['phase']])
    return_mask = np.array([p == 'return' for p in traj['phase']])

    return {
        'total_dist': total_dist,
        'max_from_start': max_from_start,
        'max_x': max_x,
        'min_to_dest': min_to_dest,
        'duration': duration,
        'avg_speed': avg_speed,
        'max_lateral': max_lateral,
        'mean_lateral': mean_lateral,
        'outbound_count': np.sum(outbound_mask),
        'return_count': np.sum(return_mask),
    }


def plot_trajectories(trajs, labels, title, output_path, show_obstacles=True):
    """Plot one or more trajectories on the road map."""
    fig, axes = plt.subplots(3, 1, figsize=(16, 13),
                             gridspec_kw={'height_ratios': [3, 1.2, 1]})
    fig.suptitle(title, fontsize=14, fontweight='bold', y=0.98)

    ax_map = axes[0]
    ax_err = axes[1]
    ax_metrics = axes[2]

    # --- TOP: map view ---
    # road reference
    ax_map.plot(ROAD_X, ROAD_Y, color=ROAD_COLOR, linewidth=8, alpha=0.3, zorder=1, label='road center')
    ax_map.plot(ROAD_X, ROAD_Y, color=ROAD_COLOR, linewidth=1, linestyle='--', alpha=0.5, zorder=2)

    # obstacles
    if show_obstacles:
        for i, group in enumerate(CONE_GROUPS):
            for j, (cx, cy) in enumerate(group):
                cone = plt.Circle((cx, cy), 0.4, color='#FF9800', alpha=0.8, zorder=5)
                ax_map.add_patch(cone)
                if i == 0 and j == 0:
                    ax_map.plot([], [], 'o', color='#FF9800', markersize=8, label='cones')
        # tent
        tent = mpatches.FancyBboxPatch(
            (TENT_POS[0]-1.5, TENT_POS[1]-1), 3, 2,
            boxstyle="round,pad=0.3", facecolor='#795548', alpha=0.7, zorder=5)
        ax_map.add_patch(tent)
        ax_map.text(TENT_POS[0], TENT_POS[1], 'tent', ha='center', va='center',
                   fontsize=7, color='white', fontweight='bold', zorder=6)

    # start / destination markers
    ax_map.plot(*SPAWN, 's', color='#4CAF50', markersize=12, zorder=10, label='start')
    ax_map.plot(*DEST, '*', color='#F44336', markersize=14, zorder=10, label='destination')

    # trajectories
    metrics_list = []
    for i, (traj, label) in enumerate(zip(trajs, labels)):
        color = TRAJ_COLORS[i % len(TRAJ_COLORS)]
        x, y = traj['x'], traj['y']

        # split by phase
        outbound_mask = np.array([p == 'outbound' for p in traj['phase']])
        return_mask = np.array([p == 'return' for p in traj['phase']])

        if np.any(outbound_mask):
            ax_map.plot(x[outbound_mask], y[outbound_mask], color=color,
                       linewidth=2, alpha=0.9, zorder=7, label=f'{label} (outbound)')
        if np.any(return_mask):
            ax_map.plot(x[return_mask], y[return_mask], color=color,
                       linewidth=2, alpha=0.5, linestyle='--', zorder=7,
                       label=f'{label} (return)')

        # max reach marker (furthest X in outbound) + endpoint
        if np.any(outbound_mask):
            ox = x[outbound_mask]
            oy = y[outbound_mask]
            max_idx = np.argmax(ox)
            ax_map.plot(ox[max_idx], oy[max_idx], 'D', color=color, markersize=9, zorder=9,
                       markeredgecolor='white', markeredgewidth=1.5)
            ax_map.annotate(f'max reach\n({ox[max_idx]:.0f},{oy[max_idx]:.0f})',
                           (ox[max_idx], oy[max_idx]), textcoords="offset points",
                           xytext=(0, 12), ha='center', fontsize=7, color=color,
                           fontweight='bold')
        ax_map.plot(x[-1], y[-1], 'o', color=color, markersize=6, zorder=8,
                   markeredgecolor='white', markeredgewidth=1, alpha=0.6)

        m = compute_metrics(traj)
        metrics_list.append(m)

    ax_map.set_xlabel('X (m)', fontsize=10)
    ax_map.set_ylabel('Y (m)', fontsize=10)
    ax_map.set_aspect('equal')
    ax_map.legend(loc='upper left', fontsize=8, framealpha=0.9)
    ax_map.grid(True, alpha=0.3)
    ax_map.set_xlim(-105, 85)
    ax_map.set_ylim(-15, 15)

    # --- MIDDLE: lateral error vs X ---
    for i, (traj, label) in enumerate(zip(trajs, labels)):
        color = TRAJ_COLORS[i % len(TRAJ_COLORS)]
        x, y = traj['x'], traj['y']

        # lateral error = distance from road centerline (signed: + = north, - = south)
        road_y_at_x = np.interp(x, ROAD_X, ROAD_Y)
        lat_err = y - road_y_at_x

        # plot vs X position (more intuitive than time - shows where errors happen)
        outbound = np.array([p == 'outbound' for p in traj['phase']])
        ret = np.array([p == 'return' for p in traj['phase']])

        if np.any(outbound):
            ax_err.plot(x[outbound], lat_err[outbound], color=color,
                       linewidth=1.5, alpha=0.8, label=f'{label}')
        if np.any(ret):
            ax_err.plot(x[ret], lat_err[ret], color=color,
                       linewidth=1, alpha=0.4, linestyle='--')

        # annotate max error
        abs_err = np.abs(lat_err)
        max_idx = np.argmax(abs_err)
        ax_err.annotate(f'{lat_err[max_idx]:+.1f}m',
                       (x[max_idx], lat_err[max_idx]),
                       textcoords="offset points", xytext=(5, 5),
                       fontsize=7, color=color, fontweight='bold')

    ax_err.axhline(0, color=ROAD_COLOR, linewidth=1, linestyle='-', alpha=0.5)
    ax_err.axhline(2, color='#FF9800', linewidth=0.8, linestyle=':', alpha=0.5)
    ax_err.axhline(-2, color='#FF9800', linewidth=0.8, linestyle=':', alpha=0.5)
    ax_err.fill_between([-105, 85], -2, 2, color='#4CAF50', alpha=0.05)
    ax_err.set_xlabel('X position (m)', fontsize=10)
    ax_err.set_ylabel('Lateral error (m)', fontsize=10)
    ax_err.set_title('Lateral drift from road center (+ = north, - = south)',
                    fontsize=10, style='italic')
    ax_err.set_xlim(-105, 85)
    ax_err.set_ylim(-12, 12)
    ax_err.legend(loc='upper left', fontsize=8, framealpha=0.9)
    ax_err.grid(True, alpha=0.3)

    # --- BOTTOM: metrics table ---
    ax_metrics.axis('off')
    if metrics_list:
        col_labels = ['Metric'] + labels
        table_data = []
        keys = [
            ('total_dist', 'Distance traveled', '{:.0f}m'),
            ('max_x', 'Max X reached', '{:.0f}m'),
            ('duration', 'Duration', '{:.0f}s'),
            ('avg_speed', 'Avg speed (moving)', '{:.2f} m/s'),
            ('max_lateral', 'Max lateral drift', '{:.1f}m'),
            ('mean_lateral', 'Mean lateral drift', '{:.1f}m'),
        ]
        for key, name, fmt in keys:
            row = [name]
            for m in metrics_list:
                if key in m:
                    row.append(fmt.format(m[key]))
                else:
                    row.append('-')
            table_data.append(row)

        # route completion
        row = ['Route completion']
        for m in metrics_list:
            if 'max_x' in m:
                route_len = DEST[0] - SPAWN[0]  # 167m
                progress = (m['max_x'] - SPAWN[0])
                pct = progress / route_len * 100
                row.append(f'{progress:.0f}m ({pct:.0f}%)')
            else:
                row.append('-')
        table_data.append(row)

        table = ax_metrics.table(
            cellText=table_data, colLabels=col_labels,
            loc='center', cellLoc='center',
            colColours=['#E3F2FD'] + ['#FFF3E0'] * len(labels))
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.0, 1.4)

        # style header
        for j in range(len(col_labels)):
            table[0, j].set_fontsize(10)
            table[0, j].set_text_props(fontweight='bold')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f'saved: {output_path}')
    return metrics_list


def main():
    parser = argparse.ArgumentParser(description='Plot navigation trajectory')
    parser.add_argument('csvs', nargs='+', help='trajectory CSV file(s)')
    parser.add_argument('--labels', type=str, default=None,
                       help='comma-separated labels for each CSV')
    parser.add_argument('--title', type=str, default='Navigation Trajectory',
                       help='plot title')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='output PNG path (default: same dir as first CSV)')
    parser.add_argument('--no-obstacles', action='store_true',
                       help='hide obstacle markers')
    args = parser.parse_args()

    # load trajectories
    trajs = []
    for csv_path in args.csvs:
        if not os.path.exists(csv_path):
            print(f'error: {csv_path} not found')
            sys.exit(1)
        traj = load_trajectory(csv_path)
        if len(traj['x']) == 0:
            print(f'warning: {csv_path} has no data, skipping')
            continue
        trajs.append(traj)

    if not trajs:
        print('error: no valid trajectories')
        sys.exit(1)

    # labels
    if args.labels:
        labels = args.labels.split(',')
    else:
        labels = [os.path.splitext(os.path.basename(c))[0] for c in args.csvs]
    labels = labels[:len(trajs)]

    # output path
    if args.output:
        output = args.output
    else:
        base = os.path.splitext(args.csvs[0])[0]
        output = base + '_plot.png'

    plot_trajectories(trajs, labels, args.title, output, show_obstacles=not args.no_obstacles)


if __name__ == '__main__':
    main()
