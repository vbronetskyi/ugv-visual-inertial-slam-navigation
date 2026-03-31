#!/usr/bin/env python3
"""Analyze repeat run: extract traj CSV from tf_slam.log, plot with canonical script."""
import os
import re
import sys
import math
import csv

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

REPEAT_DIR = '/workspace/simulation/isaac/experiments/52_obstacles_v9/results/repeat_run'
TRAJ_CSV = os.path.join(REPEAT_DIR, 'trajectory.csv')
OUT_PNG = os.path.join(REPEAT_DIR, 'trajectory_map.png')

LINE_RE = re.compile(
    r'\[(\d+\.\d+)\].*\[SLAM\]: nav=\(([\-\d\.]+),([\-\d\.]+)\) '
    r'gt=\(([\-\d\.]+),([\-\d\.]+)\) err=([\d\.]+)m enc_err=([\d\.]+)m '
    r'yaw_err=([\d\.]+). dist=(\d+)m slam_f=(\d+) lost=(\d+)'
)


def parse_tf_slam(path):
    rows = []
    with open(path) as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            ts, nx, ny, gx, gy, err, enc, yaw, dist, sf, lost = m.groups()
            rows.append({
                'ts': float(ts),
                'nav_x': float(nx), 'nav_y': float(ny),
                'gt_x': float(gx),  'gt_y': float(gy),
                'err': float(err), 'enc_err': float(enc),
                'yaw_err': float(yaw), 'dist_m': int(dist),
                'slam_f': int(sf), 'lost': int(lost),
            })
    return rows


def main():
    rows = parse_tf_slam(os.path.join(REPEAT_DIR, 'tf_slam.log'))
    print(f"parsed {len(rows)} tf samples")
    with open(TRAJ_CSV, 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"  wrote {TRAJ_CSV}")

    # Stats
    import numpy as np
    gx = np.array([r['gt_x'] for r in rows])
    gy = np.array([r['gt_y'] for r in rows])
    err = np.array([r['err'] for r in rows])
    dists = np.diff(np.column_stack([gx, gy]), axis=0)
    gt_dist = float(np.hypot(dists[:, 0], dists[:, 1]).sum())

    # WP counts
    goals_log = os.path.join(REPEAT_DIR, 'goals.log')
    reached = timeouts = 0
    total_wps = 94
    if os.path.exists(goals_log):
        with open(goals_log) as f:
            text = f.read()
        reached = text.count('REACHED')
        timeouts = text.count('TIMEOUT')
    n_plans = sum(1 for _ in open(os.path.join(REPEAT_DIR, 'plans/plans_summary.csv'))) - 1 \
        if os.path.exists(os.path.join(REPEAT_DIR, 'plans/plans_summary.csv')) else 0
    n_snaps = sum(1 for _ in open(os.path.join(REPEAT_DIR, 'snapshots/snapshots_summary.csv'))) - 1 \
        if os.path.exists(os.path.join(REPEAT_DIR, 'snapshots/snapshots_summary.csv')) else 0

    metrics = [
        f"WPs reached:       {reached}/{total_wps}",
        f"GT dist travelled: {gt_dist:.1f} m",
        f"SLAM pos err mean: {err.mean():.2f} m  max: {err.max():.2f} m",
        f"Frame losses:      {rows[-1]['lost']}",
        f"Nav2 plans:        {n_plans}",
        f"Costmap snaps:     {n_snaps}",
        f"Outcome:           STUCK at cone group 1",
    ]
    for m in metrics:
        print("   " + m)

    plot_trajectory_map(
        trajectories=[{
            'csv': TRAJ_CSV,
            'label': f'Repeat run (GT) - {reached}/{total_wps} WPs, {gt_dist:.0f}m',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        }, {
            'csv': TRAJ_CSV,
            'label': 'SLAM estimate',
            'color': '#ff6600',
            'x_col': 'nav_x', 'y_col': 'nav_y',
        }],
        output=OUT_PNG,
        title='Exp 52 v9 - Repeat Run (teach map + depth obstacle_layer, inflation=1.2m)',
        metrics_lines=metrics,
        with_obstacles=True,
        with_waypoints=True,
        route='south',
    )


if __name__ == '__main__':
    main()
