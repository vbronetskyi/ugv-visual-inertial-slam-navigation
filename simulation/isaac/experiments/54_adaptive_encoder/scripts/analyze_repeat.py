#!/usr/bin/env python3
"""Analyze repeat run 4: extract traj CSV from tf_slam.log, plot with canonical script."""
import os, re, sys, csv
import numpy as np

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

REPEAT_DIR = '/workspace/simulation/isaac/experiments/53_proactive_reroute/results/repeat_run2'
TRAJ_CSV = os.path.join(REPEAT_DIR, 'trajectory.csv')
OUT_PNG  = os.path.join(REPEAT_DIR, 'trajectory_map.png')

LINE_RE = re.compile(
    r'\[(\d+\.\d+)\].*\[SLAM\]: nav=\(([\-\d\.]+),([\-\d\.]+)\) '
    r'gt=\(([\-\d\.]+),([\-\d\.]+)\) err=([\d\.]+)m enc_err=([\d\.]+)m '
    r'yaw_err=([\d\.]+). dist=(\d+)m slam_f=(\d+) lost=(\d+)'
)


def main():
    rows = []
    with open(os.path.join(REPEAT_DIR, 'tf_slam.log')) as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            ts, nx, ny, gx, gy, err, enc, yaw, dist, sf, lost = m.groups()
            rows.append({
                'ts': float(ts), 'nav_x': float(nx), 'nav_y': float(ny),
                'gt_x': float(gx),  'gt_y': float(gy),
                'err': float(err), 'enc_err': float(enc),
                'yaw_err': float(yaw), 'dist_m': int(dist),
                'slam_f': int(sf), 'lost': int(lost),
            })
    print(f"parsed {len(rows)} samples")
    with open(TRAJ_CSV, 'w') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader()
        for r in rows: w.writerow(r)

    gx = np.array([r['gt_x'] for r in rows]); gy = np.array([r['gt_y'] for r in rows])
    err = np.array([r['err'] for r in rows])
    gt_dist = float(np.hypot(*np.diff(np.column_stack([gx,gy]), axis=0).T).sum())

    goals_log = os.path.join(REPEAT_DIR, 'goals.log')
    text = open(goals_log).read() if os.path.exists(goals_log) else ''
    reached = text.count('REACHED'); timeouts = text.count('TIMEOUT')
    failed = text.count('plan failed')

    # Find SLAM-break timestamp (err > 20m for first time)
    break_idx = next((i for i, r in enumerate(rows) if r['err'] > 20), None)
    break_info = ''
    if break_idx is not None:
        b = rows[break_idx]
        before = rows[break_idx - 1]
        break_info = (
            f"SLAM break at ts+{(b['ts']-rows[0]['ts']):.0f}s: "
            f"err {before['err']:.1f}m->{b['err']:.1f}m, "
            f"lost frames {before['lost']}->{b['lost']}"
        )

    metrics = [
        f"WPs reached:       {reached}/94",
        f"GT dist travelled: {gt_dist:.1f} m",
        f"SLAM err (healthy) before break: mean {err[:break_idx].mean():.2f} m" if break_idx else f"SLAM err mean: {err.mean():.2f} m",
        f"Frame losses:      {rows[-1]['lost']}",
        f"Plans failed:      {failed}",
        f"Outcome: {break_info}" if break_info else "Outcome: unknown",
    ]
    print('\n'.join('   ' + m for m in metrics))

    plot_trajectory_map(
        trajectories=[
            {'csv': TRAJ_CSV,
             'label': f'Run 4 GT - {reached}/94 WPs, {gt_dist:.0f} m',
             'color': '#1f77b4', 'x_col': 'gt_x', 'y_col': 'gt_y'},
            {'csv': TRAJ_CSV,
             'label': 'SLAM estimate (breaks near tent)',
             'color': '#ff6600', 'x_col': 'nav_x', 'y_col': 'nav_y'},
        ],
        output=OUT_PNG,
        title='Exp 53 - Proactive + proximity, run 2 (PP proximity + proactive WP projection cones) - SLAM lost near tent',
        metrics_lines=metrics,
        with_obstacles=True, with_waypoints=True, route='south',
    )


if __name__ == '__main__':
    main()
