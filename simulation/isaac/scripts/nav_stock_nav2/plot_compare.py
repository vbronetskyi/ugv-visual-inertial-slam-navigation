#!/usr/bin/env python3
"""Canonical-style comparison plots for exp 74 pure-stock-Nav2 baseline
vs our custom T&R pipeline on route 09_se_ne.

Outputs:
  compare_routes.png              GT trajectories (our custom, pure-stock)
                                  on scene map + props + skipped markers.
  compare_localisation.png        VIO raw + our system's anchor-corrected vs
                                  stock-run's uncorrected VIO - the
                                  'with matcher' vs "without matcher" diff.
"""
import csv, math, sys
from pathlib import Path

sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

ROUTE = '09_se_ne'
OUR = Path('/root/isaac_tr_datasets/09_se_ne/repeat/results/repeat_run')
STOCK = Path('/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline/results/run_09')
OUT_DIR = Path('/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline/results')
SCRATCH = OUT_DIR / 'plot_scratch'
SCRATCH.mkdir(parents=True, exist_ok=True)

XLIM, YLIM = (30, 100), (-55, 60)


def split_anchor_csv(src, dst_vio, dst_anchor):
    rows = list(csv.DictReader(open(src)))
    rows = [r for r in rows if r.get('anchor_x') and r.get('anchor_y')]
    with open(dst_vio, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for r in rows:
            w.writerow([r['vio_x'], r['vio_y']])
    with open(dst_anchor, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for r in rows:
            w.writerow([r['anchor_x'], r['anchor_y']])


def tf_slam_to_csv(src, dst):
    """Parse tf_slam.log 'nav=(X,Y)' points into a CSV. exp 74 has no
    matcher, so anchor_matches.csv is empty; we use tf_slam.log as the
    VIO/encoder fusion track instead."""
    rows = []
    with open(src) as f:
        for line in f:
            m = _NAV_RE.search(line)
            if m:
                rows.append((float(m.group(1)), float(m.group(2))))
    with open(dst, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['x', 'y'])
        for x, y in rows:
            w.writerow([x, y])
    return len(rows)


import re
_NAV_RE = re.compile(r'nav=\(([-\d.]+),([-\d.]+)\)')


def mean_shift(vio_p, anc_p):
    # TODO: write proper unit tests one day
    v = list(csv.DictReader(open(vio_p)))
    a = list(csv.DictReader(open(anc_p)))
    errs = [math.hypot(float(vi['x']) - float(ai['x']),
                       float(vi['y']) - float(ai['y']))
            for vi, ai in zip(v, a)]
    return (sum(errs)/len(errs), max(errs)) if errs else (0.0, 0.0)


def main():
    our_gt = OUR / 'traj_gt.csv'
    stk_gt = STOCK / 'traj_gt.csv'

    # compare_routes.png
    plot_trajectory_map(
        trajectories=[
            {'csv': str(our_gt),  'label': 'Our custom T&R (ours)  36/36 reached, 689 s',
             'color': '#16a34a', 'x_col': 'x', 'y_col': 'y', 'linewidth': 2.4},
            {'csv': str(stk_gt), 'label': 'Pure stock Nav2 (no matcher)  22/35 reached, 1048 s',
             'color': '#dc2626', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.9},
        ],
        output=str(OUT_DIR / 'compare_routes.png'),
        title=f'{ROUTE} - our T&R vs pure stock Nav2 (no anchor corrections)',
        metrics_lines=[
            'our custom       36 / 36 reached   0 skipped   dur 689 s',
            'pure stock Nav2  22 / 35 reached  13 skipped   dur 1048 s',
            '  no visual_landmark_matcher (pure VIO+encoder drift);',
            '  stock Nav2: map_server + planner_server + controller_server (RPP)',
            '  + bt_navigator + behavior_server + waypoint_follower (stop_on_failure=false);',
            '  supervisor_kept so return leg is obstacle-free.',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=ROUTE,
        xlim=XLIM,
        ylim=YLIM,
        figsize=(16, 10),
    )

    # compare_localisation.png
    our_vio = SCRATCH / 'our_vio.csv'
    our_anc = SCRATCH / 'our_anchor.csv'
    split_anchor_csv(OUR / 'anchor_matches.csv', our_vio, our_anc)
    our_mean, our_max = mean_shift(our_vio, our_anc)

    # stock run has no matcher -> no anchor_matches to use; fall back to
    # tf_slam.log fused SLAM+encoder track
    stk_nav = SCRATCH / 'stock_nav_fused.csv'
    n_stk = tf_slam_to_csv(STOCK / 'tf_slam.log', stk_nav)

    plot_trajectory_map(
        trajectories=[
            {'csv': str(OUR / 'traj_gt.csv'),
             'label': 'GT (Isaac)  our run', 'color': '#0f172a',
             'x_col': 'x', 'y_col': 'y', 'linewidth': 1.8},
            {'csv': str(our_vio),
             'label': 'VIO raw  our run',
             'color': '#fbbf24', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.0},
            {'csv': str(our_anc),
             'label': f'Anchor-corrected  our run  '
                      f'(matcher shifts mean {our_mean:.2f} m, max {our_max:.2f} m)',
             'color': '#16a34a', 'x_col': 'x', 'y_col': 'y', 'linewidth': 1.4},
            {'csv': str(STOCK / 'traj_gt.csv'),
             'label': 'GT (Isaac)  stock run',
             'color': '#64748b', 'x_col': 'x', 'y_col': 'y',
             'linewidth': 1.2, 'linestyle': '--'},
            {'csv': str(stk_nav),
             'label': f'stock nav-frame pose (VIO+encoder, NO matcher)  {n_stk} samples',
             'color': '#dc2626', 'x_col': 'x', 'y_col': 'y', 'linewidth': 1.3},
        ],
        output=str(OUT_DIR / 'compare_localisation.png'),
        title=f'{ROUTE} - localisation:  anchor-corrected (ours) vs pure VIO+encoder (stock)',
        metrics_lines=[
            f'our:    VIO -> matcher pulls nav back to GT; shift mean {our_mean:.2f} m',
            f'stock:  no matcher; nav-frame drifts from GT as VIO error accumulates;',
            f'        tf_slam stays in `no_anchor` regime for entire run.',
        ],
        with_obstacles=True,
        with_waypoints=False,
        route=ROUTE,
        xlim=XLIM,
        ylim=YLIM,
        figsize=(16, 10),
    )

    print(f'routes       -> {OUT_DIR}/compare_routes.png')
    print(f'localisation -> {OUT_DIR}/compare_localisation.png')


if __name__ == '__main__':
    main()
