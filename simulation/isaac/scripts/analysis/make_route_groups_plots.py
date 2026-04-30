#!/usr/bin/env python3
"""one trajectory plot per route group

groups overlap (a route can appear in several plots): G1 forest is the
9 routes with corridor density >= 7 features/100m, G2 open is 6 routes
below that, G3 short is 8 routes <= 205 m, G4 long is 7 routes > 300 m,
G5 cones+tent is the 4 campaign-style runs, G6 mixed props is the
11 routes with barrels/dumpsters/cardboxes/etc.  each PNG uses the
same scene-background template as the rest of the thesis figures but
overlays *every* route in the group with a distinct colour
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts/analysis')
from plot_trajectory_map import plot_trajectory_map


DSET = Path('/root/isaac_tr_datasets')
ARC = Path('/root/isaac_archive/experiments')

OUT_DIR = Path('/workspace/simulation/isaac/results/final/dev_history/route_groups')
OUT_DIR.mkdir(parents=True, exist_ok=True)


# (route_id, csv path, x_col, y_col)
ROUTE_TRAJ = {
    '01_road':         (DSET / '01_road/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '02_north_forest': (DSET / '02_north_forest/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '03_south':        (DSET / '03_south/teach/isaac_slam_1776579789/groundtruth.csv', 'x', 'y'),
    '04_nw_se':        (DSET / '04_nw_se/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '05_ne_sw':        (DSET / '05_ne_sw/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '06_nw_ne':        (DSET / '06_nw_ne/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '07_se_sw':        (DSET / '07_se_sw/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '08_nw_sw':        (DSET / '08_nw_sw/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '09_se_ne':        (DSET / '09_se_ne/teach/teach_outputs/traj_gt_world.csv', 'x', 'y'),
    '10_nmid_smid':    (DSET / '10_nmid_smid/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
    '11_nw_mid':       (DSET / '11_nw_mid/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
    '12_ne_mid':       (DSET / '12_ne_mid/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
    '13_cross_nws':    (DSET / '13_cross_nws/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
    '14_se_mid':       (DSET / '14_se_mid/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
    '15_wmid_smid':    (DSET / '15_wmid_smid/teach/teach_outputs/traj_gt.csv', 'x', 'y'),
}

# distinct colours per route (15 distinguishable hues)
ROUTE_COLOR = {
    '01_road':         '#dc2626',
    '02_north_forest': '#ea580c',
    '03_south':        '#ca8a04',
    '04_nw_se':        '#65a30d',
    '05_ne_sw':        '#15803d',
    '06_nw_ne':        '#0891b2',
    '07_se_sw':        '#1d4ed8',
    '08_nw_sw':        '#7c3aed',
    '09_se_ne':        '#be185d',
    '10_nmid_smid':    '#0f172a',
    '11_nw_mid':       '#a16207',
    '12_ne_mid':       '#16a34a',
    '13_cross_nws':    '#0e7490',
    '14_se_mid':       '#9333ea',
    '15_wmid_smid':    '#b91c1c',
}

GROUPS = [
    ('G1_forest',
     'group 1, routes through dense forest (10 routes)',
     ['02_north_forest', '03_south', '04_nw_se', '05_ne_sw', '06_nw_ne',
      '07_se_sw', '08_nw_sw', '11_nw_mid', '13_cross_nws', '15_wmid_smid']),
    ('G2_open',
     'group 2, routes through open / minimal forest (5 routes)',
     ['01_road', '09_se_ne', '10_nmid_smid', '12_ne_mid', '14_se_mid']),
    ('G3_short',
     'group 3, short routes (8 routes, length 160-205 m)',
     ['08_nw_sw', '09_se_ne', '10_nmid_smid', '11_nw_mid', '12_ne_mid',
      '13_cross_nws', '14_se_mid', '15_wmid_smid']),
    ('G4_long',
     'group 4, long routes (7 routes, length 333-407 m)',
     ['01_road', '02_north_forest', '03_south', '04_nw_se', '05_ne_sw',
      '06_nw_ne', '07_se_sw']),
    ('G5_cones_tent',
     'group 5, cone walls + tent obstacles (4 routes)',
     ['01_road', '02_north_forest', '03_south', '04_nw_se']),
    ('G6_mixed_props',
     'group 6, mixed prop obstacles, barrels / dumpsters / cardboxes / etc. (11 routes)',
     ['05_ne_sw', '06_nw_ne', '07_se_sw', '08_nw_sw', '09_se_ne',
      '10_nmid_smid', '11_nw_mid', '12_ne_mid', '13_cross_nws',
      '14_se_mid', '15_wmid_smid']),
]


def render_group(group_id, title, route_ids):
    trajs = []
    missing = []
    for rid in route_ids:
        path, xc, yc = ROUTE_TRAJ[rid]
        if not path.is_file():
            missing.append(rid); continue
        trajs.append({
            'csv': str(path),
            'label': f'{rid}',
            'color': ROUTE_COLOR[rid],
            'linewidth': 1.5,
            'x_col': xc, 'y_col': yc,
        })
    out = OUT_DIR / f'{group_id}.png'
    plot_trajectory_map(
        trajectories=trajs,
        output=str(out),
        title=title,
        metrics_lines=[
            f'group {group_id.split("_")[0][1:]}: {len(trajs)} routes shown',
            f'each colour is one route, traced from teach GT',
        ] + ([f'missing data: {", ".join(missing)}'] if missing else []),
        with_obstacles=False, with_waypoints=False,
        with_reference_path=False,
        route='south',
        xlim=(-115, 90), ylim=(-50, 45),
        figsize=(15, 9.0),
    )
    print(f'  wrote {out}  ({len(trajs)} routes)')


def main():
    for gid, title, routes in GROUPS:
        render_group(gid, title, routes)


if __name__ == '__main__':
    main()
