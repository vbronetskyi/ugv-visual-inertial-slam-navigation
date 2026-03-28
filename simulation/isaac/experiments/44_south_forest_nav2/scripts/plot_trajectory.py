#!/usr/bin/env python3
"""Exp 44: South forest trajectory plot - v1 (sparse WPs) vs v2 (dense WPs)."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/44_south_forest_nav2/logs/exp44_gt_v2_traj.csv',
            'label': 'v1: 4m WPs, inflation 0.5m - stuck (-19.1, -21.6)',
            'color': '#d62728', 'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/44_south_forest_nav2/logs/exp44_v2_traj.csv',
            'label': 'v2: 2m WPs, inflation 0.3m - stuck (-19.4, -19.8)',
            'color': '#1f77b4', 'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/44_south_forest_nav2/results/exp44_trajectory_v2.png',
    title='Exp 44: South Forest - Nav2 v1 vs v2, both blocked by oak cluster at x≈-19',
    metrics_lines=[
        'v1 (sparse):  23/46 (50%), stuck at (-19.1, -21.6)',
        'v2 (dense):   50/99 (51%), stuck at (-19.4, -19.8)',
        'v2 diff:      tighter path, 9 consecutive skips at end',
        '',
        'Both hit same oak+shrub cluster at x=-19',
        'v2 stuck 1.8m north of v1 (cluster center shift)',
        'Exp 30 GT controller passed same x at y=-24',
        '',
        'Root cause: teach path at y=-24 passes 1m from',
        'shrubs; any inflation >0.2m blocks teach corridor,',
        'Nav2 detours north into oak',
    ],
    route='south',
    with_obstacles=False,
    with_waypoints=False,
    anchors_json='/workspace/simulation/isaac/route_memory/south/anchors.json',
)
