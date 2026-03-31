#!/usr/bin/env python3
"""Plot exp 50 (live, current progress) - shows obstacles + GT so far."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/50_vio_obstacles_removed/logs/exp50_gt_live.csv',
            'label': 'GT trajectory (in progress)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/50_vio_obstacles_removed/results/exp50_live.png',
    title='Exp 50: Dynamic Obstacles + Nav2+VIO (LIVE - route in progress)',
    metrics_lines=[
        'Obstacles:  3 cone groups (x=-75, -18, 35) + tent @ (-50, -37)',
        'Strategy:   Nav2 plans bypass using depth camera',
        'Turnaround: supervisor removes obstacles at x>65',
        'Return:     obstacle-free path back to spawn',
    ],
    route='south',
    with_obstacles=True,
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
