#!/usr/bin/env python3
"""Preview obstacle layout on south route."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/50_vio_obstacles_removed/logs/route_backdrop.csv',
            'label': 'Pre-planned route (outbound + return)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/50_vio_obstacles_removed/results/exp50_obstacle_layout.png',
    title='Exp 50: Obstacle Layout Preview (new positions)',
    metrics_lines=[
        'Group 1: x=-75, y=-23..-27  (deep forest, on route)',
        'Group 2: x=-18, y=-22..-26  (transition zone, on route)',
        'Group 3: x=+5,  y=-16..-20  (after first building, on route)',
        'Tent:    (-45, -38)         (deep forest, east of original)',
    ],
    route='south',
    with_obstacles=True,
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
