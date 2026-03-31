#!/usr/bin/env python3
"""Plot exp 45 GT-fixed full route trajectory."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[{
        'csv': '/workspace/simulation/isaac/experiments/45_south_forest_smac/logs/exp45_gtfixed_traj.csv',
        'label': 'GT fixed route (full outbound+return)',
        'color': '#1f77b4',
        'x_col': 'gt_x', 'y_col': 'gt_y',
    }],
    output='/workspace/simulation/isaac/experiments/45_south_forest_smac/results/exp45_gtfixed_trajectory.png',
    title='Exp 45: GT baseline with fixed route (>=1.2m clearance)',
    metrics_lines=[
        'Route:       South forest, 200 WPs, ~196m one-way',
        'Method:      Internal pure-pursuit auto-drive',
        'Clearance:   >=1.2m to every collision obstacle',
        'Outcome:     full drive (outbound + return)',
    ],
    route='south',
    with_obstacles=False,
    with_waypoints=False,
    anchors_json='/workspace/simulation/isaac/route_memory/south/anchors.json',
)
