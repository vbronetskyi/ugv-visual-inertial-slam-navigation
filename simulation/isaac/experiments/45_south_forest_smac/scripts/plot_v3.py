#!/usr/bin/env python3
"""Plot exp 45 v3 - full-route Nav2 success."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/45_south_forest_smac/logs/exp45_gtfixed_traj.csv',
            'label': 'GT baseline (fixed route)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/45_south_forest_smac/logs/exp45_v3_traj.csv',
            'label': 'Nav2 v3 (SmacPlanner + smoother + fixed WPs + GT tf)',
            'color': '#d62728',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/45_south_forest_smac/results/exp45_v3_trajectory.png',
    title='Exp 45 v3: Nav2 full-route success (72/98 REACHED, 99% distance)',
    metrics_lines=[
        'Method:     Nav2 SmacPlanner2D + velocity_smoother',
        'Route:      South forest fixed (1.2m clearance)',
        'WPs:        99 anchors @ 1.5m spacing',
        'Localiz:    GT (tf_wall_clock_relay --use-gt)',
        'Reached:    72/98 (73% hard, 99% dist coverage)',
        'Final:      (68.6, -4.6), ~0.5m from goal (69.6, -4.2)',
        'Stuck zone: WPs 51-58 (halo) bypassed via timeout+skip',
    ],
    route='south',
    with_obstacles=False,
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/45_south_forest_smac/config/south_anchors_fixed.json',
)
