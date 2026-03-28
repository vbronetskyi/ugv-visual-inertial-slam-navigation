#!/usr/bin/env python3
"""Exp 43: SLAM+Encoder trajectory plots + combined comparison."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

# SLAM only, with obstacles
plot_trajectory_map(
    trajectories=[
        {'csv': '/workspace/simulation/isaac/experiments/43_slam_localization/logs/exp43_final_traj.csv',
         'label': 'SLAM+Encoder (with obstacles) - 41/43 (95%)',
         'color': '#d62728', 'x_col': 'gt_x', 'y_col': 'gt_y'},
    ],
    output='/workspace/simulation/isaac/experiments/43_slam_localization/results/exp43_trajectory_obs_v2.png',
    title='Exp 43: SLAM+Encoder - With Obstacles',
    metrics_lines=[
        "Method:       ORB-SLAM3 + Encoder fusion",
        "Fallback:     encoder+compass when SLAM lost",
        "SLAM coverage: 20% of ticks",
        "Result:       41/43 waypoints (95%)",
        "Skipped:      2",
        "Duration:     460s",
        "Path length:  184.6m",
        "CTE mean:     2.64m",
        "Loc err:      mean 1.58m, max 2.34m",
    ],
    with_obstacles=True,
    with_waypoints=True,
)

# Combined comparison of all 3 methods
plot_trajectory_map(
    trajectories=[
        {'csv': '/workspace/simulation/isaac/experiments/41_trajectory_follow/logs/exp41_roundtrip_outbound_traj.csv',
         'label': 'GT (exp 41) - 95% [upper bound]',
         'color': '#2ca02c', 'x_col': 'x', 'y_col': 'y'},
        {'csv': '/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_obs_traj.csv',
         'label': 'Encoder+Compass (exp 42) - 95%',
         'color': '#1f77b4', 'x_col': 'x', 'y_col': 'y'},
        {'csv': '/workspace/simulation/isaac/experiments/43_slam_localization/logs/exp43_final_traj.csv',
         'label': 'SLAM+Encoder (exp 43) - 95%',
         'color': '#d62728', 'x_col': 'gt_x', 'y_col': 'gt_y'},
    ],
    output='/workspace/simulation/isaac/experiments/43_slam_localization/results/exp43_trajectory_comparison_v2.png',
    title='All 3 Localization Methods - With Obstacles',
    metrics_lines=[
        "Method                  Reached   Path     CTE     LocErr",
        "───────────────────────────────────────────────────",
        "GT (ideal upper bound)   41/43    194.5m   1.68m   0.00m",
        "Encoder+Compass          41/43    183.5m   1.60m   0.90m",
        "SLAM+Encoder             41/43    184.6m   2.64m   1.58m",
        "───────────────────────────────────────────────────",
        "Reference (no obstacles): 157.5m  |  Goal tolerance: 3.0m",
    ],
    with_obstacles=True,
    with_waypoints=True,
)
