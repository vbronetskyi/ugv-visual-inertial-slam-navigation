#!/usr/bin/env python3
"""Exp 41: GT localization trajectory plot."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {'csv': '/workspace/simulation/isaac/experiments/41_trajectory_follow/logs/exp41_roundtrip_outbound_traj.csv',
         'label': 'GT localization (outbound) - 41/43 (95%)',
         'color': '#2ca02c', 'x_col': 'x', 'y_col': 'y'},
    ],
    output='/workspace/simulation/isaac/experiments/41_trajectory_follow/results/exp41_trajectory_v2.png',
    title='Exp 41: GT Localization - Teach-and-Repeat with Obstacles',
    metrics_lines=[
        "Method:        GT (ground truth)",
        "Result:        41/43 waypoints (95%)",
        "Skipped:       2 (inside Barrier 1 + tent)",
        "Duration:      491s",
        "Path length:   194.5m",
        "CTE mean:      1.68m",
        "Loc err:       0.00m (perfect by def.)",
        "Role:          Upper-bound baseline",
    ],
    with_obstacles=True,
    with_waypoints=True,
)
