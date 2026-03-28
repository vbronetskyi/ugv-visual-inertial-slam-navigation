#!/usr/bin/env python3
"""Exp 42: Encoder+Compass localization trajectory plots (with and without obstacles)."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

# No obstacles
plot_trajectory_map(
    trajectories=[
        {'csv': '/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_no_obs_traj.csv',
         'label': 'Encoder+Compass (no obstacles) - 43/43 (100%)',
         'color': '#1f77b4', 'x_col': 'x', 'y_col': 'y'},
    ],
    output='/workspace/simulation/isaac/experiments/42_real_localization/results/exp42_trajectory_no_obs_v2.png',
    title='Exp 42: Encoder+Compass - No Obstacles',
    metrics_lines=[
        "Method:       Encoder + Compass (realistic)",
        "Sensors:      GT diff + 0.5% noise (encoders)",
        "              GT yaw + 3° noise (compass)",
        "Result:       43/43 waypoints (100%)",
        "Skipped:      0",
        "Duration:     396s",
        "Pos drift:    max 0.8m / mean ~0.4m",
        "Heading err:  max 3.5° / mean ~1.5°",
    ],
    with_obstacles=False,
    with_waypoints=True,
)

# With obstacles
plot_trajectory_map(
    trajectories=[
        {'csv': '/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_obs_traj.csv',
         'label': 'Encoder+Compass (with obstacles) - 41/43 (95%)',
         'color': '#1f77b4', 'x_col': 'x', 'y_col': 'y'},
    ],
    output='/workspace/simulation/isaac/experiments/42_real_localization/results/exp42_trajectory_obs_v2.png',
    title='Exp 42: Encoder+Compass - With Obstacles',
    metrics_lines=[
        "Method:      Encoder + Compass (realistic)",
        "Result:      41/43 waypoints (95%)",
        "Skipped:     2 (WP 11 barrier, WP 18 tent)",
        "Duration:    461s",
        "Path length: 183.5m",
        "CTE mean:    1.60m",
        "Loc err:     mean 0.90m, max 1.13m",
    ],
    with_obstacles=True,
    with_waypoints=True,
)
