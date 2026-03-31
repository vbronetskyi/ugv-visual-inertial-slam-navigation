#!/usr/bin/env python3
"""Plot exp 47 - Nav2 + VIO SLAM localization."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/47_nav2_vio_slam/logs/exp47_gt_traj.csv',
            'label': 'GT trajectory (actual path)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/47_nav2_vio_slam/logs/exp47_vio_traj.csv',
            'label': 'VIO (used by Nav2 for localization)',
            'color': '#d62728',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/results/exp47_nav2_vio_trajectory.png',
    title='Exp 47: Nav2 + VIO SLAM Localization (193m outbound, ATE 0.16m, scale 1.003)',
    metrics_lines=[
        'Method:      Nav2 MPPI + SmacPlanner2D',
        'Localiz:     VIO SLAM (ORB-SLAM3 RGB-D-Inertial live)',
        'Warmup:      30s pure-pursuit before switching to VIO',
        'Camera:      640x480 @ 10 Hz, synthetic IMU 200 Hz',
        'Route:       South outbound 193m (spawn -> buildings)',
        'ATE RMSE:    0.159m (max 0.333m, median 0.137m)',
        'Scale:       1.003, VIO/GT ratio 1.000',
        'WPs:         33/39 reached, 2604 frames, 0 lost',
    ],
    route='south',
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
