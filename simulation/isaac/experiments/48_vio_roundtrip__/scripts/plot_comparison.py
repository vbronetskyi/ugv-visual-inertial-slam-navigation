#!/usr/bin/env python3
"""Plot exp 48 - GT vs VIO vs RGB-D only comparison."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_gt_traj.csv',
            'label': 'GT (pure pursuit actual path)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv',
            'label': 'VIO (RGB-D-Inertial, ATE 0.49m)',
            'color': '#2ca02c',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_rgbd_traj.csv',
            'label': 'RGB-D only (no IMU, ATE 1.98m)',
            'color': '#d62728',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/48_vio_roundtrip/results/exp48_comparison.png',
    title='Exp 48: VIO vs RGB-D Only SLAM (392m roundtrip)',
    metrics_lines=[
        'Method:           ORB-SLAM3 (RGB-D-Inertial vs RGB-D-only)',
        'Navigation:       Pure pursuit @ 0.76 m/s',
        'Route:            South roundtrip 392m',
        '',
        'VIO (RGB-D+IMU):  ATE 0.49m, scale 1.001, 5588 frames, 0 lost',
        'RGB-D only:       ATE 1.98m, scale 1.022, 5543 frames, 1 reset',
        '',
        'IMU improves VIO accuracy 4x on this roundtrip',
        'Both tracked full 392m route - IMU helps most on turns',
    ],
    route='south',
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
