#!/usr/bin/env python3
"""Plot exp 48 - VIO live accuracy on full roundtrip."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_gt_traj.csv',
            'label': 'GT trajectory (pure pursuit)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv',
            'label': 'VIO live (aligned)',
            'color': '#d62728',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/48_vio_roundtrip/results/exp48_vio_roundtrip.png',
    title='Exp 48: VIO Live Round-Trip (392m, ATE 0.49m, scale 1.001, 0 lost frames)',
    metrics_lines=[
        'Method:      ORB-SLAM3 RGB-D-Inertial live',
        'Navigation:  Pure pursuit @ 0.76 m/s',
        'IMU:         Synthetic from GT (200 Hz, Phidgets noise)',
        'Camera:      640x480 @ 10 Hz, fx=fy=320',
        'Route:       South roundtrip 392m (outbound+turnaround+return)',
        'ATE RMSE:    0.491m (max 0.824m, median 0.511m)',
        'Scale:       1.001, VIO/GT ratio 1.002',
        'Frames:      5588 tracked, 0 lost, 0 map resets',
    ],
    route='south',
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
