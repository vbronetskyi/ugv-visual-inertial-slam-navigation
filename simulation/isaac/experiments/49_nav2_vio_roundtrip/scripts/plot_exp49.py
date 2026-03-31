#!/usr/bin/env python3
"""Plot exp 49 - Nav2 + VIO SLAM localization on full roundtrip."""
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
from plot_trajectory_map import plot_trajectory_map

plot_trajectory_map(
    trajectories=[
        {
            'csv': '/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip/logs/exp49_gt_traj.csv',
            'label': 'GT trajectory (actual path)',
            'color': '#1f77b4',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
        {
            'csv': '/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip/logs/exp49_vio_traj.csv',
            'label': 'VIO (used by Nav2 for localization)',
            'color': '#d62728',
            'x_col': 'gt_x', 'y_col': 'gt_y',
        },
    ],
    output='/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip/results/exp49_roundtrip.png',
    title='Exp 49: Nav2 + VIO SLAM Full Round-Trip (395m, ATE 0.53m, 81/91 WPs, 0 skipped)',
    metrics_lines=[
        'Method:      Nav2 MPPI + SmacPlanner2D',
        'Localiz:     VIO SLAM (ORB-SLAM3 RGB-D-Inertial live)',
        'Warmup:      ~60s pure-pursuit before switching to VIO TF',
        'Route:       Pre-planned 91 goals @ 4m (from 797 dense WPs)',
        'Roundtrip:   outbound + natural turnaround loop + return',
        'ATE RMSE:    0.534m (max 0.891m, median 0.527m)',
        'Scale:       1.001, VIO/GT ratio 1.001',
        'Success:     81/91 reached, 0 skipped, 5600 frames, 0 lost',
    ],
    route='south',
    with_waypoints=True,
    anchors_json='/workspace/simulation/isaac/experiments/47_nav2_vio_slam/config/south_anchors_fixed.json',
)
