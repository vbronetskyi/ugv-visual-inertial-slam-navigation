"""Stock Nav2 full stack: map_server + planner_server + controller_server +
behavior_server + bt_navigator + lifecycle_manager.

Replaces pure_pursuit_path_follower + send_goals_hybrid.py with Nav2's
default behavior tree (navigate_through_poses_w_replanning_and_recovery.xml
from nav2_bt_navigator) + RPP controller + stock recovery behaviors
(spin, back-up, wait, drive_on_heading).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline/config/nav2_stock_params.yaml"
    map_yaml = "/root/isaac_tr_datasets/08_nw_sw/teach/teach_outputs/teach_map.yaml"
    bt_dir = "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees"

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[config, {'use_sim_time': False}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[config, {'use_sim_time': False}]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[config, {'use_sim_time': False}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[config, {
                 'use_sim_time': False,
                 'default_nav_through_poses_bt_xml': f'{bt_dir}/navigate_through_poses_w_replanning_and_recovery.xml',
                 'default_nav_to_pose_bt_xml': f'{bt_dir}/navigate_to_pose_w_replanning_and_recovery.xml',
             }]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[config, {'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True,
                          'node_names': lifecycle_nodes,
                          'bond_timeout': 10.0,
                          'use_sim_time': False}]),
    ])
