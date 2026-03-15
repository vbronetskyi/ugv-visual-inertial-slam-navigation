"""
Nav2 launch for SLAM-frame navigation.
Uses empty occupancy grid - only depth obstacle layer for avoidance.
No static map needed: robot + waypoints both in SLAM coordinate frame.

usage:
  source /opt/ros/jazzy/setup.bash
  ros2 launch nav2_slam_frame_launch.py use_sim_time:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # NOTE: relies on scripts/common/ being on path
    config_dir = os.path.dirname(os.path.abspath(__file__))
    params_file = os.path.join(config_dir, 'nav2_husky_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # empty map in SLAM frame (origin at -50,-50, 200x200m)
    map_yaml = os.path.join(config_dir, 'slam_frame_map.yaml')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 20.0,
            }],
        ),
    ])
