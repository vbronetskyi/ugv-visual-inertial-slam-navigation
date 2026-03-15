"""
Nav2 launch for Husky A200 in Isaac Sim.
Starts: controller_server (MPPI), planner_server, costmap nodes,
        behavior_server, bt_navigator, velocity_smoother, lifecycle_manager.
Does NOT start localization (ORB-SLAM3 handles map->odom tf separately).

usage:
  source /opt/ros/jazzy/setup.bash
  ros2 launch nav2_husky_launch.py
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

    map_yaml = '/workspace/simulation/isaac/results/navigation/slam_map/map.yaml'

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # map server - publishes static SLAM map for global costmap
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
            remappings=[],  # no velocity_smoother, publish directly to /cmd_vel
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

        # velocity_smoother removed - controller publishes directly to /cmd_vel

        # depth_image_proc removed - tf_wall_clock_relay.py converts depth->pointcloud
        # directly with wall clock timestamps (avoids sim_time sync issues)

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
