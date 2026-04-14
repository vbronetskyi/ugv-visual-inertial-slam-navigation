import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = "/workspace/simulation/isaac/routes/05_ne_sw/repeat/config/nav2_planner_only.yaml"
    map_yaml = "/root/isaac_tr_datasets/05_ne_sw/teach/teach_outputs/teach_map.yaml"
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[config, {'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner', output='screen',
             parameters=[{'autostart': True,
                          'node_names': ['map_server', 'planner_server'],
                          'bond_timeout': 10.0,
                          'use_sim_time': False}]),
    ])
