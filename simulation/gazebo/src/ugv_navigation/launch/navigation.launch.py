"""
Launch Nav2 + SLAM Toolbox for the UGV

Usage:
  ros2 launch ugv_navigation navigation.launch.py

Modes:
  slam:=true  - build map while navigating (default)
  slam:=false - use existing map with AMCL
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('ugv_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # args
    use_slam = LaunchConfiguration('slam', default='true')
    params_file = PathJoinSubstitution([pkg_nav, 'config', 'nav2_params.yaml'])
    map_file = LaunchConfiguration('map', default='')

    # SLAM Toolbox - when slam:=true
    slam_toolbox = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items(),
    )

    # rviz2
    rviz_config = PathJoinSubstitution([pkg_nav, 'config', 'nav2_view.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Run SLAM (true) or localization (false)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML file for localization mode'),
        slam_toolbox,
        nav2_bringup,
        rviz,
    ])
