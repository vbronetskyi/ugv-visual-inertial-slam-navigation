"""
Launch Gazebo Harmonic simulation with Husky A200 (no SLAM or Nav2)

Usage:
  ros2 launch ugv_gazebo simulation.launch.py headless:=true
  ros2 launch ugv_gazebo simulation.launch.py world:=warehouse
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('ugv_gazebo')
    pkg_description = FindPackageShare('ugv_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    headless = LaunchConfiguration('headless', default='false')
    world_name = LaunchConfiguration('world', default='outdoor_terrain')

    world_file = PathJoinSubstitution([
        pkg_gazebo, 'worlds',
        PythonExpression(["'", world_name, "' + '.sdf'"])
    ])
    bridge_config = PathJoinSubstitution([pkg_gazebo, 'config', 'bridge_config.yaml'])
    robot_sdf = PathJoinSubstitution([pkg_description, 'sdf', 'model.sdf'])
    robot_urdf = PathJoinSubstitution([pkg_description, 'urdf', 'husky_frames.urdf'])

    gz_args = PythonExpression([
        "'-r -s ' + '", world_file, "' if '", headless,
        "' == 'true' else '-r ' + '", world_file, "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'husky_a200',
            '-file', robot_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', robot_urdf]), value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run in headless mode (server only, no GUI)'),
        DeclareLaunchArgument('world', default_value='outdoor_terrain',
                              description='World name: outdoor_terrain or warehouse'),
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_publisher,
    ])
