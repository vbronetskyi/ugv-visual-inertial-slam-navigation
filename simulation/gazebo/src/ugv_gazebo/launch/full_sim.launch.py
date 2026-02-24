"""
Full Husky A200 simulation: Gazebo + SLAM + Nav2

Usage:
  # SLAM Toolbox (default, lightweight, uses 2D LiDAR):
  ros2 launch ugv_gazebo full_sim.launch.py headless:=true

  # RTAB-Map SLAM (uses D435i depth, heavier):  ros2 launch ugv_gazebo full_sim.launch.py slam_type:=rtabmap headless:=true

  # use warehouse world:  ros2 launch ugv_gazebo full_sim.launch.py world:=warehouse headless:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('ugv_gazebo')
    pkg_description = FindPackageShare('ugv_description')
    pkg_navigation = FindPackageShare('ugv_navigation')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # args
    headless = LaunchConfiguration('headless', default='false')
    slam_type = LaunchConfiguration('slam_type', default='none')
    world_name = LaunchConfiguration('world', default='outdoor_terrain')

    # paths
    world_file = PathJoinSubstitution([
        pkg_gazebo, 'worlds',
        PythonExpression(["'", world_name, "' + '.sdf'"])
    ])
    bridge_config = PathJoinSubstitution([pkg_gazebo, 'config', 'bridge_config.yaml'])
    robot_sdf = PathJoinSubstitution([pkg_description, 'sdf', 'model.sdf'])
    robot_urdf = PathJoinSubstitution([pkg_description, 'urdf', 'husky_frames.urdf'])
    nav2_params = PathJoinSubstitution([pkg_navigation, 'config', 'nav2_params.yaml'])
    rtabmap_params = PathJoinSubstitution([pkg_navigation, 'config', 'rtabmap_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_navigation, 'config', 'nav2_view.rviz'])

    gz_args = PythonExpression([
        "'-r -s ' + '", world_file, "' if '", headless,
        "' == 'true' else '-r ' + '", world_file, "'"
    ])

    # GZ_SIM_RESOURCE_PATH lets Gazebo find file:// URIs for heightmap/textures.
    # Without this the outdoor_terrain heightmap renders as a flat plane.
    worlds_dir = PathJoinSubstitution([pkg_gazebo, 'worlds'])

    nav2_common = {'use_sim_time': True}
    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ===================== Gazebo =====================
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
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'husky_a200', '-file', robot_sdf,
                   '-x', '-105.0', '-y', '-8.0', '-z', '12.0'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # ===================== TF =====================
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', robot_urdf]), value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ===================== SLAM =====================
    use_slam_toolbox = PythonExpression(["'", slam_type, "' == 'slam_toolbox'"])
    slam_toolbox_node = Node(
        condition=IfCondition(use_slam_toolbox),
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[nav2_params, {'use_sim_time': True}],
        output='screen',
    )

    # dedicated lifecycle manager for slam_toolbox - no bond needed
    slam_lifecycle_manager = Node(
        condition=IfCondition(use_slam_toolbox),
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_slam', output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    use_rtabmap = PythonExpression(["'", slam_type, "' == 'rtabmap'"])
    rtabmap_node = Node(
        condition=IfCondition(use_rtabmap),
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw'),
            ('odom', '/odom'), ('imu', '/imu/data'),
        ],
        arguments=['--delete_db_on_start'],
        output='screen',
        additional_env={'OMP_NUM_THREADS': '2'},
    )

    # ===================== Nav2 Core =====================
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[nav2_params, nav2_common],
        remappings=nav2_remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # Nav2 lifecycle - delayed so SLAM can publish /map first
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'collision_monitor',
                'velocity_smoother',
            ],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
        }],
    )

    # ===================== RViz =====================
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", headless, "' != 'true'"])),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('slam_type', default_value='slam_toolbox',
                              description='slam_toolbox or rtabmap'),
        DeclareLaunchArgument('world', default_value='outdoor_terrain'),
        # tell Gazebo where file:// resources live
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds_dir),
        # Gazebo
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_publisher,
        # SLAM - lifecycle_manager_slam auto-activates slam_toolbox
        slam_toolbox_node,
        TimerAction(period=10.0, actions=[slam_lifecycle_manager]),
        rtabmap_node,
        # Nav2 core - delayed for /map
        TimerAction(period=15.0, actions=[
            controller_server,
            planner_server,
            bt_navigator,
            behavior_server,
            collision_monitor,
            velocity_smoother,
        ]),
        TimerAction(period=30.0, actions=[nav2_lifecycle_manager]),
        # viz
        rviz,
    ])
