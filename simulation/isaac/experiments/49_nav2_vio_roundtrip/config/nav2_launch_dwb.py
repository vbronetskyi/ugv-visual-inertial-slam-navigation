"""Nav2 launch exp 49 continuation: DWB controller + NavFn planner."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    config_dir = "/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip/config"
    params_file = os.path.join(config_dir, 'nav2_params_dwb.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file, param_rewrites=param_substitutions, convert_types=True)
    map_yaml = os.path.join(config_dir, 'blank_south_map.yaml')
    lifecycle_nodes = ['map_server', 'controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'velocity_smoother']
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_raw')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[configured_params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[configured_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[configured_params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[configured_params],
             remappings=[('cmd_vel', 'cmd_vel_raw'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': lifecycle_nodes, 'bond_timeout': 10.0}]),
    ])
