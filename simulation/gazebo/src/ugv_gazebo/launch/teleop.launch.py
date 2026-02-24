"""
Teleop launch for Husky A200: keyboard + Xbox 360 gamepad

Usage:
  # keyboard only:
  ros2 launch ugv_gazebo teleop.launch.py

  # xbox 360 gamepad:  ros2 launch ugv_gazebo teleop.launch.py joy:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration('joy', default='false')

    # keyboard teleop - always available
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        parameters=[{'use_sim_time': True}],
        # husky max: 1.0 m/s linear, 1.0 rad/s angular
        prefix='xterm -e',
        output='screen',
    )

    # joy node (Xbox 360 gamepad)
    joy_node = Node(
        condition=IfCondition(use_joy),
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': True,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    # teleop twist joy - converts joy msgs to cmd_vel
    teleop_joy = Node(
        condition=IfCondition(use_joy),
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'use_sim_time': True,
            'axis_linear.x': 1,
            'scale_linear.x': 1.0,
            'axis_angular.yaw': 0,
            'scale_angular.yaw': 1.0,
            'enable_button': 4,
            'enable_turbo_button': 5,
            'scale_linear_turbo.x': 1.0,
            'scale_angular_turbo.yaw': 1.0,
        }],
        remappings=[('cmd_vel', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('joy', default_value='false',
                              description='Enable Xbox 360 gamepad teleop'),
        keyboard_teleop,
        joy_node,
        teleop_joy,
    ])
