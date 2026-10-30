import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ired_challenge'), 'config', 'navigation_challenge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('practice_watchdog', default_value='false'),
        DeclareLaunchArgument('leave_after', default_value='0.0'),
        DeclareLaunchArgument('min_stop', default_value='5.0'),
        Node(package='ired_challenge', executable='mps_obstacles', name='mps_obstacles',
             parameters=[params], output='screen'),
        Node(package='ired_challenge', executable='navigation_challenge', name='navigation_challenge',
             parameters=[params, {'practice_watchdog': ParameterValue(LaunchConfiguration('practice_watchdog'), value_type=bool),
                                  'leave_after': ParameterValue(LaunchConfiguration('leave_after'), value_type=float),
                                  'min_stop': ParameterValue(LaunchConfiguration('min_stop'), value_type=float)}],
             output='screen'),
    ])
