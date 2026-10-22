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
        Node(package='ired_challenge', executable='mps_obstacles', name='mps_obstacles',
             parameters=[params], output='screen'),
        Node(package='ired_challenge', executable='navigation_challenge', name='navigation_challenge',
             parameters=[params, {'practice_watchdog': ParameterValue(LaunchConfiguration('practice_watchdog'), value_type=bool)}], output='screen'),
    ])
