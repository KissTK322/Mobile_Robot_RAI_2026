import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ired_challenge'), 'config', 'navigation_challenge.yaml')
    return LaunchDescription([
        Node(package='ired_challenge', executable='mps_obstacles', name='mps_obstacles',
             parameters=[params], output='screen'),
        Node(package='ired_challenge', executable='navigation_challenge', name='navigation_challenge',
             parameters=[params], output='screen'),
    ])
