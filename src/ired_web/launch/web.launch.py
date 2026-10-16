from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    video_port = LaunchConfiguration('video_port')
    foxglove_port = LaunchConfiguration('foxglove_port')
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('video_port', default_value='8081'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': rosbridge_port, 'address': ''}],
            output='screen'),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': video_port}],
            output='screen'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': foxglove_port}],
            output='screen'),
    ])
