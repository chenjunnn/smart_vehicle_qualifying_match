import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(get_package_share_directory(
        'lane_detector'), 'config/default.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='video_path'),

        Node(
            package='lane_detector',
            executable='lane_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[default_params_file, {'video_path': LaunchConfiguration('video_path')}],
        )
    ])
