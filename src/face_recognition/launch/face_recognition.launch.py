import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('face_recognition'), 'config', 'face_recognition.yaml')
    return LaunchDescription([Node(respawn=True, respawn_delay=3.0, package='face_recognition', executable='face_recognition_node',
                                   parameters=[cfg], output='screen')])
