"""Speak / EnrollFace executor (needs tts_node and face_recognition_node running)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='social_extension',
                                   executable='social_extension_node', output='screen')])
