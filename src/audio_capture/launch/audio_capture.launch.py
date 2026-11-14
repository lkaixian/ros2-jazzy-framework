"""Microphone (arecord) or WAV-file source."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('audio_capture'), 'config', 'audio_capture.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='arecord'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('file', default_value=''),
        Node(respawn=True, respawn_delay=3.0, package='audio_capture', executable='audio_capture_node', output='screen',
             parameters=[cfg, {'source': ParameterValue(LC('source'), value_type=str), 'device': ParameterValue(LC('device'), value_type=str), 'file': ParameterValue(LC('file'), value_type=str)}]),
    ])
