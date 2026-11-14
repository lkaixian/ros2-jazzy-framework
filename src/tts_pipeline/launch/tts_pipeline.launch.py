"""Queued TTS (espeak/piper/kokoro) and plan announcer."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('tts_pipeline'), 'config', 'tts_pipeline.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('engine', default_value='espeak'),
        DeclareLaunchArgument('voice', default_value=''),
        Node(respawn=True, respawn_delay=3.0, package='tts_pipeline', executable='tts_node', output='screen',
             parameters=[cfg, {'engine': ParameterValue(LC('engine'), value_type=str), 'voice': ParameterValue(LC('voice'), value_type=str)}]),
        Node(respawn=True, respawn_delay=3.0, package='tts_pipeline', executable='plan_announcer_node', output='screen',
             parameters=[cfg]),
    ])
