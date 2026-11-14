"""VAD -> STT -> bt/task (engine must be installed; mock only tests plumbing)."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('stt_pipeline'), 'config', 'stt_pipeline.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('engine', default_value='faster_whisper'),
        DeclareLaunchArgument('model', default_value='base.en'),
        DeclareLaunchArgument('wake_word', default_value=''),
        Node(respawn=True, respawn_delay=3.0, package='stt_pipeline', executable='vad_node', output='screen',
             parameters=[cfg]),
        Node(respawn=True, respawn_delay=3.0, package='stt_pipeline', executable='stt_node', output='screen',
             parameters=[cfg, {'engine': ParameterValue(LC('engine'), value_type=str), 'model': ParameterValue(LC('model'), value_type=str)}]),
        Node(respawn=True, respawn_delay=3.0, package='stt_pipeline', executable='transcript_to_task_node', output='screen',
             parameters=[cfg, {'wake_word': ParameterValue(LC('wake_word'), value_type=str)}]),
    ])
