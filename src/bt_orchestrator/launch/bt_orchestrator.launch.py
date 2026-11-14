"""Orchestrator (generate -> validate -> retry) and scene summarizer."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('bt_orchestrator'), 'config', 'bt_orchestrator.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('max_attempts', default_value='3'),
        Node(respawn=True, respawn_delay=3.0, package='bt_orchestrator', executable='orchestrator_node', output='screen',
             parameters=[cfg, {'max_attempts': ParameterValue(LC('max_attempts'), value_type=int)}]),
        Node(respawn=True, respawn_delay=3.0, package='bt_orchestrator', executable='scene_summarizer_node', output='screen',
             parameters=[cfg]),
    ])
