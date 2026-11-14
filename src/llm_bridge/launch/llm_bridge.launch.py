"""C++ llama.cpp bridge (needs a running llama-server)."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('llm_bridge'), 'config', 'llm_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://127.0.0.1:8080'),
        Node(respawn=True, respawn_delay=3.0, package='llm_bridge', executable='llm_bridge_node', output='screen',
             parameters=[cfg, {'server_url': ParameterValue(LC('server_url'), value_type=str)}]),
    ])
