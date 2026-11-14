"""SLM BT-XML generator (rule-based fallback unless model_path is set)."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('bt_slm'), 'config', 'bt_slm.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        Node(respawn=True, respawn_delay=3.0, package='bt_slm', executable='bt_slm_node', output='screen',
             parameters=[cfg, {'model_path': ParameterValue(LC('model_path'), value_type=str)}]),
    ])
