"""BT executor: dry-run and manual run by default; dry_run:=false commands hardware."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(share('bt_executor'), 'config', 'bt_executor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('auto_run', default_value='false'),
        Node(respawn=True, respawn_delay=3.0, package='bt_executor', executable='bt_executor_node', output='screen',
             parameters=[cfg, {'dry_run': ParameterValue(LC('dry_run'), value_type=bool), 'auto_run': ParameterValue(LC('auto_run'), value_type=bool)}]),
    ])
