"""BT XML validator service."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(respawn=True, respawn_delay=3.0, package='bt_validator', executable='bt_validator_node', output='screen',
             parameters=[]),
    ])
