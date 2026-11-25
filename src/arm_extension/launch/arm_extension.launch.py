"""Guarded arm executor (params: framework_bringup/param/extensions.yaml, `live` is false unless set)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(respawn=True, respawn_delay=3.0, package='arm_extension', executable='arm_extension_node', output='screen')])
