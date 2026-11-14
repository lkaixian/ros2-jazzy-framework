"""BT XML validator service."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        Node(respawn=True, respawn_delay=3.0, package='bt_validator', executable='bt_validator_node', output='screen',
             parameters=[]),
    ])
