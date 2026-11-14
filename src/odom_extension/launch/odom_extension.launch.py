"""Odometry motion executor (publishes odom_extension/cmd_vel; run with cmd_vel_mux_node)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        Node(respawn=True, respawn_delay=3.0, package='odom_extension', executable='odom_extension_node', output='screen',
             parameters=[{'odom_topic': ParameterValue(LC('odom_topic'), value_type=str)}]),
    ])
