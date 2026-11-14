"""PID executor (publishes pid_extension/cmd_vel; run with cmd_vel_mux_node)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('persons_topic', default_value='persons'),
        Node(respawn=True, respawn_delay=3.0, package='pid_extension', executable='pid_extension_node', output='screen',
             parameters=[{'odom_topic': ParameterValue(LC('odom_topic'), value_type=str), 'persons_topic': ParameterValue(LC('persons_topic'), value_type=str)}]),
    ])
