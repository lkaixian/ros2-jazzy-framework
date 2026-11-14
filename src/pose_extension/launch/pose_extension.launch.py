"""Gesture conditions from person keypoints."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('persons_topic', default_value='persons'),
        Node(respawn=True, respawn_delay=3.0, package='pose_extension', executable='pose_extension_node', output='screen',
             parameters=[{'persons_topic': ParameterValue(LC('persons_topic'), value_type=str)}]),
    ])
