"""MoveIt arm executor: refuses to move until group/ee_link/base_frame are set."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_group', default_value=''),
        DeclareLaunchArgument('arm_ee_link', default_value=''),
        DeclareLaunchArgument('arm_base_frame', default_value=''),
        DeclareLaunchArgument('move_group_action', default_value='move_action'),
        DeclareLaunchArgument('gripper_action', default_value=''),
        Node(respawn=True, respawn_delay=3.0, package='arm_extension', executable='arm_extension_node', output='screen',
             parameters=[{'group': ParameterValue(LC('arm_group'), value_type=str), 'ee_link': ParameterValue(LC('arm_ee_link'), value_type=str), 'base_frame': ParameterValue(LC('arm_base_frame'), value_type=str),
                          'move_group_action': ParameterValue(LC('move_group_action'), value_type=str), 'gripper_action': ParameterValue(LC('gripper_action'), value_type=str)}]),
    ])
