"""Extension executors (arm/pose/odom/pid) and the cmd_vel safety mux.

Only the mux publishes the robot's real velocity topic: it applies priority (odom_extension
over pid_extension), a 0.5 s watchdog and hard velocity clamps.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

EXTENSIONS = ('arm', 'pose', 'odom', 'pid')


def _s(name):
    return ParameterValue(LC(name), value_type=str)


def _b(name):
    return ParameterValue(LC(name), value_type=bool)


def generate_launch_description():
    args = {f'enable_{e}': 'true' for e in EXTENSIONS}
    args.update({
        'odom_topic': 'odom', 'cmd_vel_topic': 'cmd_vel', 'cmd_vel_stamped': 'false',
        'max_linear': '0.2', 'max_angular': '0.6',
        'persons_topic': 'persons',
        # MoveIt (arm model not fixed yet): arm_extension refuses to move until these are set
        'arm_group': '', 'arm_ee_link': '', 'arm_base_frame': '',
        'move_group_action': 'move_action', 'gripper_action': '',
    })
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    mux_cfg = os.path.join(share('bt_common'), 'config', 'cmd_vel_mux.yaml')
    actions += [
        Node(package='bt_common', executable='cmd_vel_mux_node', output='screen',
             parameters=[mux_cfg, {'output': _s('cmd_vel_topic'), 'stamped': _b('cmd_vel_stamped'),
                                   'max_linear': ParameterValue(LC('max_linear'), value_type=float),
                                   'max_angular': ParameterValue(LC('max_angular'), value_type=float)}]),
        Node(package='odom_extension', executable='odom_extension_node', output='screen',
             condition=IfCondition(LC('enable_odom')),
             parameters=[{'odom_topic': _s('odom_topic'),
                          'max_linear': ParameterValue(LC('max_linear'), value_type=float),
                          'max_angular': ParameterValue(LC('max_angular'), value_type=float)}]),
        Node(package='pid_extension', executable='pid_extension_node', output='screen',
             condition=IfCondition(LC('enable_pid')),
             parameters=[{'odom_topic': _s('odom_topic'), 'persons_topic': _s('persons_topic'),
                          'max_angular': ParameterValue(LC('max_angular'), value_type=float)}]),
        Node(package='pose_extension', executable='pose_extension_node', output='screen',
             condition=IfCondition(LC('enable_pose')),
             parameters=[{'persons_topic': _s('persons_topic')}]),
        Node(package='arm_extension', executable='arm_extension_node', output='screen',
             condition=IfCondition(LC('enable_arm')),
             parameters=[{'group': _s('arm_group'), 'ee_link': _s('arm_ee_link'),
                          'base_frame': _s('arm_base_frame'),
                          'move_group_action': _s('move_group_action'),
                          'gripper_action': _s('gripper_action')}]),
    ]
    return LaunchDescription(actions)
