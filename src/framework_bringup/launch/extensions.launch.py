"""Extension executors (arm/pose/odom/pid/social) and the cmd_vel safety mux.
Parameters: param/extensions.yaml. Only the mux publishes the robot's real velocity topic.
"""
from bt_common.launch_utils import (declare_overrides, declare_params_dir, flag, make_node)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

ARGS = ('odom_topic', 'cmd_vel_topic', 'cmd_vel_stamped', 'max_linear', 'max_angular',
        'persons_topic', 'arm_group', 'arm_ee_link', 'arm_base_frame', 'move_group_action',
        'gripper_action', 'scan_yaw_offset', 'scan_required')
F = 'extensions.yaml'


def _nodes(context):
    n = lambda *a, **k: make_node(context, *a, **k)  # noqa: E731
    out = [n('bt_common', 'battery_guard_node', 'battery_guard_node', F),
           n('bt_common', 'cmd_vel_mux_node', 'cmd_vel_mux_node', F,
             {'cmd_vel_topic': 'output', 'cmd_vel_stamped': 'stamped',
              'max_linear': 'max_linear', 'max_angular': 'max_angular'})]
    if flag(context, 'enable_odom'):
        out.append(n('odom_extension', 'odom_extension_node', 'odom_extension_node', F,
                     {'odom_topic': 'odom_topic', 'max_linear': 'max_linear',
                      'max_angular': 'max_angular', 'scan_yaw_offset': 'scan_yaw_offset',
                      'scan_required': 'scan_required'}))
    if flag(context, 'enable_pid'):
        out.append(n('pid_extension', 'pid_extension_node', 'pid_extension_node', F,
                     {'odom_topic': 'odom_topic', 'persons_topic': 'persons_topic',
                      'max_angular': 'max_angular', 'scan_yaw_offset': 'scan_yaw_offset',
                      'scan_required': 'scan_required'}))
    if flag(context, 'enable_pose'):
        out.append(n('pose_extension', 'pose_extension_node', 'pose_extension_node', F,
                     {'persons_topic': 'persons_topic'}))
    if flag(context, 'enable_social'):
        out.append(n('social_extension', 'social_extension_node', 'social_extension_node', F))
    if flag(context, 'enable_nav'):
        out.append(n('nav_extension', 'nav_extension_node', 'nav_extension_node', F))
    if flag(context, 'enable_arm'):
        out.append(n('arm_extension', 'arm_extension_node', 'arm_extension_node', F,
                     {'arm_group': 'group', 'arm_ee_link': 'ee_link',
                      'arm_base_frame': 'base_frame', 'move_group_action': 'move_group_action',
                      'gripper_action': 'gripper_action'}))
    return out


def generate_launch_description():
    return LaunchDescription(
        [declare_params_dir()]
        + [DeclareLaunchArgument(f'enable_{e}', default_value='true')
           for e in ('arm', 'pose', 'odom', 'pid', 'social', 'nav')]
        + declare_overrides(*ARGS) + [OpaqueFunction(function=_nodes)])
