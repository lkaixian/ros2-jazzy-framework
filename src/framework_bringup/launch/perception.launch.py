"""YOLO11 perception: object / person / face detectors. Parameters: param/perception.yaml."""
from bt_common.launch_utils import (declare_overrides, declare_params_dir, flag, make_node)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

COMMON = {'image_topic': 'image_topic', 'show': 'show'}


def _nodes(context):
    out = []
    for flag_name, pkg, exe, node in [
            ('enable_object', 'object_detection', 'object_detection_node', 'object_detection_node'),
            ('enable_person', 'person_detection', 'person_detection_node', 'person_detection_node'),
            ('enable_face', 'face_recognition', 'face_recognition_node', 'face_recognition_node')]:
        if flag(context, flag_name):
            out.append(make_node(context, pkg, exe, node, 'perception.yaml', COMMON))
            if flag_name == 'enable_face':
                out.append(make_node(context, pkg, 'auto_register_node', 'auto_register_node', 'perception.yaml'))
    if flag(context, 'enable_depth_ranging'):
        out.append(make_node(context, 'person_detection', 'person_depth_node', 'person_depth_node',
                             'perception.yaml'))
    return out


def generate_launch_description():
    return LaunchDescription([
        declare_params_dir(),
        DeclareLaunchArgument('enable_object', default_value='true'),
        DeclareLaunchArgument('enable_person', default_value='true'),
        DeclareLaunchArgument('enable_face', default_value='false',
                              description='needs user-supplied YOLO11 face weights'),
        DeclareLaunchArgument('enable_depth_ranging', default_value='false',
                              description='persons_ranged with distance_m from the RealSense depth'),
        *declare_overrides(*COMMON),
        OpaqueFunction(function=_nodes),
    ])
