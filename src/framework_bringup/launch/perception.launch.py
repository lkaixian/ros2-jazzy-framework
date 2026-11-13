"""YOLO11 perception: object / person / face detectors."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _node(pkg, exe, enable):
    cfg = os.path.join(share(pkg), 'config', f'{pkg}.yaml')
    return Node(package=pkg, executable=exe, parameters=[
        cfg, {'image_topic': ParameterValue(LC('image_topic'), value_type=str),
              'show': ParameterValue(LC('show'), value_type=bool)}],
        condition=IfCondition(LC(enable)), output='screen')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='',
                              description='sensor_msgs/Image topic; empty = open webcam'),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('enable_object', default_value='true'),
        DeclareLaunchArgument('enable_person', default_value='true'),
        DeclareLaunchArgument('enable_face', default_value='false',
                              description='needs user-supplied YOLO11 face weights'),
        _node('object_detection', 'object_detection_node', 'enable_object'),
        _node('person_detection', 'person_detection_node', 'enable_person'),
        _node('face_recognition', 'face_recognition_node', 'enable_face'),
    ])
