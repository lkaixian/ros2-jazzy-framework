"""Navigation sensors: RPLIDAR A1 (sllidar_ros2 from ~/ros2_ws) and Intel RealSense (+ optional
depth -> 2D scan). Opens real devices: only run it on the robot.

  ros2 launch framework_bringup sensors.launch.py                 # lidar + camera
  ros2 launch framework_bringup sensors.launch.py enable_camera:=false lidar_port:=/dev/ttyUSB1
  ros2 launch framework_bringup sensors.launch.py depth_scan:=true # /depth_scan for the costmaps

Parameters: param/navigation.yaml (lidar:, depth_scan:), param/realsense.yaml (rs_launch args).
The Kobuki base driver is not started here:  ros2 launch kobuki_node kobuki_node-launch.py
(it publishes TF odom -> base and listens on commands/velocity).
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from ament_index_python.packages import PackageNotFoundError
from bt_common.launch_utils import (declare_overrides, declare_params_dir, flag, given,
                                    load_yaml, param_path)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _lidar(context, cfg):
    lc = cfg['lidar']
    try:
        pkg = share(lc['launch_package'])
    except PackageNotFoundError:
        return [LogInfo(msg=f"[sensors] package {lc['launch_package']} not found: lidar NOT "
                            f'started (build ~/ros2_ws and source it first)')]
    path = os.path.join(pkg, 'launch', lc['launch_file'])
    args = {'serial_port': lc['serial_port'], 'serial_baudrate': str(lc['serial_baudrate']),
            'frame_id': lc['frame_id'], 'angle_compensate': str(lc['angle_compensate']),
            'scan_mode': lc['scan_mode']}
    for arg, key in (('lidar_port', 'serial_port'), ('lidar_frame', 'frame_id')):
        v = given(context, arg)
        if v is not None:
            args[key] = v
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                     launch_arguments=args.items())]


def _camera(context):
    try:
        pkg = share('realsense2_camera')
    except PackageNotFoundError:
        return [LogInfo(msg='[sensors] realsense2_camera not installed: camera NOT started')]
    rs = load_yaml(param_path(context, 'realsense.yaml'))
    args = {k: (str(v).lower() if isinstance(v, bool) else str(v)) for k, v in rs.items()}
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rs_launch.py')),
        launch_arguments=args.items())]


def _depth_scan(context, cfg):
    rs = load_yaml(param_path(context, 'realsense.yaml'))
    cam = f"/{rs.get('camera_namespace', 'camera')}/{rs.get('camera_name', 'camera')}"
    return [Node(package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
                 name='depthimage_to_laserscan', output='screen',
                 parameters=[dict(cfg['depth_scan'])],
                 remappings=[('depth', f'{cam}/depth/image_rect_raw'),
                             ('depth_camera_info', f'{cam}/depth/camera_info'),
                             ('scan', '/depth_scan')])]


def _actions(context):
    cfg = load_yaml(param_path(context, 'navigation.yaml'))
    out = []
    if flag(context, 'enable_lidar'):
        out += _lidar(context, cfg)
    if flag(context, 'enable_camera'):
        out += _camera(context)
    if flag(context, 'depth_scan'):
        out += _depth_scan(context, cfg)
    return out


def generate_launch_description():
    return LaunchDescription([
        declare_params_dir(),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('depth_scan', default_value='false',
                              description='publish /depth_scan from the RealSense depth image'),
        *declare_overrides('lidar_port', 'lidar_frame'),
        OpaqueFunction(function=_actions),
    ])
