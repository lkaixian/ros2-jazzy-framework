"""Simulated robot (nav_sim) + navigation stack: test Nav2/SLAM/AMCL without hardware.

  ros2 launch framework_bringup nav_sim.launch.py                       # AMCL on maps/test_room
  ros2 launch framework_bringup nav_sim.launch.py map:=$HOME/maps/my_map.yaml start_x:=5.6 start_y:=0.1
  ros2 launch framework_bringup nav_sim.launch.py slam:=slam_toolbox    # build a map in the sim

Run the safety mux too (it forwards nav2/cmd_vel to the simulated robot's cmd_vel):
  ros2 launch framework_bringup extensions.launch.py scan_yaw_offset:=0.0 scan_required:=false enable_arm:=false enable_pose:=false \
      enable_odom:=false enable_pid:=false enable_social:=false enable_nav:=true
or skip it for a quick look:  cmd_vel_topic:=nav2/cmd_vel
Without slam: AMCL gets the simulated start pose as its initial pose (override: initial_pose:=x,y,yaw).
Parameters: param/nav_sim.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from bt_common.launch_utils import (declare_overrides, declare_params_dir, given, load_yaml,
                                    make_node, param_path)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGS = ('map', 'cmd_vel_topic', 'start_x', 'start_y', 'start_yaw', 'odom_slip', 'slam',
        'initial_pose', 'planner')
SIM_MAP = {'map': 'map_yaml', 'cmd_vel_topic': 'cmd_vel_topic', 'start_x': 'start_x',
           'start_y': 'start_y', 'start_yaw': 'start_yaw', 'odom_slip': 'odom_slip'}


def _actions(context):
    sim = make_node(context, 'nav_sim', 'nav_sim_node', 'nav_sim_node', 'nav_sim.yaml', SIM_MAP)
    # AMCL should start where the simulated robot starts, unless told otherwise
    launch_args = {}
    if given(context, 'initial_pose') is None and (given(context, 'slam') or 'none') == 'none':
        ref = load_yaml(param_path(context, 'nav_sim.yaml'))['nav_sim_node']['ros__parameters']
        x = float(given(context, 'start_x') or ref['start_x'])
        y = float(given(context, 'start_y') or ref['start_y'])
        yaw = float(given(context, 'start_yaw') or ref['start_yaw'])
        launch_args['initial_pose'] = f'{x},{y},{yaw}'
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('framework_bringup'), 'launch',
                                                   'navigation.launch.py')),
        launch_arguments=launch_args.items())
    return [sim, nav]


def generate_launch_description():
    return LaunchDescription([declare_params_dir(), *declare_overrides(*ARGS),
                              OpaqueFunction(function=_actions)])
