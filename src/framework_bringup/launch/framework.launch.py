"""Bring up everything: perception + extensions + LLM/SLM/validator/executor pipeline,
plus, only when asked for: enable_audio:=true (STT/TTS), enable_sensors:=true (lidar + RealSense)
and enable_navigation:=true (Nav2 + SLAM/AMCL).

ros2 launch framework_bringup framework.launch.py image_topic:=/camera/image_raw \
    start_llama_server:=true slm_model_path:=/models/bt-slm-250m enable_face:=true \
    enable_audio:=true enable_sensors:=true enable_navigation:=true map:=$HOME/maps/room1.yaml

Parameters live in framework_bringup/param/*.yaml (folder: params_dir:=...). Launch arguments
given on the command line override the files; arguments left out change nothing.
Every argument of the included launch files is forwarded.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from bt_common.launch_utils import declare_params_dir
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    d = os.path.join(share('framework_bringup'), 'launch')

    def include(name, **kw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(d, name)), **kw)

    return LaunchDescription([
        declare_params_dir(),
        # the safety mux must publish where the Kobuki driver listens, else PID / odom / Nav2 commands go nowhere
        # (nav_sim.launch.py keeps `cmd_vel` for the simulator). Motion is still gated by dry_run (default true).
        DeclareLaunchArgument('cmd_vel_topic', default_value='commands/velocity',
                              description='topic cmd_vel_mux publishes to; the Kobuki driver listens on commands/velocity'),
        DeclareLaunchArgument('enable_audio', default_value='false',
                              description='STT/TTS pipeline (opens the microphone; needs an '
                                          'STT engine installed, see param/audio.yaml)'),
        DeclareLaunchArgument('enable_sensors', default_value='false',
                              description='RPLIDAR + RealSense drivers (sensors.launch.py); needs '
                                          'the devices attached'),
        DeclareLaunchArgument('sensors_delay', default_value='10.0',
                              description='seconds to wait before starting the sensors (lidar)'),
        DeclareLaunchArgument('enable_navigation', default_value='false',
                              description='Nav2 + SLAM/AMCL (navigation.launch.py); pass slam:=... '
                                          'to map or map:=<yaml> to navigate. The odom->base TF '
                                          '(Kobuki driver) must come from elsewhere'),
        include('perception.launch.py'),
        include('extensions.launch.py'),
        include('pipeline.launch.py'),
        include('audio.launch.py', condition=IfCondition(LaunchConfiguration('enable_audio'))),
        # the RPLIDAR died ~3 s after start when launched together with the other ~30 nodes
        TimerAction(period=LaunchConfiguration('sensors_delay'), actions=[
            include('sensors.launch.py', condition=IfCondition(LaunchConfiguration('enable_sensors')))]),
        include('navigation.launch.py',
                condition=IfCondition(LaunchConfiguration('enable_navigation'))),
    ])
