"""Cluster platform (job placement across machines).

Thin wrapper so it can be launched from the workspace root:
    ros2 launch launch/cluster.launch.py role:=agent host_id:=pi roles:=core
It includes src/framework_bringup/launch/cluster.launch.py (or the installed copy), so all
arguments documented there apply. The workspace must be built and sourced.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAME = 'cluster.launch.py'


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    src = os.path.join(here, '..', 'src', 'framework_bringup', 'launch', NAME)
    path = src if os.path.exists(src) else os.path.join(
        get_package_share_directory('framework_bringup'), 'launch', NAME)
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(path))])
