"""Everything: perception + extensions + LLM/SLM pipeline (+ audio with enable_audio:=true).

Thin wrapper so the stack can be launched from the workspace root:
    ros2 launch launch/framework.launch.py [arg:=value ...]
It includes the matching file of src/framework_bringup (or the installed copy), so all
arguments documented there apply. The workspace must be built and sourced.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAME = 'framework.launch.py'


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    src = os.path.join(here, '..', 'src', 'framework_bringup', 'launch', NAME)
    path = src if os.path.exists(src) else os.path.join(
        get_package_share_directory('framework_bringup'), 'launch', NAME)
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(path))])
