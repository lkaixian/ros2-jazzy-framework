"""Bring up everything: perception + extensions + LLM/SLM/validator pipeline.

ros2 launch framework_bringup framework.launch.py image_topic:=/camera/image_raw \
    start_llama_server:=true llama_model:=/models/qwen2.5-3b.gguf \
    slm_model_path:=/models/bt-slm-250m enable_face:=true
Every argument of the included launch files is forwarded (audio only if enable_audio:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    d = os.path.join(share('framework_bringup'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(d, f)))
        for f in ('perception.launch.py', 'extensions.launch.py', 'pipeline.launch.py')])
