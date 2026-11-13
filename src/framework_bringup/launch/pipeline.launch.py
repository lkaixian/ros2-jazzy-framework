"""LLM (llama.cpp, C++) -> SLM -> validator pipeline."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_llama_server', default_value='false',
                              description='spawn llama-server (needs llama.cpp on PATH)'),
        DeclareLaunchArgument('llama_model', default_value='',
                              description='path to 2B-4B GGUF for llama-server'),
        DeclareLaunchArgument('llama_url', default_value='http://127.0.0.1:8080'),
        DeclareLaunchArgument('llama_port', default_value='8080'),
        DeclareLaunchArgument('slm_model_path', default_value='',
                              description='fine-tuned SLM dir (base: LiquidAI/LFM2.5-230M-Base); '
                                              'empty = rule-based fallback'),
        DeclareLaunchArgument('max_attempts', default_value='3'),

        ExecuteProcess(
            cmd=['llama-server', '-m', LC('llama_model'), '--port', LC('llama_port')],
            condition=IfCondition(LC('start_llama_server')), output='screen'),

        Node(package='llm_bridge', executable='llm_bridge_node', output='screen',
             parameters=[os.path.join(share('llm_bridge'), 'config', 'llm_bridge.yaml'),
                         {'server_url': LC('llama_url')}]),
        Node(package='bt_slm', executable='bt_slm_node', output='screen',
             parameters=[os.path.join(share('bt_slm'), 'config', 'bt_slm.yaml'),
                         {'model_path': LC('slm_model_path')}]),
        Node(package='bt_validator', executable='bt_validator_node', output='screen'),
        Node(package='bt_orchestrator', executable='orchestrator_node', output='screen',
             parameters=[{'max_attempts': ParameterValue(LC('max_attempts'), value_type=int)}]),
        Node(package='bt_orchestrator', executable='scene_summarizer_node', output='screen'),
    ])
