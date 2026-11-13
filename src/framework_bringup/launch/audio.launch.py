"""Audio: microphone -> VAD -> STT -> bt/task, and tts/say -> speaker (+ plan announcer)."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'enable_stt': 'true', 'enable_tts': 'true', 'announce_plans': 'true',
    'audio_source': 'arecord',      # arecord | file
    'audio_device': '',             # ALSA capture device, e.g. plughw:2,0
    'audio_file': '',               # 16 kHz mono 16-bit WAV when audio_source:=file
    'stt_engine': 'faster_whisper',  # faster_whisper | vosk | mock
    'stt_model': 'base.en',
    'wake_word': '',
    'tts_engine': 'espeak',         # espeak | piper | kokoro | mock
    'tts_voice': '',
    'piper_model': '',
    'kokoro_model': '',
    'kokoro_voices': '',
    'playback_device': '',
}


def _s(name):
    """String-typed parameter from a launch argument (keeps '' and '16000' as strings)."""
    return ParameterValue(LC(name), value_type=str)


def generate_launch_description():
    cfg = lambda pkg: os.path.join(share(pkg), 'config', f'{pkg}.yaml')  # noqa: E731
    stt = IfCondition(LC('enable_stt'))
    tts = IfCondition(LC('enable_tts'))
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            Node(package='audio_capture', executable='audio_capture_node', condition=stt,
                 parameters=[cfg('audio_capture'), {'source': _s('audio_source'),
                                                    'device': _s('audio_device'),
                                                    'file': _s('audio_file')}],
                 output='screen'),
            Node(package='stt_pipeline', executable='vad_node', condition=stt,
                 parameters=[cfg('stt_pipeline')], output='screen'),
            Node(package='stt_pipeline', executable='stt_node', condition=stt,
                 parameters=[cfg('stt_pipeline'), {'engine': _s('stt_engine'),
                                                   'model': _s('stt_model')}],
                 output='screen'),
            Node(package='stt_pipeline', executable='transcript_to_task_node', condition=stt,
                 parameters=[cfg('stt_pipeline'), {'wake_word': _s('wake_word')}],
                 output='screen'),
            Node(package='tts_pipeline', executable='tts_node', condition=tts,
                 parameters=[cfg('tts_pipeline'), {'engine': _s('tts_engine'),
                                                   'voice': _s('tts_voice'),
                                                   'piper_model': _s('piper_model'),
                                                   'kokoro_model': _s('kokoro_model'),
                                                   'kokoro_voices': _s('kokoro_voices'),
                                                   'audio_device': _s('playback_device')}],
                 output='screen'),
            Node(package='tts_pipeline', executable='plan_announcer_node',
                 condition=IfCondition(LC('announce_plans')),
                 parameters=[cfg('tts_pipeline')], output='screen'),
        ])
