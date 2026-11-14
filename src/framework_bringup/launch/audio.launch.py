"""Audio: microphone -> VAD -> STT -> bt/task, and tts/say -> speaker (+ plan announcer).
Parameters: param/audio.yaml."""
from bt_common.launch_utils import (declare_overrides, declare_params_dir, flag, make_node)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

ARGS = ('audio_source', 'audio_device', 'audio_file', 'stt_engine', 'stt_model', 'wake_word',
        'tts_engine', 'tts_voice', 'piper_model', 'kokoro_model', 'kokoro_voices',
        'playback_device')
F = 'audio.yaml'


def _nodes(context):
    n = lambda *a, **k: make_node(context, *a, **k)  # noqa: E731
    out = []
    if flag(context, 'enable_stt'):
        out += [
            n('audio_capture', 'audio_capture_node', 'audio_capture_node', F,
              {'audio_source': 'source', 'audio_device': 'device', 'audio_file': 'file'}),
            n('stt_pipeline', 'vad_node', 'vad_node', F),
            n('stt_pipeline', 'sound_event_node', 'sound_event_node', F),
            n('stt_pipeline', 'stt_node', 'stt_node', F,
              {'stt_engine': 'engine', 'stt_model': 'model'}),
            n('stt_pipeline', 'transcript_to_task_node', 'transcript_to_task_node', F,
              {'wake_word': 'wake_word'}),
        ]
    if flag(context, 'enable_tts'):
        out.append(n('tts_pipeline', 'tts_node', 'tts_node', F,
                     {'tts_engine': 'engine', 'tts_voice': 'voice', 'piper_model': 'piper_model',
                      'kokoro_model': 'kokoro_model', 'kokoro_voices': 'kokoro_voices',
                      'playback_device': 'audio_device'}))
    if flag(context, 'announce_plans'):
        out.append(n('tts_pipeline', 'plan_announcer_node', 'plan_announcer_node', F))
    return out


def generate_launch_description():
    return LaunchDescription(
        [declare_params_dir(),
         DeclareLaunchArgument('enable_stt', default_value='true'),
         DeclareLaunchArgument('enable_tts', default_value='true'),
         DeclareLaunchArgument('announce_plans', default_value='true')]
        + declare_overrides(*ARGS) + [OpaqueFunction(function=_nodes)])
