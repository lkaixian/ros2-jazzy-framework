import json

import numpy as np
import rclpy
from audio_msgr.msg import AudioChunk, TtsStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from stt_pipeline.sound import ToneDetector


class SoundEventNode(Node):
    """audio/raw -> audio/events ({"kind": "doorbell", "freq": Hz}) when a steady tone (bell) is heard.

    Ignores the audio while the robot is speaking (its own voice and beeps are not events).
    """

    def __init__(self):
        super().__init__('sound_event_node')
        for k, v in dict(window_sec=0.12, need_windows=3, freq_tolerance=0.04, peak_ratio=0.5, min_rms=300.0,
                         refractory_sec=2.0).items():
            self.declare_parameter(k, v)
        self.det, self.rate, self.speaking = None, None, False
        self.pub = self.create_publisher(String, 'audio/events', 10)
        self.create_subscription(AudioChunk, 'audio/raw', self._on_audio, qos_profile_sensor_data)
        self.create_subscription(TtsStatus, 'tts/status', lambda m: setattr(
            self, 'speaking', m.state == TtsStatus.SPEAKING), 10)

    def _on_audio(self, m):
        if self.speaking or m.channels != 1:      # shortcut: mono only; a multi-channel mic needs a mixdown first
            return
        if self.det is None or self.rate != m.sample_rate:
            p = lambda k: self.get_parameter(k).value  # noqa: E731
            self.rate = m.sample_rate
            self.det = ToneDetector(m.sample_rate, p('window_sec'), p('need_windows'), p('freq_tolerance'),
                                    p('peak_ratio'), p('min_rms'), p('refractory_sec'))
        for _, freq in self.det.feed(np.frombuffer(bytes(m.data), dtype='<i2')):
            self.get_logger().info(f'tone detected: {freq:.0f} Hz (doorbell)')
            self.pub.publish(String(data=json.dumps({'kind': 'doorbell', 'freq': round(freq, 1)})))


def main(args=None):
    rclpy.init(args=args)
    node = SoundEventNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
