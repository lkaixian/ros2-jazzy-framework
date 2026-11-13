import time

import rclpy
from audio_msgr.msg import AudioChunk, TtsStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from stt_pipeline.vad import Segmenter


class VadNode(Node):
    """audio/raw -> stt/utterance (one AudioChunk per detected utterance)."""

    def __init__(self):
        super().__init__('vad_node')
        for k, v in dict(threshold_rms=500.0, noise_ratio=3.0, min_speech_ms=150,
                         hangover_ms=600, preroll_ms=300, max_utterance_sec=15.0,
                         mute_while_speaking=True, mute_tail_ms=400).items():
            self.declare_parameter(k, v)
        self.seg = None
        self.fmt = None
        self.muted_until = 0.0
        self.speaking = False
        self.pub = self.create_publisher(AudioChunk, 'stt/utterance', 10)
        self.create_subscription(AudioChunk, 'audio/raw', self._on_audio,
                                 qos_profile_sensor_data)
        self.create_subscription(TtsStatus, 'tts/status', self._on_tts, 10)

    def _make(self, rate):
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        return Segmenter(rate, 30, p('threshold_rms'), p('noise_ratio'), p('min_speech_ms'),
                         p('hangover_ms'), p('preroll_ms'), p('max_utterance_sec'))

    def _on_tts(self, m):
        was = self.speaking
        self.speaking = m.state == TtsStatus.SPEAKING
        if self.speaking or was:
            tail = self.get_parameter('mute_tail_ms').value / 1000.0
            self.muted_until = time.monotonic() + (1e6 if self.speaking else tail)
            if self.seg:
                self.seg.reset()
                self.seg.buf = b''

    def _on_audio(self, m):
        if m.channels != 1:
            self.get_logger().error('vad_node needs mono audio', throttle_duration_sec=10.0)
            return
        if self.seg is None or self.fmt != m.sample_rate:
            self.fmt, self.seg = m.sample_rate, self._make(m.sample_rate)
        if (self.get_parameter('mute_while_speaking').value
                and time.monotonic() < self.muted_until):
            return
        for utt in self.seg.feed(bytes(m.data)):
            out = AudioChunk(sample_rate=m.sample_rate, channels=1, data=list(utt))
            out.header = m.header
            self.pub.publish(out)
            self.get_logger().info(f'utterance {len(utt) * 1000 // (2 * m.sample_rate)} ms')


def main(args=None):
    rclpy.init(args=args)
    node = VadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
