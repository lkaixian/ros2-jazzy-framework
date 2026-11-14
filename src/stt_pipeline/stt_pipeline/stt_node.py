import queue
import threading

import rclpy
from audio_msgr.msg import AudioChunk, Transcript
from rclpy.node import Node

from stt_pipeline import engines
from stt_pipeline.vocab import hint_prompt, place_phrases


class SttNode(Node):
    """stt/utterance -> stt/transcript. Inference runs on a worker thread."""

    def __init__(self):
        super().__init__('stt_node')
        for k, v in dict(engine='faster_whisper', model='base.en', language='',
                         device='cpu', compute_type='int8', min_confidence=0.0,
                         hint_places=True).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        prompt = hint_prompt(place_phrases()) if p('hint_places') else ''     # place names bias Whisper towards them
        # A missing/broken engine must stop the node, not silently degrade.
        self.engine = engines.create(p('engine'), model=p('model'), language=p('language'),
                                     device=p('device'), compute_type=p('compute_type'),
                                     initial_prompt=prompt)
        self.get_logger().info(f'STT engine: {self.engine.name}')
        self.q = queue.Queue(maxsize=8)
        self.pub = self.create_publisher(Transcript, 'stt/transcript', 10)
        self.create_subscription(AudioChunk, 'stt/utterance', self._on_utt, 10)
        threading.Thread(target=self._worker, daemon=True).start()

    def _on_utt(self, m):
        try:
            self.q.put_nowait(m)
        except queue.Full:
            self.get_logger().warn('STT backlog full; dropping utterance')

    def _worker(self):
        while rclpy.ok():
            m = self.q.get()
            try:
                text, lang, conf = self.engine.transcribe(bytes(m.data), m.sample_rate)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'transcribe failed: {type(e).__name__}: {e}')
                continue
            if not text.strip() or (0 <= conf < self.get_parameter('min_confidence').value):
                continue
            t = Transcript(text=text.strip(), language=lang, confidence=float(conf),
                           is_final=True, engine=self.engine.name,
                           duration_sec=len(m.data) / (2.0 * m.sample_rate))
            t.header = m.header
            self.pub.publish(t)
            self.get_logger().info(f'"{t.text}" (conf {conf:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
