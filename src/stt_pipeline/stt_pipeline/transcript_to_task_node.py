import rclpy
from audio_msgr.msg import Transcript
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from stt_pipeline.vocab import place_phrases, snap
from stt_pipeline.wake import WakeGate


class TranscriptToTask(Node):
    """Final transcripts (optionally wake-word gated) -> bt/task for the LLM pipeline."""

    def __init__(self):
        super().__init__('transcript_to_task_node')
        self.declare_parameter('wake_word', '')
        self.declare_parameter('wake_window_sec', 8.0)
        self.declare_parameter('snap_places', True)     # 'for corner' -> 'far corner' before the task goes to the LLM
        self.places = place_phrases() if self.get_parameter('snap_places').value else []
        self.gate = WakeGate(self.get_parameter('wake_word').value,
                             self.get_parameter('wake_window_sec').value)
        self.pub = self.create_publisher(String, 'bt/task', 10)
        self.create_subscription(Transcript, 'stt/transcript', self._on, 10)
        self.capturing = False    # set by WaitForTranscript: the next answer is data, not a task
        self.create_subscription(Bool, 'stt/capture', lambda m: setattr(self, 'capturing', m.data),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on(self, m):
        if not m.is_final or self.capturing:
            return
        task = self.gate.task(m.text)
        if task and self.places:
            task = snap(task, self.places)
        if task:
            self.get_logger().info(f'task: {task}')
            self.pub.publish(String(data=task))


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptToTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
