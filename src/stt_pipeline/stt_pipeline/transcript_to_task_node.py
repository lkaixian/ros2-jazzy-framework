import time

import rclpy
from audio_msgr.msg import Transcript
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from stt_pipeline.vocab import place_phrases, snap
from stt_pipeline.wake import StartGate, WakeGate


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
        # a competition task that starts only when someone says the start phrase: the plan is published as a task spec (no LLM), and while
        # it is set no other speech becomes a task (the answers a running task listens for are read by its own nodes)
        self.declare_parameter('start_phrase', '')       # comma separated, e.g. "start the task,start task"; '' = off
        self.declare_parameter('start_spec', '')         # the JSON task spec published on bt/task_spec when it is heard
        self.declare_parameter('start_cooldown_sec', 30.0)
        phrases = [p for p in self.get_parameter('start_phrase').value.split(',') if p.strip()]
        self.start = StartGate(phrases) if phrases and self.get_parameter('start_spec').value else None
        self.started_at = float('-inf')
        self.spec_pub = self.create_publisher(String, 'bt/task_spec', 10)
        self.pub = self.create_publisher(String, 'bt/task', 10)
        self.create_subscription(Transcript, 'stt/transcript', self._on, 10)
        self.capturing = False    # set by WaitForTranscript: the next answer is data, not a task
        self.create_subscription(Bool, 'stt/capture', lambda m: setattr(self, 'capturing', m.data),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on(self, m):
        if not m.is_final or self.capturing:
            return
        if self.start is not None:
            if self.start.heard(m.text) and time.monotonic() - self.started_at >= self.get_parameter('start_cooldown_sec').value:
                self.started_at = time.monotonic()
                self.get_logger().info(f'start signal heard ({m.text!r}): starting the task')
                self.spec_pub.publish(String(data=self.get_parameter('start_spec').value))
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
