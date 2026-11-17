import json
import time

import rclpy
from audio_msgr.srv import Speak
from face_recognition_msgr.msg import FaceArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FREE = ('idle', 'done', 'failed', 'halted')      # task/state values in which the robot may start something new
SPEC = {'goal': 'register the unknown person', 'steps': [{'macro': 'register_person'}]}


class UnknownTracker:
    """Fires once when the biggest face stays 'unknown' for `hold` s (gaps up to `gap` s are forgiven), then
    stays quiet for `cooldown` s so a person who declines is not nagged."""

    def __init__(self, hold, gap, cooldown):
        self.hold, self.gap, self.cooldown = hold, gap, cooldown
        self.since = self.last = None
        self.quiet_until = 0.0

    def update(self, now, unknown, free):
        """unknown: True/False/None (no usable face this frame). Returns True when to ask."""
        if unknown:
            if self.since is None or now - self.last > self.gap:      # first sighting, or back after a long gap
                self.since = now
            self.last = now
        elif self.last is None or now - self.last > self.gap:     # a known face, or nobody, for longer than a blink
            self.since = self.last = None
        if self.since is not None and now - self.since >= self.hold and free and now >= self.quiet_until:
            self.since = self.last = None
            self.quiet_until = now + self.cooldown
            return True
        return False


class AutoRegisterNode(Node):
    """An unknown face in view for a few seconds -> submit the `register_person` macro (ask the name, enrol, welcome)."""

    def __init__(self):
        super().__init__('auto_register_node')
        for k, v in dict(enabled=True, unknown_sec=2.5, gap_sec=0.6, cooldown_sec=60.0, min_conf=0.6,
                         min_width_px=60.0, startup_grace_sec=10.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.tracker = UnknownTracker(p('unknown_sec'), p('gap_sec'), p('cooldown_sec'))
        self.task, self.t0 = 'idle', time.monotonic()
        self.speak = self.create_client(Speak, 'tts/speak')     # the question needs a voice
        self.pub = self.create_publisher(String, 'bt/task_spec', 10)
        self.create_subscription(String, 'task/state', self._on_state, TL)
        self.create_subscription(FaceArray, 'faces', self._on_faces, 10)
        self.get_logger().info(f'auto-register: unknown face for {p("unknown_sec")} s -> register_person'
                               + ('' if p('enabled') else ' (disabled)'))

    def _on_state(self, m):
        try:
            self.task = json.loads(m.data).get('state', 'idle')
        except ValueError:
            pass

    def _on_faces(self, m):
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        if not p('enabled'):
            return
        big = max((f for f in m.faces if f.confidence >= p('min_conf') and f.x_max - f.x_min >= p('min_width_px')),
                  key=lambda f: (f.x_max - f.x_min) * (f.y_max - f.y_min), default=None)
        unknown = None if big is None else big.identity == 'unknown'
        # not while the system is still starting (grace period) or the voice service is missing
        ready = time.monotonic() - self.t0 >= p('startup_grace_sec') and self.speak.service_is_ready()
        if self.tracker.update(time.monotonic(), unknown, ready and self.task in FREE):
            self.get_logger().info('unknown face in view: asking who they are')
            self.pub.publish(String(data=json.dumps(SPEC)))


def main(args=None):
    rclpy.init(args=args)
    node = AutoRegisterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
