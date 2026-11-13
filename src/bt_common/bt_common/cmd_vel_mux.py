"""Priority mux + watchdog + clamp in front of the robot's cmd_vel.

Inputs are listed highest priority first. The first input that is fresh wins. When every
input goes stale the mux publishes a zero Twist once, then stays silent.

Bumper/cliff events latch a stop: while latched the mux publishes zero Twist at the loop rate
and ignores every input. The latch clears only through the `safety_reset` service, which refuses
while a sensor is still active, and it discards all buffered commands so a reset never replays
an old command (it restores permission, not motion). This is a software gate: it is not a
hardware e-stop and is unverified on a real Kobuki.
"""
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from std_srvs.srv import Trigger


def clamp(x, lim):
    return max(-lim, min(lim, x))


def select(last, now, timeout):
    """last: list of (stamp or None, Twist) in priority order -> chosen index or None."""
    for i, (stamp, _) in enumerate(last):
        if stamp is not None and now - stamp <= timeout:
            return i
    return None


class StopLatch:
    """Latching stop. Active sensors are tracked so a reset is refused while any is still hit."""

    def __init__(self):
        self.latched = False
        self.reason = ''
        self.active = set()

    def event(self, key, hit, reason=''):
        """key: hashable sensor id, hit: True when pressed / at a cliff. Returns True on a new latch."""
        if hit:
            self.active.add(key)
            new = not self.latched
            self.latched = True
            if new or not self.reason:
                self.reason = reason or str(key)
            return new
        self.active.discard(key)
        return False

    def reset(self):
        """-> (ok, message). A hit sensor or an unavailable safety input blocks the reset."""
        if not self.latched:
            return True, 'not latched'
        if self.active:
            return False, f'still active: {sorted(map(str, self.active))}'
        self.latched = False
        self.reason = ''
        return True, 'reset'


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')
        self.declare_parameter('inputs', ['odom_extension/cmd_vel', 'pid_extension/cmd_vel',
                                          'nav2/cmd_vel'])
        self.declare_parameter('output', 'cmd_vel')
        self.declare_parameter('stamped', False)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('rate_hz', 20.0)
        # Kobuki bumper/cliff latch. Empty topic = that sensor is not watched.
        self.declare_parameter('safety_events_enabled', True)
        self.declare_parameter('bumper_topic', 'events/bumper')
        self.declare_parameter('bumper_type', 'kobuki_ros_interfaces/msg/BumperEvent')
        self.declare_parameter('cliff_topic', 'events/cliff')
        self.declare_parameter('cliff_type', 'kobuki_ros_interfaces/msg/CliffEvent')
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.stamped = p('stamped')
        self.last = []
        for i, topic in enumerate(p('inputs')):
            self.last.append((None, Twist()))
            self.create_subscription(Twist, topic, lambda m, i=i: self._on(i, m), 10)
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                         p('output'), 10)
        self.active = None
        self.stop = StopLatch()
        self.state_pub = self.create_publisher(String, 'safety/stop_state', 10)
        self.create_service(Trigger, 'safety_reset', self._reset)
        if p('safety_events_enabled'):
            # hit-state field and value that means "stop" for each Kobuki event message
            self._watch('bumper', p('bumper_topic'), p('bumper_type'), 'bumper', 'state')
            self._watch('cliff', p('cliff_topic'), p('cliff_type'), 'sensor', 'state')
        else:
            self.get_logger().warn('safety_events_enabled=false: bumper/cliff do NOT stop the robot')
        self.create_timer(1.0 / p('rate_hz'), self._tick)

    def _watch(self, kind, topic, type_name, id_field, state_field):
        if not topic:
            return
        try:
            msg_type = get_message(type_name)
        except (ImportError, AttributeError, ValueError) as e:
            # fail closed: a safety input we cannot read keeps the robot stopped
            self.get_logger().error(f'{kind}: cannot load {type_name} ({e}); staying stopped')
            self.stop.event(f'{kind}-unavailable', True, f'{kind} message type unavailable')
            return
        self.create_subscription(
            msg_type, topic,
            lambda m, k=kind, i=id_field, st=state_field: self._on_event(
                k, getattr(m, i), getattr(m, st) == 1), 10)

    def _on_event(self, kind, ident, hit):
        if self.stop.event((kind, ident), hit, f'{kind} {ident}') and hit:
            self.get_logger().error(f'STOP latched: {kind} {ident}')
            self._drop_commands()
        self._publish_state()

    def _drop_commands(self):
        self.last = [(None, Twist()) for _ in self.last]
        self.active = None

    def _reset(self, _req, resp):
        resp.success, resp.message = self.stop.reset()
        if resp.success:
            self._drop_commands()
            self.get_logger().warn('stop latch reset: waiting for fresh commands')
        else:
            self.get_logger().warn(f'reset refused: {resp.message}')
        self._publish_state()
        return resp

    def _publish_state(self):
        m = String()
        m.data = f'latched: {self.stop.reason}' if self.stop.latched else 'armed'
        self.state_pub.publish(m)

    def _on(self, i, msg):
        self.last[i] = (time.monotonic(), msg)

    def _out(self, tw):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist = tw
            self.pub.publish(m)
        else:
            self.pub.publish(tw)

    def _tick(self):
        if self.stop.latched:
            self._out(Twist())
            self.active = None
            return
        idx = select(self.last, time.monotonic(), self.get_parameter('timeout_sec').value)
        if idx is None:
            if self.active is not None:     # fresh -> stale: stop once
                self._out(Twist())
                self.active = None
            return
        if idx != self.active:
            self.get_logger().info(f'cmd_vel source: input {idx}')
        self.active = idx
        src = self.last[idx][1]
        out = Twist()
        ml, ma = self.get_parameter('max_linear').value, self.get_parameter('max_angular').value
        out.linear.x = clamp(src.linear.x, ml)
        out.linear.y = clamp(src.linear.y, ml)
        out.angular.z = clamp(src.angular.z, ma)
        self._out(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
