"""Priority mux + watchdog + clamp in front of the robot's cmd_vel.

Inputs are listed highest priority first. The first input that is fresh wins. When every
input goes stale the mux publishes a zero Twist once, then stays silent.
"""
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def clamp(x, lim):
    return max(-lim, min(lim, x))


def select(last, now, timeout):
    """last: list of (stamp or None, Twist) in priority order -> chosen index or None."""
    for i, (stamp, _) in enumerate(last):
        if stamp is not None and now - stamp <= timeout:
            return i
    return None


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
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.stamped = p('stamped')
        self.last = []
        for i, topic in enumerate(p('inputs')):
            self.last.append((None, Twist()))
            self.create_subscription(Twist, topic, lambda m, i=i: self._on(i, m), 10)
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                         p('output'), 10)
        self.active = None
        self.create_timer(1.0 / p('rate_hz'), self._tick)

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
