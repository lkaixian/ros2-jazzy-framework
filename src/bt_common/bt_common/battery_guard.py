"""Halt the running plan when the Kobuki battery is critically low.

Reads SensorState.battery (0.1 V units) from `sensors/core`. Below `critical_v` for `debounce_sec` (and not charging) it calls
bt_executor/halt (repeated every `repeat_sec` while it stays low) and publishes {"level": "critical"} on safety/battery; below
`low_v` it only warns. Teleop and Nav2 are NOT stopped: this guards the autonomous plan, the operator decides about the rest.
shortcut: the 14.0 V / 13.2 V thresholds are the Kobuki driver's documented defaults for the 4-cell pack, not measured on this
robot; recheck them with the real pack before relying on this.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from kobuki_ros_interfaces.msg import SensorState
except ImportError:          # no Kobuki driver messages installed: the node idles
    SensorState = None


def battery_level(raw, charger=0, low=14.0, critical=13.2):
    """SensorState.battery (0.1 V) -> 'unknown' | 'ok' | 'low' | 'critical'. Charging never counts as critical."""
    if raw <= 0:
        return 'unknown'
    v = raw / 10.0
    if v < critical and not charger:
        return 'critical'
    return 'low' if v < low else 'ok'


class BatteryWatch:
    """Debounced decision: update(level, now) -> 'halt' (call halt now), 'warn' or None."""

    def __init__(self, debounce_sec=3.0, repeat_sec=5.0, warn_sec=60.0):
        self.debounce, self.repeat, self.warn_every = debounce_sec, repeat_sec, warn_sec
        self.since, self.last_halt, self.last_warn = None, -1e9, -1e9

    def update(self, level, now):
        if level != 'critical':
            self.since = None
            if level == 'low' and now - self.last_warn >= self.warn_every:
                self.last_warn = now
                return 'warn'
            return None
        self.since = self.since if self.since is not None else now
        if now - self.since >= self.debounce and now - self.last_halt >= self.repeat:
            self.last_halt = now
            return 'halt'
        return None


class BatteryGuard(Node):
    def __init__(self):
        super().__init__('battery_guard_node')
        for k, v in dict(topic='sensors/core', low_v=14.0, critical_v=13.2, debounce_sec=3.0, repeat_sec=5.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.low, self.crit = p('low_v'), p('critical_v')
        self.watch = BatteryWatch(p('debounce_sec'), p('repeat_sec'))
        self.pub = self.create_publisher(String, 'safety/battery', 10)
        self.halt = self.create_client(Trigger, 'bt_executor/halt')
        if SensorState is None:
            self.get_logger().warn('kobuki_ros_interfaces not installed: battery guard idle')
        else:
            self.create_subscription(SensorState, p('topic'), self._on, 5)

    def _on(self, m):
        level = battery_level(m.battery, m.charger, self.low, self.crit)
        act = self.watch.update(level, time.monotonic())
        if act == 'warn':
            self.get_logger().warn(f'base battery low: {m.battery / 10.0:.1f} V')
        elif act == 'halt':
            self.get_logger().error(f'base battery CRITICAL ({m.battery / 10.0:.1f} V): halting the plan')
            self.pub.publish(String(data=json.dumps({'level': 'critical', 'volts': m.battery / 10.0})))
            if self.halt.service_is_ready():
                self.halt.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = BatteryGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
