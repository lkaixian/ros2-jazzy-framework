"""robot/mode: one latched JSON that says whether the robot may run a plan (init, safety, teleop, mission budget). See mode.py."""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bt_orchestrator.mode import ModeFsm

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# init signal name -> (topic, message type, QoS); 'executor' is its halt service being ready (bt/status is only published at start)
SIGNALS = {'llm': ('llm/status', String, LATCHED), 'scan': ('/scan', LaserScan, qos_profile_sensor_data)}


class RobotModeNode(Node):
    def __init__(self):
        super().__init__('robot_mode_node')
        for k, v in dict(required=['executor', 'llm'], init_timeout_sec=60.0, teleop_hold_sec=1.0, battery_clear_sec=15.0,
                         mission_budget_sec=0.0, period_sec=0.5).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.fsm = ModeFsm(p('required'), p('init_timeout_sec'), p('teleop_hold_sec'), p('battery_clear_sec'), p('mission_budget_sec'))
        for name in p('required'):
            if name == 'executor':
                continue
            topic, typ, qos = SIGNALS[name]            # an unknown name stops the node: a typo must not silently skip a check
            self.create_subscription(typ, topic, lambda _m, n=name: self.fsm.heard(n), qos)
        self.create_subscription(String, 'safety/stop_state', lambda m: self.fsm.on_stop_state(m.data), 10)
        self.create_subscription(String, 'safety/battery', self._on_battery, 10)
        self.create_subscription(Twist, 'teleop/cmd_vel', lambda _m: self.fsm.on_teleop(self.now()), 10)
        self.create_subscription(String, 'task/state', self._on_task, LATCHED)
        self.pub = self.create_publisher(String, 'robot/mode', LATCHED)
        self.halt = self.create_client(Trigger, 'bt_executor/halt')
        self.create_service(Trigger, 'robot/reset', self._reset)
        self.last = None
        self.create_timer(p('period_sec'), self._tick)

    def now(self):
        return time.monotonic()

    def _on_battery(self, m):
        try:
            self.fsm.on_battery(json.loads(m.data).get('level', ''), self.now())
        except ValueError:
            pass

    def _on_task(self, m):
        try:
            self.fsm.on_task_state(json.loads(m.data).get('state', ''), self.now())
        except ValueError:
            pass

    def _reset(self, _req, resp):
        resp.success, resp.message = self.fsm.reset(self.now())
        return resp

    def _tick(self):
        now = self.now()
        if self.halt.service_is_ready():
            self.fsm.heard('executor')
        for _kind, why in self.fsm.tick(now):
            self.get_logger().error(f'halting the plan: {why}')
            if self.halt.service_is_ready():
                self.halt.call_async(Trigger.Request())
        snap = self.fsm.snapshot(now)
        key = (snap['mode'], tuple(snap['reasons']), tuple(snap['missing']))
        if key != self.last:
            self.get_logger().info(f"mode {snap['mode']} run_ok={snap['run_ok']} {snap['reasons'] or snap['missing'] or ''}")
        self.last = key
        self.pub.publish(String(data=json.dumps(snap)))      # every tick: the executor gate treats a silent node as unsafe


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
