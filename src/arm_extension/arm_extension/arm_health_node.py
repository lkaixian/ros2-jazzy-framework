"""Servo health for the arm: publishes `arm/health` (JSON, latched) and, only when allowed, reboots the servos to clear a hardware error.

arm_extension refuses live motion unless the latest health is ok. A reboot drops torque on every servo, so it is gated by
arm_control.health.RebootGate (arm parked at rest and still, fault persistent, rationed) and `auto_reboot` is off by default; the
`arm_health/reboot` service is the manual route, with the same gate."""
import json
import time

import rclpy
from arm_control.health import RebootGate, assess
from dynamixel_interfaces.msg import DynamixelState
from dynamixel_interfaces.srv import GetDataFromDxl, RebootDxl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ArmHealth(Node):
    def __init__(self):
        super().__init__('arm_health_node')
        for k, v in dict(auto_reboot=False, poll_sec=2.0, stale_sec=3.0, dxl_ids=[11, 12, 13, 14, 15],
                         rest_pose=[0.0, -1.85, 1.52, 0.45], ns='/dynamixel_hardware_interface').items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        ns = self.p('ns')
        self.state, self.state_t, self.q, self.q_prev, self.speed = None, 0.0, None, None, 0.0
        self.temps, self.volts, self.gate, self.rebooting_until = {}, {}, RebootGate(), 0.0
        self.pub = self.create_publisher(__import__('std_msgs.msg', fromlist=['String']).String, 'arm/health', LATCHED)
        self.get = self.create_client(GetDataFromDxl, f'{ns}/get_dxl_data')
        self.reboot_cli = self.create_client(RebootDxl, f'{ns}/reboot_dxl')
        self.create_subscription(DynamixelState, f'{ns}/dxl_state', self._on_state, 10)
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_service(Trigger, 'arm_health/reboot', self._manual)
        self.create_timer(self.p('poll_sec'), self._tick)

    def _on_state(self, m):
        self.state, self.state_t = m, time.monotonic()

    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            now = time.monotonic()
            q = {j: d[j] for j in JOINTS}
            if self.q is not None and now > self.q_t:
                self.speed = max(abs(q[j] - self.q[j]) for j in JOINTS) / (now - self.q_t)
            self.q, self.q_t = q, now

    def _poll(self):
        for i in self.p('dxl_ids'):
            for item, store in (('Present Temperature', self.temps), ('Present Input Voltage', self.volts)):
                if self.get.service_is_ready():
                    f = self.get.call_async(GetDataFromDxl.Request(id=i, item_name=item, timeout_sec=1.0))
                    f.add_done_callback(lambda fut, i=i, store=store, item=item: self._got(fut, i, store, item))

    def _got(self, fut, i, store, item):
        r = fut.result()
        if r is not None and r.result:
            store[i] = r.item_data / (10.0 if 'Voltage' in item else 1.0)       # voltage register: 0.1 V units

    def _tick(self):
        now = time.monotonic()
        self._poll()
        if self.state is None or now - self.state_t > self.p('stale_sec'):
            problems, rebootable = ['no dxl_state from the driver (driver down or stale)'], False
        else:
            problems, rebootable = assess(list(self.state.id), list(self.state.dxl_hw_state), self.state.comm_state, self.temps, self.volts)
        if now < self.rebooting_until:
            problems = problems + ['servos rebooting'] if 'servos rebooting' not in problems else problems
            rebootable = False
        rest = dict(zip(JOINTS, self.p('rest_pose')))
        ok_gate, why = self.gate.allow(now, problems, rebootable, self.q, rest, self.speed)
        if ok_gate and self.p('auto_reboot'):
            self._reboot()
        self.pub.publish(__import__('std_msgs.msg', fromlist=['String']).String(data=json.dumps(
            {'ok': not problems, 'problems': problems, 'rebootable': rebootable, 'reboot_gate': why,
             'auto_reboot': self.p('auto_reboot')})))

    def _reboot(self):
        if not self.reboot_cli.service_is_ready():
            self.get_logger().error('reboot_dxl service not available')
            return False
        self.get_logger().warn('rebooting the servos (arm is parked at rest)')
        self.reboot_cli.call_async(RebootDxl.Request())
        now = time.monotonic()
        self.gate.record(now)
        self.rebooting_until = now + 5.0
        return True

    def _manual(self, _req, res):
        now = time.monotonic()
        if self.state is None:
            res.success, res.message = False, 'no dxl_state yet'
            return res
        problems, rebootable = assess(list(self.state.id), list(self.state.dxl_hw_state), self.state.comm_state, self.temps, self.volts)
        self.gate.since = (now - 10.0) if problems else None            # a manual request does not wait out the persistence timer
        ok, why = self.gate.allow(now, problems, rebootable, self.q, dict(zip(JOINTS, self.p('rest_pose'))), self.speed)
        res.success = bool(ok and self._reboot())
        res.message = 'rebooting' if res.success else why
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ArmHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
