#!/usr/bin/env python3
"""battery_guard_node halts a running plan on a critically low (fake) Kobuki battery and leaves a healthy one alone.

Live bt_executor (dry-run) runs a long Sleep plan; fake SensorState messages report 13.0 V (critical) for the first case and
15.5 V for the control. Isolated DDS domain (BATTERY_TEST_DOMAIN, default 106). Exit 0 only if both checks passed."""
import json
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BATTERY_TEST_DOMAIN', '106')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_pipeline_msgs.msg import BtPlan  # noqa: E402
from bt_slm.fallback import build_xml  # noqa: E402
from kobuki_ros_interfaces.msg import SensorState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


def run_case(n, raw, wait):
    """Start a 60 s plan, publish battery `raw` for `wait` s, return the final bt/status state seen (or 'running')."""
    status = []
    sub = n.create_subscription(String, 'bt/status', lambda m: status.append(json.loads(m.data)), 10)
    pub = n.create_publisher(BtPlan, 'bt/plan', 10)
    bat = n.create_publisher(SensorState, 'sensors/core', 10)
    cli = n.create_client(Trigger, 'bt_executor/run')
    spec = {'goal': 'long', 'steps': [{'node': 'Sleep', 'ports': {'msec': '60000'}}]}
    plan = BtPlan(task='t', valid=True, xml=build_xml(json.dumps(spec)))
    end, started = time.monotonic() + 20, False
    while time.monotonic() < end and not started:
        rclpy.spin_once(n, timeout_sec=0.2)
        if not any(s['state'] == 'plan_received' for s in status):
            pub.publish(plan)
        elif cli.wait_for_service(timeout_sec=1.0):
            f = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=5)
            started = f.done() and f.result().success
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait:
        bat.publish(SensorState(battery=raw, charger=0))
        rclpy.spin_once(n, timeout_sec=0.2)
        if any(s['state'] == 'finished' for s in status):
            break
    fin = [s for s in status if s['state'] == 'finished']
    n.destroy_subscription(sub)
    return fin[-1]['message'] if fin else 'running'


def main():
    procs = [subprocess.Popen(['ros2', 'run', 'bt_executor', 'bt_executor_node'], start_new_session=True),
             subprocess.Popen(['ros2', 'run', 'bt_common', 'battery_guard_node'], start_new_session=True)]
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('battery_probe')
        time.sleep(3)
        res['healthy battery (15.5 V): the plan keeps running'] = run_case(n, 155, 9) == 'running'
        # halt the control run so the executor is free for the next case
        c = n.create_client(Trigger, 'bt_executor/halt')
        c.wait_for_service(timeout_sec=3)
        f = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, f, timeout_sec=5)
        time.sleep(2)
        out = run_case(n, 130, 15)
        print('critical case ended with:', out)
        res['critical battery (13.0 V): the plan is halted'] = out == 'HALTED'
    finally:
        for p in procs:
            os.killpg(os.getpgid(p.pid), 15)
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
