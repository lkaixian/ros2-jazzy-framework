#!/usr/bin/env python3
"""robot_mode_node + bt_executor mode_gate, real nodes, own DDS domain.
No mode node: run refused (fails closed). Mode node up and executor heard: READY, run accepted.
Stop latch: SAFE_STOP, run refused. Latch cleared: READY again. Run with the workspace sourced."""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

XML = '<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence><Sleep msec="500"/></Sequence></BehaviorTree></root>'
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def main():
    os.environ['ROS_DOMAIN_ID'] = '88'
    procs = [subprocess.Popen(['ros2', 'run', 'bt_executor', 'bt_executor_node', '--ros-args', '-p', 'mode_gate:=true'], start_new_session=True)]
    fails = []
    try:
        rclpy.init()
        n = rclpy.create_node('mode_test')
        plan = n.create_publisher(BtPlan, 'bt/plan', 10)
        stop = n.create_publisher(String, 'safety/stop_state', 10)
        cli = n.create_client(Trigger, 'bt_executor/run')
        modes = []
        n.create_subscription(String, 'robot/mode', lambda m: modes.append(json.loads(m.data)), LATCHED)

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)

        def run():
            f = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=5)
            return f.result()

        def check(name, ok):
            print(('ok   ' if ok else 'FAIL ') + name)
            if not ok:
                fails.append(name)

        end = time.monotonic() + 30
        while time.monotonic() < end and not (cli.service_is_ready() and plan.get_subscription_count()):
            spin(0.2)
        plan.publish(BtPlan(task='t', xml=XML, valid=True))
        spin(0.5)
        r = run()
        check('no robot/mode: run refused', not r.success and 'no fresh robot/mode' in r.message)

        procs.append(subprocess.Popen(['ros2', 'run', 'bt_orchestrator', 'robot_mode_node', '--ros-args', '-p', "required:=['executor']"],
                                      start_new_session=True))
        end = time.monotonic() + 30
        while time.monotonic() < end and not (modes and modes[-1]['mode'] == 'ready'):
            spin(0.2)
        check('executor heard: mode ready, run_ok', bool(modes) and modes[-1]['mode'] == 'ready' and modes[-1]['run_ok'])
        r = run()
        check('ready: run accepted', r.success)
        spin(1.2)

        stop.publish(String(data='latched: bumper 1'))
        spin(1.5)
        check('stop latch: mode safe_stop', modes[-1]['mode'] == 'safe_stop' and 'stop latch: bumper 1' in modes[-1]['reasons'])
        r = run()
        check('safe_stop: run refused', not r.success and 'safe_stop' in r.message)

        stop.publish(String(data='armed'))
        spin(1.5)
        check('latch cleared: ready again', modes[-1]['mode'] == 'ready')
        check('ready again: run accepted', run().success)
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGTERM)
        rclpy.try_shutdown()
    print('PASS' if not fails else f'FAIL: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
