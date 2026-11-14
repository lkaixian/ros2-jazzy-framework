#!/usr/bin/python3
"""preflight: is the running stack safe to start a task? One PASS/WARN/FAIL line per check.

  ros2 run framework_bringup preflight [--no-robot]     # exit 1 if any FAIL
Checks: LLM_API_KEY, SROS2, mux + executor alive, executor in dry-run, LLM endpoints, /scan flowing,
TF odom->base. --no-robot skips the lidar and TF checks (sim-less dry runs).
shortcut: topic/frame names (/scan, odom, base) are the Kobuki + RPLIDAR defaults, not read from params.
"""
import argparse
import json
import os
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)


def llm_verdict(status):
    """(level, text) for a parsed llm/status message (None = nothing received)."""
    if status is None:
        return 'FAIL', 'no llm/status (llm_bridge not running?)'
    up = [e for e in status.get('endpoints', []) if e.get('available')]
    if status.get('state') == 'down' or not up:
        return 'FAIL', f"LLM {status.get('state', '?')}: no endpoint up"
    return ('WARN' if status.get('state') == 'degraded' else 'PASS',
            f"LLM {status.get('state')}, {len(up)}/{len(status['endpoints'])} endpoints up")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--no-robot', action='store_true')
    a = ap.parse_args(argv)
    rclpy.init()
    n = rclpy.create_node('framework_preflight')
    got = {}
    n.create_subscription(String, 'llm/status', lambda m: got.update(llm=json.loads(m.data)), TL)
    n.create_subscription(LaserScan, '/scan', lambda m: got.update(scan=True), qos_profile_sensor_data)
    tf = Buffer()
    TransformListener(tf, n)
    cli = n.create_client(GetParameters, '/bt_executor_node/get_parameters')
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    res = []
    add = lambda lvl, txt: res.append((lvl, txt))  # noqa: E731
    add('PASS' if os.environ.get('LLM_API_KEY') else 'WARN',
        'LLM_API_KEY set' if os.environ.get('LLM_API_KEY') else 'LLM_API_KEY not set (ok only for a local, keyless server)')
    sec = os.environ.get('ROS_SECURITY_ENABLE', '').lower() == 'true'
    add('PASS' if sec else 'WARN', 'SROS2 enabled' if sec else 'DDS unsecured (no workspace sourced)')
    online = {x for x, _ in n.get_node_names_and_namespaces()}
    for name in ('cmd_vel_mux_node', 'bt_executor_node'):
        add('PASS' if name in online else 'FAIL', f'{name} ' + ('alive' if name in online else 'NOT running'))
    dry = None
    if cli.wait_for_service(timeout_sec=1.0):
        f = cli.call_async(GetParameters.Request(names=['dry_run']))
        rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
        dry = f.result().values[0].bool_value if f.done() else None
    add('PASS' if dry else 'WARN',
        'executor in dry-run' if dry else 'executor NOT in dry-run (or unreachable): plans will move hardware')
    add(*llm_verdict(got.get('llm')))
    if not a.no_robot:
        add('PASS' if got.get('scan') else 'FAIL', '/scan publishing' if got.get('scan') else 'no /scan in 3 s')
        ok = tf.can_transform('odom', 'base', rclpy.time.Time())
        add('PASS' if ok else 'FAIL', 'TF odom -> base' if ok else 'no TF odom -> base')
    for lvl, txt in res:
        print(f'{lvl:<5} {txt}')
    n.destroy_node()
    rclpy.try_shutdown()
    return 1 if any(l == 'FAIL' for l, _ in res) else 0


if __name__ == '__main__':
    sys.exit(main())
