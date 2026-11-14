#!/usr/bin/env python3
"""Lidar guard of odom_extension (B-04): DriveDistance must stop for an obstacle in the travel direction and must not
stop for a clear scan. Fake odometry + fake scan, isolated DDS domain (GUARD_TEST_DOMAIN, default 96, localhost only).
Exit 0 only if every check passed."""
import math
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('GUARD_TEST_DOMAIN', '96')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

N = 360


def scan(front_m, rear_m=3.0, side_m=3.0):
    s = LaserScan(angle_min=-math.pi, angle_max=math.pi - 2 * math.pi / N, angle_increment=2 * math.pi / N,
                  range_min=0.05, range_max=12.0)
    s.ranges = [3.0] * N
    for i in range(N):
        a = s.angle_min + i * s.angle_increment
        if abs(a) < 0.3:
            s.ranges[i] = front_m
        elif abs(abs(a) - math.pi) < 0.3:
            s.ranges[i] = rear_m
        elif abs(a - math.pi / 2) < 0.3:
            s.ranges[i] = side_m
    return s


def main():
    proc = subprocess.Popen(['ros2', 'run', 'odom_extension', 'odom_extension_node'], start_new_session=True)
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('guard_probe')
        state = {'front': 3.0, 'rear': 3.0, 'side': 3.0}
        cmds_w = []
        odom_pub = n.create_publisher(Odometry, 'odom', 10)
        scan_pub = n.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        cmds = []
        n.create_subscription(Twist, 'odom_extension/cmd_vel', lambda m: (cmds.append(m.linear.x), cmds_w.append(m.angular.z)), 10)

        def tick():
            o = Odometry()
            o.header.frame_id = 'odom'
            o.pose.pose.orientation.w = 1.0
            odom_pub.publish(o)
            scan_pub.publish(scan(state['front'], state['rear'], state['side']))
        n.create_timer(0.05, tick)
        ac = ActionClient(n, ExecuteNode, 'odom_extension/execute')
        assert ac.wait_for_server(timeout_sec=15), 'odom_extension/execute not up'

        def run(node_id, port, value, wait, cancel_after):
            fut = ac.send_goal_async(ExecuteNode.Goal(node_id=node_id, port_names=[port], port_values=[str(value)]))
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
            gh = fut.result()
            rfut = gh.get_result_async()
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait and not rfut.done():
                rclpy.spin_once(n, timeout_sec=0.05)
                if cancel_after and time.monotonic() - t0 > cancel_after:
                    gh.cancel_goal_async()
                    cancel_after = None
            return rfut.result().result if rfut.done() else None

        def drive(meters, wait=6.0, cancel_after=None):
            return run('DriveDistance', 'meters', meters, wait, cancel_after)

        def turn(radians, wait=6.0, cancel_after=None):
            return run('RotateBy', 'radians', radians, wait, cancel_after)

        def settle(sec=0.6):                    # keep the fake odom/scan flowing so the node sees the new state
            t0 = time.monotonic()
            while time.monotonic() - t0 < sec:
                rclpy.spin_once(n, timeout_sec=0.05)
        settle(1.5)
        state.update(front=0.2)
        settle()
        r = drive(1.0)
        res['blocked ahead: DriveDistance refuses'] = bool(r) and not r.success and 'obstacle' in r.message and 'ahead' in r.message
        state.update(front=3.0)
        settle()
        cmds.clear()
        drive(1.0, wait=2.5, cancel_after=1.0)
        res['clear scan: it drives forward'] = any(c > 0.05 for c in cmds)
        state.update(front=0.2, rear=3.0)
        settle()
        cmds.clear()
        drive(-0.5, wait=2.5, cancel_after=1.0)
        res['obstacle only ahead: reversing is allowed'] = any(c < -0.05 for c in cmds)
        state.update(front=3.0, rear=0.2)
        settle()
        r = drive(-0.5)
        res['blocked behind: reversing refuses'] = bool(r) and not r.success and 'behind' in r.message
        state.update(front=3.0, rear=3.0, side=0.15)
        settle()
        r = turn(1.0)
        res['something at 15 cm beside the robot: RotateBy refuses'] = bool(r) and not r.success and 'not turning' in r.message
        state.update(side=3.0)
        settle()
        cmds_w.clear()
        turn(1.0, wait=2.5, cancel_after=1.0)
        res['clear all round: RotateBy turns'] = any(abs(c) > 0.05 for c in cmds_w)
    finally:
        os.killpg(os.getpgid(proc.pid), 15)
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
