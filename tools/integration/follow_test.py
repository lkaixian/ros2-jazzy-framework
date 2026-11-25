#!/usr/bin/env python3
"""FollowTarget of pid_extension: arrival ends it by default, `until_cancelled` keeps following until cancelled, `lock` stays with the first person,
and an obstacle ahead (not the person) makes it turn toward the freer side. Fake depth-ranged persons + fake scan, isolated DDS domain
(FOLLOW_TEST_DOMAIN, default 97, localhost only). Exit 0 only if every check passed."""
import math
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('FOLLOW_TEST_DOMAIN', '97')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from person_detection_msgr.msg import Person, PersonArray  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

N = 360


def scan(front=3.0, left=3.0, right=3.0):
    s = LaserScan()
    s.angle_min, s.angle_increment, s.range_min, s.range_max = -math.pi, 2 * math.pi / N, 0.05, 12.0
    s.ranges = [3.0] * N
    for i in range(N):
        a = s.angle_min + i * s.angle_increment
        if abs(a) < 0.3:
            s.ranges[i] = front
        elif 0.5 < a < 1.3:
            s.ranges[i] = left
        elif -1.3 < a < -0.5:
            s.ranges[i] = right
    return s


def person(tid, cx, dist, h=200.0):
    p = Person()
    p.track_id, p.confidence, p.distance_m = tid, 0.9, float(dist)
    p.x_min, p.x_max, p.y_min, p.y_max = cx - h / 4, cx + h / 4, 240 - h / 2, 240 + h / 2
    return p


def main():
    proc = subprocess.Popen(['ros2', 'run', 'pid_extension', 'pid_extension_node'], start_new_session=True)
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('follow_probe')
        world = {'persons': [person(1, 320, 1.05)], 'scan': scan()}
        cmds = []
        pers_pub = n.create_publisher(PersonArray, 'persons', 10)
        scan_pub = n.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        n.create_subscription(Twist, 'pid_extension/cmd_vel', lambda m: cmds.append((time.monotonic(), m.linear.x, m.angular.z)), 10)

        def tick():
            a = PersonArray()
            a.image_width, a.image_height = 640, 480
            a.persons = world['persons']
            pers_pub.publish(a)
            scan_pub.publish(world['scan'])
        n.create_timer(0.05, tick)
        ac = ActionClient(n, ExecuteNode, 'pid_extension/execute')
        assert ac.wait_for_server(timeout_sec=15), 'pid_extension/execute not up'

        def spin(sec):
            t0 = time.monotonic()
            while time.monotonic() - t0 < sec:
                rclpy.spin_once(n, timeout_sec=0.05)

        def start(**ports):
            ports = {'approach': 'true', 'distance': '1.0', 'timeout_ms': '15000', **ports}
            fut = ac.send_goal_async(ExecuteNode.Goal(node_id='FollowTarget', port_names=list(ports), port_values=[str(v) for v in ports.values()]))
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
            gh = fut.result()
            return gh, gh.get_result_async()

        def stop(gh, rfut):
            gh.cancel_goal_async()
            t0 = time.monotonic()
            while not rfut.done() and time.monotonic() - t0 < 3:
                rclpy.spin_once(n, timeout_sec=0.05)
            return rfut.result().result if rfut.done() else None

        def mean(vals):
            return sum(vals) / len(vals) if vals else 0.0

        spin(1.5)
        # 1. default: close and aligned -> SUCCESS
        gh, rf = start()
        spin(3.0)
        r = rf.result().result if rf.done() else None
        res['default: arrived and aligned -> success'] = bool(r) and r.success
        if not rf.done():
            stop(gh, rf)
        # 2. until_cancelled: the same situation keeps running; cancel ends it
        gh, rf = start(until_cancelled='true')
        spin(3.0)
        res['until_cancelled: still following after arriving'] = not rf.done()
        r = stop(gh, rf)
        res['until_cancelled: cancel ends it'] = bool(r) and not r.success and 'cancel' in r.message
        # 3. lock: first person on the left stays the target when a bigger one shows up on the right
        for lock, want_sign in (('true', 1), ('false', -1)):
            world['persons'] = [person(1, 160, 1.5, h=150)]
            gh, rf = start(until_cancelled='true', lock=lock)
            spin(1.5)
            world['persons'] = [person(1, 160, 1.5, h=150), person(2, 480, 1.5, h=260)]
            spin(0.5)
            cmds.clear()
            spin(1.5)
            w = mean([c[2] for c in cmds])
            res[f'lock={lock}: turns {"toward the first person (left)" if want_sign > 0 else "toward the bigger person (right)"}'] = w * want_sign > 0.05
            stop(gh, rf)
        # 4. obstacle ahead, person far away: turn toward the freer side, do not drive
        for left, right, want_sign in ((3.0, 0.5, 1), (0.5, 3.0, -1)):
            world['persons'] = [person(1, 320, 2.5)]
            world['scan'] = scan(front=0.3, left=left, right=right)
            spin(0.4)                        # the node must hold the new scan before it starts
            gh, rf = start(until_cancelled='true')
            cmds.clear()
            spin(1.5)
            w, v = mean([c[2] for c in cmds]), max([c[1] for c in cmds] or [0.0])
            res[f'obstacle ahead, freer side {"left" if want_sign > 0 else "right"}: turns that way, no forward drive'] = w * want_sign > 0.1 and v < 0.01
            stop(gh, rf)
            world['scan'] = scan()
        # 5. the person (not an obstacle) is what blocks: arrived -> no spinning away
        world['persons'] = [person(1, 320, 0.45)]
        world['scan'] = scan(front=0.3, left=3.0, right=0.5)
        spin(0.4)
        gh, rf = start(until_cancelled='true', distance='0.5')
        cmds.clear()
        spin(1.5)
        res['person right ahead (arrived): stays put, does not turn away'] = abs(mean([c[2] for c in cmds])) < 0.1
        stop(gh, rf)
    finally:
        os.killpg(os.getpgid(proc.pid), 15)
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
