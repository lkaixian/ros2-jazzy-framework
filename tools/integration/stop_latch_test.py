#!/usr/bin/env python3
"""Sim check of the cmd_vel_mux bumper/cliff latch with fake Kobuki events (no hardware).

Needs the workspace sourced and kobuki_ros_interfaces installed. Starts its own mux on an
isolated DDS domain (STOP_LATCH_DOMAIN, default 77, localhost only; the shell's own
ROS_DOMAIN_ID is deliberately ignored so it never talks to a robot or other nodes) and stops
it on exit.
Exit code 0 only if every check passed.
"""
import os
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('STOP_LATCH_DOMAIN', '77')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from kobuki_ros_interfaces.msg import BumperEvent, CliffEvent  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


def main():
    # own process group: `ros2 run` leaves its child alive on SIGINT, which kept pipes open (CI hang)
    proc = subprocess.Popen(['ros2', 'run', 'bt_common', 'cmd_vel_mux_node'],
                            start_new_session=True)
    n = None
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('stop_latch_probe')
        out, state = [], []
        n.create_subscription(Twist, 'cmd_vel', lambda m: out.append((time.monotonic(), m.linear.x)), 10)
        n.create_subscription(String, 'safety/stop_state', lambda m: state.append(m.data), 10)
        cmd = n.create_publisher(Twist, 'odom_extension/cmd_vel', 10)
        bump = n.create_publisher(BumperEvent, 'events/bumper', 10)
        cliff = n.create_publisher(CliffEvent, 'events/cliff', 10)
        cli = n.create_client(Trigger, 'safety_reset')

        def spin(sec, drive=False):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                if drive:
                    t = Twist()
                    t.linear.x = 0.2
                    cmd.publish(t)
                rclpy.spin_once(n, timeout_sec=0.02)

        def reset():
            f = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=3)
            return f.result() is not None and f.result().success

        if not cli.wait_for_service(timeout_sec=15):
            print('FAIL mux safety_reset service did not appear')
            return 1
        spin(1.0, drive=True)
        res['moving before event'] = bool(out) and out[-1][1] > 0.1
        e = BumperEvent()
        e.bumper, e.state = 1, 1
        t0 = time.monotonic()
        bump.publish(e)
        before = len(out)
        spin(0.6, drive=True)                 # the producer keeps commanding
        after = [v for (t, v) in out[before:] if t > t0 + 0.15]
        res['zero while bumper pressed, producer still driving'] = bool(after) and all(v == 0.0 for v in after)
        res['reset refused while pressed'] = reset() is False
        e.state = 0
        bump.publish(e)
        spin(0.3, drive=True)
        res['release alone keeps the latch'] = out[-1][1] == 0.0
        c = CliffEvent()
        c.sensor, c.state = 0, 1
        cliff.publish(c)
        spin(0.3)
        res['reset refused while cliff active'] = reset() is False
        c.state = 0
        cliff.publish(c)
        spin(1.1)                             # producer silent
        mark = len(out)
        res['reset ok when all clear'] = reset() is True
        spin(0.6)
        res['no replay after reset'] = all(v == 0.0 for (_, v) in out[mark:])
        spin(0.5, drive=True)
        res['fresh command accepted after reset'] = out[-1][1] > 0.1
        res['state topic reports armed and latched'] = (
            'armed' in state and any(s.startswith('latched') for s in state))
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(2)
        if n is not None:
            n.destroy_node()
        rclpy.try_shutdown()
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
