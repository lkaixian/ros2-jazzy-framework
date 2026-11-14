#!/usr/bin/python3
"""teleop: drive the robot by keyboard, taking over from any running plan (manual override, B-07).

  ros2 run framework_bringup teleop
  w/s forward/back   a/d turn left/right   space stop   q quit   (hold a key: it repeats)
Publishes to `teleop/cmd_vel`, the FIRST input of cmd_vel_mux (highest priority), so it wins over the executor, PID and Nav2
as long as keys keep arriving; 0.5 s after the last key the mux falls back to the next source. The first key also calls
bt_executor/halt so a plan does not drive off again the moment you let go. Speeds are further clamped by the mux.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

KEYS = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0), ' ': (0.0, 0.0)}
LINEAR, ANGULAR = 0.2, 0.6      # m/s, rad/s before the mux clamp


def twist_for(key):
    """Key -> (linear m/s, angular rad/s), or None for a key that means nothing."""
    k = KEYS.get(key.lower())
    return None if k is None else (k[0] * LINEAR, k[1] * ANGULAR)


def main(argv=None):
    if not sys.stdin.isatty():
        print('teleop needs a terminal', file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node('framework_teleop')
    pub = node.create_publisher(Twist, 'teleop/cmd_vel', 10)
    halt = node.create_client(Trigger, 'bt_executor/halt')
    old = termios.tcgetattr(sys.stdin)
    halted, last = False, 0.0
    print(__doc__.split('\n\n')[0].split('\n', 1)[1].strip())
    try:
        tty.setcbreak(sys.stdin)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors='replace')
            if key == 'q':
                break
            cmd = twist_for(key)
            if cmd is None:
                continue
            if not halted and halt.service_is_ready():
                halt.call_async(Trigger.Request())
                halted = True
            t = Twist()
            t.linear.x, t.angular.z = cmd
            pub.publish(t)
            last = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())      # explicit zero; the mux watchdog would also stop within 0.5 s
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
