#!/usr/bmn/python3
"""teleop: drive the robot by keyboard, taking over from any running plan (manual override, B-07).

  ros2 run framework_bringup teleop
  w/s forward/back   a/d turn left/right   space stop   r reset the bumper/cliff stop   q quit   (hold a key: it repeats)
Publishes to `teleop/cmd_vel`, the FIRST input of cmd_vel_mux (highest priority), so it wins over the executor, PID and Nav2
as long as keys keep arriving; after the last key the robot ramps to a stop (about 0.5 s), then the mux falls back to
the next source. The first key also calls
bt_executor/halt so a plan does not drive off again the moment you let go. Speeds are further clamped by the mux.
A bumper or cliff hit latches a stop in the mux (every command ignored); release the sensor, then `r` calls its safety_reset
service (refused while the sensor is still pressed). Teleop prints when the mux latches or re-arms.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from bt_common.cmd_vel_mux import slew
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

KEYS = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0), ' ': (0.0, 0.0)}
LINEAR, ANGULAR = 0.2, 0.6      # m/s, rad/s before the mux clamp
ACCEL = (0.4, 1.2)              # m/s^2, rad/s^2: speed up and slow down at this rate
RATE = 20.0                     # Hz while moving
# A terminal has no key-up. After the first press comes the repeat delay (~0.5 s), then fast repeats; a key not seen
# again within the hold time counts as released.
HOLD_FIRST, HOLD_REPEAT, REPEAT_GAP = 0.55, 0.15, 0.2    # s

HELP = '\n'.join(ln.strip() for ln in __doc__.splitlines() if 'w/s' in ln)      # the key help line printed at start


def twist_for(key):
    """Key -> (linear m/s, angular rad/s), or None for a key that means nothing."""
    k = KEYS.get(key.lower())
    return None if k is None else (k[0] * LINEAR, k[1] * ANGULAR)


class Drive:
    """Keeps driving briefly after the last key and ramps to zero, so letting go is not a hard stop."""

    def __init__(self, accel=ACCEL):
        self.cur, self.target, self.accel = [0.0, 0.0], (0.0, 0.0), accel
        self.until = self.seen = 0.0
        self.key = None
        self.busy = False

    def press(self, now, key, cmd):
        repeating = key == self.key and now - self.seen < REPEAT_GAP
        self.key, self.seen = key, now
        self.target, self.until = cmd, now + (HOLD_REPEAT if repeating else HOLD_FIRST)
        if cmd == (0.0, 0.0):
            self.cur = [0.0, 0.0]       # space: stop now
        self.busy = True

    def tick(self, now, dt):
        """-> (linear, angular) to publish, or None when idle at zero (one zero is sent on the way)."""
        if now >= self.until:
            self.target = (0.0, 0.0)
        self.cur = [slew(c, t, a * dt) for c, t, a in zip(self.cur, self.target, self.accel)]
        busy = any(self.cur) or any(self.target)
        send = busy or self.busy
        self.busy = busy
        return tuple(self.cur) if send else None


def main(argv=None):
    if not sys.stdin.isatty():
        print('teleop needs a terminal', file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node('framework_teleop')
    pub = node.create_publisher(Twist, 'teleop/cmd_vel', 10)
    halt = node.create_client(Trigger, 'bt_executor/halt')
    reset = node.create_client(Trigger, 'safety_reset')       # cmd_vel_mux: clears the bumper/cliff stop latch
    node.create_subscription(String, 'safety/stop_state', lambda m: print(
        f'!! mux {m.data}  (release the sensor, then press r)' if m.data.startswith('latched') else f'mux {m.data}'),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    old = termios.tcgetattr(sys.stdin)
    halted = False
    drive, dt, last_pub = Drive(), 1.0 / RATE, 0.0
    print(HELP)
    try:
        tty.setcbreak(sys.stdin)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now - last_pub >= dt:
                last_pub = now
                out = drive.tick(now, dt)
                if out is not None:
                    t = Twist()
                    t.linear.x, t.angular.z = out
                    pub.publish(t)
            if not select.select([sys.stdin], [], [], 0.01)[0]:
                continue
            key = os.read(sys.stdin.fileno(), 1).decode(errors='replace')
            if key == 'q':
                break
            if key.lower() == 'r':
                if reset.service_is_ready():
                    reset.call_async(Trigger.Request()).add_done_callback(
                        lambda f: print('reset:', f.result().message if f.result() else 'no answer'))
                else:
                    print('safety_reset not available: is the framework (cmd_vel_mux) running?')
                continue
            cmd = twist_for(key)
            if cmd is None:
                continue
            if not halted and halt.service_is_ready():
                halt.call_async(Trigger.Request())
                halted = True
            drive.press(time.monotonic(), key.lower(), cmd)
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
