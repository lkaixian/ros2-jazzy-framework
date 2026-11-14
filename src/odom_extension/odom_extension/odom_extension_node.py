import math
import time

from bt_common.executor_node import (ExtensionExecutor, port_float,
                                     run_executor)
from bt_common.obstacle import ScanGuard
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from odom_extension.controllers import DriveDistance, Limits, RotateBy, goto_step, wrap  # noqa: F401


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('odom_extension')
        for k, v in dict(odom_topic='odom', cmd_vel_topic='odom_extension/cmd_vel',
                         max_linear=0.2, max_angular=0.6, kv=0.8, kw=1.5,
                         default_timeout_sec=60.0, stale_sec=0.5, rate_hz=20.0,
                         scan_topic='scan', stop_distance=0.30, guard_half_width_deg=35.0,
                         scan_stale_sec=0.5, scan_required=False, scan_yaw_offset=0.0,
                         turn_clearance=0.22).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.lim = Limits(p('max_linear'), p('max_angular'), p('kv'), p('kw'))
        self.pose = None            # (x, y, yaw)
        self.frame = ''
        self.stamp = 0.0
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom, 10)
        self.guard = ScanGuard(p('stop_distance'), p('guard_half_width_deg'), p('scan_stale_sec'),
                               p('scan_required'), p('scan_yaw_offset'))
        self.create_subscription(LaserScan, p('scan_topic'), self.guard.update, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.handlers = {'NavigateTo': self._navigate, 'DriveDistance': self._drive,
                         'RotateBy': self._rotate, 'IsAtGoal': self._at_goal}

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q))
        self.frame, self.stamp = m.header.frame_id, time.monotonic()

    def _blocked(self, v):
        return self.guard.blocked(v)

    def _fresh(self):
        stale = self.get_parameter('stale_sec').value
        return self.pose if self.pose and time.monotonic() - self.stamp <= stale else None

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def _stop(self):
        for _ in range(2):
            self._cmd(0.0, 0.0)

    def _run(self, ctl, make_step, timeout):
        """make_step(start_pose) -> step(pose) -> (v, w, done)."""
        rate = self.get_parameter('rate_hz').value
        end = time.monotonic() + timeout
        step = None
        try:
            while True:
                if ctl.cancelled:
                    return False, 'cancelled'
                pose = self._fresh()
                if pose is None:
                    return False, 'odometry missing or stale'
                if step is None:
                    step = make_step(pose)
                v, w, done = step(pose)
                if done:
                    return True, 'done'
                if time.monotonic() > end:
                    return False, 'timeout'
                near = self._blocked(v)
                if near is not None:
                    return False, f'obstacle {near:.2f} m {"ahead" if v > 0 else "behind"}: stopped'
                if v == 0.0 and w != 0.0:
                    near = self.guard.too_close(self.get_parameter('turn_clearance').value)
                    if near is not None:
                        return False, f'obstacle {near:.2f} m from the robot: not turning'
                self._cmd(v, w)
                ctl.sleep(1.0 / rate)
        finally:
            self._stop()

    def _timeout(self):
        return self.get_parameter('default_timeout_sec').value

    def _navigate(self, ports, ctl):
        gx, gy = port_float(ports, 'x'), port_float(ports, 'y')
        gyaw = port_float(ports, 'yaw', default=None) if ports.get('yaw') else None
        tol = port_float(ports, 'tolerance', 0.05, lo=0.01, hi=2.0)
        speed = port_float(ports, 'speed', 0.0, lo=0.0) or None
        frame = ports.get('frame', '')
        if frame and frame not in ('odom', self.frame):
            return False, f"frame {frame!r} unsupported (odometry frame is {self.frame!r})"
        return self._run(ctl, lambda _p: (lambda pose: goto_step(
            pose, (gx, gy), gyaw, tol, self.lim, speed)), self._timeout())

    def _drive(self, ports, ctl):
        m = port_float(ports, 'meters', lo=-20.0, hi=20.0)
        speed = port_float(ports, 'speed', 0.0, lo=0.0) or None
        return self._run(ctl, lambda p0: DriveDistance(p0, m, self.lim, speed).step,
                         self._timeout())

    def _rotate(self, ports, ctl):
        r = port_float(ports, 'radians', lo=-12.6, hi=12.6)
        speed = port_float(ports, 'speed', 0.0, lo=0.0) or None
        return self._run(ctl, lambda p0: RotateBy(p0[2], r, self.lim, speed).step,
                         self._timeout())

    def _at_goal(self, ports, ctl):
        gx, gy = port_float(ports, 'x'), port_float(ports, 'y')
        tol = port_float(ports, 'tolerance', 0.1, lo=0.01)
        pose = self._fresh()
        if pose is None:
            return False, 'odometry missing or stale'
        d = math.hypot(gx - pose[0], gy - pose[1])
        return d <= tol, f'distance {d:.3f} m'


def main(args=None):
    run_executor(OdomExtension, args)


if __name__ == '__main__':
    main()
