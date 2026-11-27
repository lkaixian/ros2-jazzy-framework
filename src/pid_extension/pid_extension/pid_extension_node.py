import math
import time

from bt_common.executor_node import (ExtensionExecutor, port_bool, port_float,
                                     port_str, run_executor)
from bt_common.obstacle import ScanGuard
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from person_detection_msgr.msg import PersonArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from pid_extension.follow import LostPolicy, Memory, select
from pid_extension.pid import MIN_STANDOFF_M, PID, range_speed


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PidExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('pid_extension')
        for k, v in dict(odom_topic='odom', persons_topic='persons',
                         cmd_vel_topic='pid_extension/cmd_vel', max_linear=0.15,
                         max_angular=0.6, stale_sec=0.5, lost_sec=1.0, rate_hz=20.0,
                         scan_topic='scan', stop_distance=0.40, guard_half_width_deg=35.0, scan_stale_sec=0.5,
                         scan_required=False, scan_yaw_offset=0.0, blocked_give_up_sec=3.0, avoid_turn=0.4,
                         hfov_deg=69.0, goto_sec=6.0, search_sec=12.0, reid_radius=1.2, turn_clearance=0.22, slow_ramp=0.3).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        ma, ml = p('max_angular'), p('max_linear')
        self.pids = {'heading': PID(1.5, 0.0, 0.1, ma), 'follow_turn': PID(0.8, 0.0, 0.05, ma),
                     'follow_range': PID(0.6, 0.0, 0.0, ml)}
        self.yaw, self.yaw_t, self.x, self.y = None, 0.0, 0.0, 0.0
        self.persons, self.persons_t = None, 0.0
        self.guard = ScanGuard(p('stop_distance'), p('guard_half_width_deg'), p('scan_stale_sec'),
                               p('scan_required'), p('scan_yaw_offset'))
        self.create_subscription(LaserScan, p('scan_topic'), self.guard.update, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom, 10)
        self.create_subscription(PersonArray, p('persons_topic'), self._on_persons, 10)
        self.pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.handlers = {'SetPidGains': self._set_gains, 'HoldHeading': self._hold,
                         'FollowTarget': self._follow}

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.x, self.y = m.pose.pose.position.x, m.pose.pose.position.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw_t = time.monotonic()

    def _on_persons(self, m):
        self.persons, self.persons_t = m, time.monotonic()

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def _stop(self):
        for _ in range(2):
            self._cmd(0.0, 0.0)

    def _set_gains(self, ports, ctl):
        name = port_str(ports, 'controller')
        if name not in self.pids:
            return False, f'unknown controller {name!r}; choose {sorted(self.pids)}'
        kp = port_float(ports, 'kp', lo=0.0, hi=50.0)
        ki = port_float(ports, 'ki', 0.0, lo=0.0, hi=50.0)
        kd = port_float(ports, 'kd', 0.0, lo=0.0, hi=50.0)
        self.pids[name].set_gains(kp, ki, kd)
        return True, f'{name}: kp={kp} ki={ki} kd={kd}'

    def _hold(self, ports, ctl):
        target = wrap(port_float(ports, 'yaw'))
        timeout = port_float(ports, 'timeout_ms', 10000.0, lo=100.0, hi=120000.0) / 1000.0
        pid, dt = self.pids['heading'], 1.0 / self.get_parameter('rate_hz').value
        pid.reset()
        end, settled = time.monotonic() + timeout, None
        try:
            while True:
                if ctl.cancelled:
                    return False, 'cancelled'
                if self.yaw is None or time.monotonic() - self.yaw_t > self.get_parameter('stale_sec').value:
                    return False, 'odometry missing or stale'
                err = wrap(target - self.yaw)
                if abs(err) < 0.05:
                    settled = settled or time.monotonic()
                    if time.monotonic() - settled >= 0.5:
                        return True, 'settled'
                else:
                    settled = None
                if time.monotonic() > end:
                    return False, 'timeout'
                self._cmd(0.0, pid.update(0.0, 0.0, dt, err=err))
                ctl.sleep(dt)
        finally:
            self._stop()

    def _pose(self):
        """(x, y, yaw) in the odom frame, or None without fresh odometry (then FollowTarget cannot look for a lost person)."""
        if self.yaw is None or time.monotonic() - self.yaw_t > self.get_parameter('stale_sec').value:
            return None
        return self.x, self.y, self.yaw

    def _pick(self, track_id, mem, lock):
        m = self.persons
        if m is None or time.monotonic() - self.persons_t > self.get_parameter('stale_sec').value:
            return None, None
        return select(m.persons, mem, self._pose() or (0.0, 0.0, 0.0), self.get_parameter('hfov_deg').value, max(1, m.image_width),
                      track_id, lock, self.get_parameter('reid_radius').value), m

    def _follow(self, ports, ctl):
        tid = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        target_h = port_float(ports, 'target_height', 0.6, lo=0.1, hi=0.95)
        approach = port_bool(ports, 'approach', False)
        # metric mode: only with approach and a depth-ranged person (distance_m > 0); the stand-off
        # is never below MIN_STANDOFF_M. Without depth the bbox-height proxy is used instead.
        target_d = (port_float(ports, 'distance', lo=MIN_STANDOFF_M, hi=8.0)
                    if ports.get('distance') else 0.0)
        search = port_bool(ports, 'search', True)            # a lost person: drive to where they were last seen, then sweep (needs odometry)
        keep = port_bool(ports, 'until_cancelled', False)    # follow on after arriving: only a cancel, a lost target or the timeout ends it
        lock = port_bool(ports, 'lock', False)               # stay with the first person seen (their track id), not whoever is biggest
        timeout = port_float(ports, 'timeout_ms', 10000.0, lo=100.0, hi=900000.0) / 1000.0
        turn, rng = self.pids['follow_turn'], self.pids['follow_range']
        turn.reset()
        rng.reset()
        dt, lost_sec = 1.0 / self.get_parameter('rate_hz').value, self.get_parameter('lost_sec').value
        end, last_seen, ok_since = time.monotonic() + timeout, time.monotonic(), None
        blocked_since = None
        mem, par = Memory(), self.get_parameter
        mem.track_id = tid
        lost = LostPolicy(par('goto_sec').value, par('search_sec').value, wait_sec=lost_sec)
        try:
            while True:
                if ctl.cancelled:
                    return False, 'cancelled'
                person, m = self._pick(tid, mem, lock)
                now = time.monotonic()
                if person is not None and lock and tid is None and person.track_id >= 0:
                    tid = person.track_id
                if person is None:
                    pose = self._pose()
                    if mem.seen_t is None or not search or pose is None:     # never saw anyone / no odometry: stand still, then give up
                        if now - last_seen > lost_sec:
                            return False, 'target lost'
                        self._cmd(0.0, 0.0)
                    else:
                        v, w, phase = lost.step(mem, pose, now, can_drive=self.guard.blocked(1.0) is None,
                                                can_turn=self.guard.too_close(par('turn_clearance').value) is None)
                        if phase == 'fail':
                            return False, 'target lost'
                        self._cmd(v, w)
                else:
                    last_seen = now
                    lost.reset()
                    W, H = max(1, m.image_width), max(1, m.image_height)
                    if tid is not None and person.track_id >= 0 and person.track_id != tid:
                        tid = person.track_id                     # the tracker renamed the person we follow: carry on with the new id
                    ex, _ = mem.update(person, self._pose() or (self.x, self.y, 0.0), now, par('hfov_deg').value, W)
                    h = (person.y_max - person.y_min) / H
                    w = -turn.update(0.0, 0.0, dt, err=ex)       # target right -> turn right (cw)
                    if approach and target_d > 0 and person.distance_m > 0:
                        v = range_speed(person.distance_m, target_d, 0.5,
                                        self.get_parameter('max_linear').value)
                        arrived = person.distance_m <= target_d + 0.1
                    else:
                        v = rng.update(target_h, h, dt) if approach and h < target_h else 0.0
                        arrived = h >= target_h - 0.03
                    v = max(0.0, v)
                    near = self.guard.blocked(v)          # never drive up to a person (or a wall) closer than stop_distance
                    v *= self.guard.slow_factor(v, par('slow_ramp').value)     # and ease off before that
                    if near is not None:
                        v = 0.0
                        if not arrived:                   # an obstacle, not the person we came for: turn toward the freer side
                            w = self.guard.freer_side() * self.get_parameter('avoid_turn').value or w
                        blocked_since = blocked_since or now
                        if now - blocked_since > self.get_parameter('blocked_give_up_sec').value:
                            return False, f'obstacle {near:.2f} m ahead: stopped'
                    else:
                        blocked_since = None
                    aligned = abs(ex) < 0.08 and (not approach or arrived)
                    ok_since = (ok_since or now) if aligned else None
                    if ok_since and now - ok_since >= 0.5 and not keep:
                        return True, 'aligned'
                    self._cmd(v, w)
                if now > end:
                    return False, 'timeout'
                ctl.sleep(dt)
        finally:
            self._stop()


def main(args=None):
    run_executor(PidExtension, args)


if __name__ == '__main__':
    main()
