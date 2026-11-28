import math
import time

from bt_common.executor_node import (ExtensionExecutor, port_bool, port_float,
                                     port_str, run_executor)
from bt_common.obstacle import ScanGuard
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from person_detection_msgr.msg import PersonArray
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from pid_extension.trail import Trail, trail_error
from pid_extension.follow import face_distance, face_visible, LostPolicy, Memory, select
from pid_extension.target import TargetModel
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
                         hfov_deg=69.0, goto_sec=6.0, search_sec=12.0, reid_radius=1.2, turn_clearance=0.22, slow_ramp=0.3, reid_sim=0.7, target_score=0.5,
                         cam_height=0.8, vfov_deg=42.0).items():
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
        self.operator_id = None      # who FollowTarget follows, published on follow/operator for pose_extension's WaitForStop
        self.operator_pub = self.create_publisher(Int32, 'follow/operator', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.handlers = {'SetPidGains': self._set_gains, 'HoldHeading': self._hold,
                         'FollowTarget': self._follow, 'BackUpForFace': self._back_for_face,
                         'CheckBehind': self._check_behind}

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

    def _pick(self, track_id, mem, lock, model=None):
        m = self.persons
        if m is None or time.monotonic() - self.persons_t > self.get_parameter('stale_sec').value:
            return None, None
        return select(m.persons, mem, self._pose() or (0.0, 0.0, 0.0), self.get_parameter('hfov_deg').value, max(1, m.image_width),
                      track_id, lock, self.get_parameter('reid_radius').value, reid_sim=self.get_parameter('reid_sim').value, model=model), m

    def _check_behind(self, ports, ctl):
        """Is something at least min_distance and at most max_distance (m) behind the robot? The lidar faces backwards on this robot, so a guest
        following it shows up there. A wall counts as well (the lidar cannot tell): use it to wait for a guest, not to prove one."""
        far = port_float(ports, 'max_distance', 2.5, lo=0.3, hi=6.0)
        near = port_float(ports, 'min_distance', 0.3, lo=0.1, hi=3.0)
        d = self.guard.clearance(-1.0)
        if d is None or d <= 0.0:
            return False, 'no lidar reading behind'
        return (True, f'something {d:.2f} m behind') if near <= d <= far else (False, f'nothing within {far:.1f} m behind (nearest {d:.2f} m)')

    def _back_for_face(self, ports, ctl):
        """The camera is low (cam_height), so a person close by shows only the torso: back away slowly, keeping them centred, until
        the nose and an eye are in the image. Speed follows the trig range at which the camera sees face_height."""
        tid = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        face_h = port_float(ports, 'face_height', 1.6, lo=1.0, hi=2.2)
        max_back = port_float(ports, 'max_back', 1.5, lo=0.1, hi=3.0)
        timeout = port_float(ports, 'timeout_ms', 15000.0, lo=1000.0, hi=120000.0) / 1000.0
        par, turn = self.get_parameter, self.pids['follow_turn']
        need = face_distance(par('cam_height').value, face_h, par('vfov_deg').value)
        turn.reset()
        dt, lost_sec = 1.0 / par('rate_hz').value, par('lost_sec').value
        mem, now = Memory(), time.monotonic()
        end, last_seen, backed, seen_face, blocked_since = now + timeout, now, 0.0, 0, None
        try:
            while True:
                if ctl.cancelled:
                    return False, 'cancelled'
                person, m = self._pick(tid, mem, False)
                now = time.monotonic()
                if person is None:
                    if now - last_seen > lost_sec:
                        return False, 'person lost'
                    self._cmd(0.0, 0.0)
                else:
                    last_seen = now
                    if person.track_id >= 0:
                        tid = person.track_id
                    seen_face = seen_face + 1 if face_visible(person, max(1, m.image_height)) else 0
                    if seen_face >= 3:
                        return True, f'face in view after backing {backed:.2f} m'
                    d = person.distance_m
                    if d > need + 0.5 or backed >= max_back:
                        return False, f'no face after backing {backed:.2f} m (person at {d:.1f} m, needs about {need:.1f} m): turned away?'
                    ex, _ = mem.update(person, self._pose() or (self.x, self.y, 0.0), now, par('hfov_deg').value, max(1, m.image_width))
                    v = -min(0.15, max(0.05, 0.5 * (need - d))) if d > 0 else -0.1     # slower the nearer to the right range
                    near = self.guard.blocked(v)
                    if near is not None:
                        v = 0.0
                        blocked_since = blocked_since or now
                        if now - blocked_since > par('blocked_give_up_sec').value:
                            return False, f'obstacle {near:.2f} m behind: stopped'
                    else:
                        blocked_since = None
                    backed += -v * dt
                    self._cmd(v, -turn.update(0.0, 0.0, dt, err=ex))
                if now > end:
                    return False, 'timeout'
                ctl.sleep(dt)
        finally:
            self._stop()

    def _follow(self, ports, ctl):
        tid = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        target_h = port_float(ports, 'target_height', 0.6, lo=0.1, hi=0.95)
        approach = port_bool(ports, 'approach', False)
        # metric mode: only with approach and a depth-ranged person (distance_m > 0); the stand-off
        # is never below MIN_STANDOFF_M. Without depth the bbox-height proxy is used instead.
        target_d = (port_float(ports, 'distance', lo=MIN_STANDOFF_M, hi=8.0)
                    if ports.get('distance') else 0.0)
        search = port_bool(ports, 'search', True)            # a lost person: drive to where they were last seen, then sweep (needs odometry)
        trail = Trail() if port_bool(ports, 'trail', False) else None      # steer along the path the person walked, not straight at them
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
        model = TargetModel(reid=par('target_score').value)
        mem.track_id = tid
        lost = LostPolicy(par('goto_sec').value, par('search_sec').value, wait_sec=lost_sec)
        try:
            while True:
                if ctl.cancelled:
                    return False, 'cancelled'
                person, m = self._pick(tid, mem, lock, model)
                now = time.monotonic()
                if person is not None and lock and tid is None and person.track_id >= 0:
                    tid = person.track_id
                if tid is not None and tid != self.operator_id:
                    self.operator_id = tid
                    self.operator_pub.publish(Int32(data=int(tid)))
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
                    if tid is not None and len(getattr(person, 'embedding', ())):      # learn the followed person against those around them
                        model.update(person.embedding, [q.embedding for q in m.persons if q is not person and len(q.embedding)])
                    W, H = max(1, m.image_width), max(1, m.image_height)
                    if tid is not None and person.track_id >= 0 and person.track_id != tid:
                        tid = person.track_id                     # the tracker renamed the person we follow: carry on with the new id
                    ex, _ = mem.update(person, self._pose() or (self.x, self.y, 0.0), now, par('hfov_deg').value, W)
                    pose = self._pose()
                    if trail is not None and pose is not None and mem.pos is not None and person.distance_m > 0:
                        trail.add(mem.pos)
                        wp = trail.target(pose[:2])
                        if wp is not None:
                            ex = trail_error(pose, wp, par('hfov_deg').value)
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
