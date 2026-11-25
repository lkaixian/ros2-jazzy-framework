#!/usr/bin/env python3
"""OpenMANIPULATOR-X helper: `park` (slow, checked move to the rest pose) and `record` (CSV of
reference / feedback / error per joint, for PID tuning). Needs the stack from arm_up.sh running.

    python3 tools/arm/arm_tool.py park
    python3 tools/arm/arm_tool.py record --sec 30 --out /tmp/run.csv
    python3 tools/arm/arm_tool.py gains                 # show servo PID (IDs 11-14)
    python3 tools/arm/arm_tool.py gains --p 700 --d 120 # set; RAM only, stock returns at next driver start
    python3 tools/arm/arm_tool.py tour [--dry]          # range-of-motion tour (use tools/arm/arm_tour.sh)
"""
import argparse
import csv
import math
import os
import signal
import sys
import time

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_interfaces.srv import GetDataFromDxl, SetDataToDxl
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

RATE = 0.03        # rad/s (~1.7 deg/s) parking speed; slower than the servos' own profile cap
MIN_SEC = 10.0     # even a tiny park move is not instant
TOL = 0.05         # rad; "already at rest" and "arrived" threshold
ACTION = '/arm_controller/follow_joint_trajectory'
ARM_IDS = (11, 12, 13, 14)                       # servo IDs of joint1..4 (not the gripper, 15)
STOCK = {'Position P Gain': 800, 'Position I Gain': 100, 'Position D Gain': 100}   # open_manipulator_x xacro
BAND = (0.5, 1.5)  # a gain outside 50-150 % of stock needs --force: too much P/D oscillates the arm
HERE = os.path.dirname(os.path.realpath(__file__))


def load_rest():
    """Rest pose from param/arm_rest_pose.yaml (source tree, else the installed share dir)."""
    paths = [os.path.join(HERE, '..', '..', 'src', 'framework_bringup', 'param', 'arm_rest_pose.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory('framework_bringup'), 'param',
                                  'arm_rest_pose.yaml'))
    except Exception:  # noqa: BLE001 - package not installed: the source path is enough
        pass
    for p in paths:
        if os.path.exists(p):
            with open(p) as f:
                r = yaml.safe_load(f)['arm_rest_pose']
            return dict(zip(r['joint_names'], map(float, r['positions'])))
    raise FileNotFoundError('arm_rest_pose.yaml not found in ' + ', '.join(paths))


def shutdown_ros():
    if rclpy.ok():
        rclpy.shutdown()


def freeze(node, handle, names):
    """Stop where the arm is. Cancel the goal and require the controller's answer; if that fails, preempt it with a
    1 s goal that holds the current pose. Returns True only when one of the two is confirmed."""
    fut = handle.cancel_goal_async()
    end = time.monotonic() + 3.0
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if fut.done() and fut.result() is not None and fut.result().return_code == 0:
        return True
    cur = node.fresh_joints(names)
    if cur is None:
        return False
    hold = FollowJointTrajectory.Goal()
    hold.trajectory.joint_names = list(names)
    hold.trajectory.points = [JointTrajectoryPoint(positions=[cur[n] for n in names], velocities=[0.0] * len(names),
                                                   time_from_start=Duration(sec=1))]
    send = node.hold_client.send_goal_async(hold)
    end = time.monotonic() + 3.0
    while not send.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return send.done() and send.result() is not None and send.result().accepted


def path_verdict(cur, target):
    """(ok, text) from the free-space map checker for the straight move cur -> target; (None, why) when no map exists or it cannot run."""
    try:
        from path_check import verdict
        return verdict({n: cur[n] for n in NAMES}, {n: target[n] for n in NAMES})
    except Exception as e:  # noqa: BLE001 - a broken checker must not move the arm: callers fall back to the corridor rule
        return None, f'path checker unavailable ({type(e).__name__}: {e})'


def taught_route(cur, target):
    """(waypoint dicts, text) for a route made only of poses the arm was hand-guided through; (None, why) when there is none."""
    try:
        from path_check import verdict_route
        from route import load_roadmap, plan
        wps, how = plan(*load_roadmap(), cur, target)
        if wps is None:
            return None, how
        pts = [dict(zip(NAMES, w)) for w in wps]
        ok, text = verdict_route(pts)
        if ok is False:
            return None, f'the taught route is not inside the free-space map ({text})'
        return pts, how
    except Exception as e:  # noqa: BLE001 - a broken planner must not move the arm
        return None, f'route planner unavailable ({type(e).__name__}: {e})'


CORRIDOR_TOL = 0.3     # rad, per joint: how far from the rest<->home segment `park` is still trusted


def in_corridor(cur, rest):
    """True when `cur` is near the straight rest->home joint-space segment, the only park path run on the real arm.
    shortcut: joint-space distance, no model of the robot body. From elsewhere a straight path nearly hit the control board."""
    a, b, c = [rest[n] for n in NAMES], list(TOUR_POSES['home']), [cur[n] for n in NAMES]
    ab, ac = [y - x for x, y in zip(a, b)], [y - x for x, y in zip(a, c)]
    s = min(1.0, max(0.0, sum(u * v for u, v in zip(ab, ac)) / sum(u * u for u in ab)))
    return max(abs(c[i] - (a[i] + s * ab[i])) for i in range(4)) <= CORRIDOR_TOL


def park_seconds(cur, target):
    """Duration for a move: never faster than RATE on the joint that travels furthest."""
    return max(MIN_SEC, max(abs(target[j] - cur[j]) for j in target) / RATE)


class Tool(Node):
    def __init__(self):
        super().__init__('arm_tool')
        self.js = {}
        self.js_t = 0.0
        self.hold_client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.create_subscription(JointState, '/joint_states', self._js, 10)

    def _js(self, m):
        self.js = dict(zip(m.name, m.position))
        self.js_t = time.monotonic()

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def fresh_joints(self, names):
        self.spin_for(1.0)
        if time.monotonic() - self.js_t > 1.0 or not all(n in self.js for n in names):
            return None
        return {n: self.js[n] for n in names}


def park(force=False):
    rest = load_rest()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Tool()
    try:
        cur = node.fresh_joints(list(rest))
        if cur is None:
            print('PARK FAILED: no fresh /joint_states (driver down?)')
            return 1
        if max(abs(rest[j] - cur[j]) for j in rest) < TOL:
            print('already at rest')
            return 0
        via = None                                        # waypoints of a taught route when the straight path is not allowed
        if not force:
            ok, why = path_verdict(cur, rest)
            if ok is False:
                via, how = taught_route(cur, rest)
                if via is None:
                    print(f'PARK REFUSED: the straight path leaves the free-space map ({why}) and there is no taught route: {how}. '
                          'Move the arm by hand onto a taught route, or --force.')
                    return 1
                print(f'straight path refused ({why}); using a taught route: {how}')
            if ok is None and not in_corridor(cur, rest):
                print(f'PARK REFUSED: {why}, and the arm is outside the tested rest<->home corridor (a straight path from elsewhere '
                      'once nearly hit the control board). Move it by hand, or --force.')
                return 1
        cli = ActionClient(node, FollowJointTrajectory, ACTION)
        if not cli.wait_for_server(timeout_sec=5.0):
            print('PARK FAILED: no', ACTION)
            return 1
        sec = park_seconds(cur, rest)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(rest)
        n = len(rest)  # zero end velocity+acceleration -> controller interpolates a smooth quintic, not a ramp
        pt = JointTrajectoryPoint(positions=[rest[j] for j in rest], velocities=[0.0] * n,
                                  accelerations=[0.0] * n)
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        if via:                                           # taught route: constant-speed legs, never faster than RATE, stopping at the end
            t, legs = 0.0, []
            for a, b in zip(via, via[1:]):
                t += max(1.0, max(abs(b[j] - a[j]) for j in rest) / RATE)
                legs.append(JointTrajectoryPoint(positions=[b[j] for j in rest], time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
            legs[-1].velocities, legs[-1].accelerations = [0.0] * n, [0.0] * n
            goal.trajectory.points, sec = legs, t
        print(f'parking over {sec:.0f} s (Ctrl+C cancels and holds)')
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('PARK FAILED: goal rejected')
            return 1
        res = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, res, timeout_sec=sec + 10.0)
        except KeyboardInterrupt:
            held = freeze(node, handle, list(rest))
            print('park cancelled: arm is HELD where it is' if held
                  else 'park cancelled but the STOP IS NOT CONFIRMED: support the arm')
            return 1
        end = node.fresh_joints(list(rest))
        if not res.done() or end is None or max(abs(rest[j] - end[j]) for j in rest) > TOL:
            print('PARK FAILED: did not arrive at the rest pose')
            return 1
        print('parked')
        return 0
    finally:
        node.destroy_node()
        shutdown_ros()


def record(sec, out):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Tool()
    rows, names = [], []
    t0 = time.monotonic()

    def on_state(m):
        nonlocal names
        names = list(m.joint_names)
        rows.append([time.monotonic() - t0] + [v for i in range(len(names)) for v in (
            m.reference.positions[i], m.feedback.positions[i], m.error.positions[i])])

    node.create_subscription(JointTrajectoryControllerState,
                             '/arm_controller/controller_state', on_state, 10)
    try:
        node.spin_for(sec)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    shutdown_ros()
    if not rows:
        print('no controller_state received (is the stack running?)')
        return 1
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t'] + [f'{j}_{k}' for j in names for k in ('ref', 'fb', 'err')])
        w.writerows(rows)
    print(f'{len(rows)} samples -> {out}')
    for k, j in enumerate(names):
        err = [abs(r[3 + 3 * k]) for r in rows]
        fb = [(r[0], r[2 + 3 * k]) for r in rows][::25]    # ~0.25 s apart: encoder steps (1.5 mrad) stay below the signal
        vel = [(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(fb, fb[1:]) if b[0] > a[0]]
        acc = [abs(b - a) / 0.25 for a, b in zip(vel, vel[1:])]
        print(f'  {j}: max|err| {max(err):.4f} rad  rms {(sum(e * e for e in err) / len(err)) ** .5:.4f}'
              f'  peak speed {max(map(abs, vel), default=0):.3f} rad/s  peak accel {max(acc, default=0):.3f} rad/s2')
    return 0


def gains(args):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Tool()
    names = {'Position P Gain': args.p, 'Position I Gain': args.i, 'Position D Gain': args.d}
    setting = {k: v for k, v in names.items() if v is not None}
    try:
        for k, v in setting.items():
            lo, hi = (round(STOCK[k] * b) for b in BAND)
            if not args.force and not lo <= v <= hi:
                print(f'refused: {k}={v} outside {lo}..{hi} (50-150 % of stock {STOCK[k]}); --force to override')
                return 1
        get = node.create_client(GetDataFromDxl, '/dynamixel_hardware_interface/get_dxl_data')
        put = node.create_client(SetDataToDxl, '/dynamixel_hardware_interface/set_dxl_data')
        if not (get.wait_for_service(5.0) and put.wait_for_service(5.0)):
            print('no dynamixel_hardware_interface services (is the stack running?)')
            return 1

        def call(cli, req):
            for _ in range(2):          # the first call after discovery can time out
                f = cli.call_async(req)
                rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
                if f.result() is not None and f.result().result:
                    return f.result()
            return f.result()

        for i in ARM_IDS:
            for k, v in setting.items():
                r = SetDataToDxl.Request(id=i, item_name=k, item_data=v)
                if not (call(put, r) or SetDataToDxl.Response()).result:
                    print(f'ID {i}: write {k} failed')
                    return 1
        if setting:
            node.spin_for(1.0)      # the driver's get_dxl_data lags a write by one cycle: read only after it settled
        for i in ARM_IDS:
            cur = {k: (call(get, GetDataFromDxl.Request(id=i, item_name=k, timeout_sec=1.0)) or
                       GetDataFromDxl.Response()) for k in STOCK}
            print(f'ID {i}: ' + '  '.join(f'{k.split()[1]}={c.item_data}{"" if c.result else "?"}'
                                           for k, c in cur.items()))
        if setting:
            print('note: the readback above can lag a write; run `gains` again to confirm the stored value')
        return 0
    finally:
        node.destroy_node()
        shutdown_ros()


# ---- range-of-motion tour: rest -> home -> long -> high -> turn 180 deg (+-90 deg about the front) -> home -> rest
# Poses (joints 2-4, rad) are 0.12 rad inside the URDF limits; computed from the URDF forward kinematics,
# 'long' = gripper 39 cm ahead of the arm base, 'high' = straight up 44 cm (checked: links stay above the arm base plane).
TOUR_POSES = {'home': (0.0, -1.0, 1.0, 0.0), 'long': (0.0, 1.38, -1.38, -0.04), 'high': (0.0, -0.14, -1.38, -0.04)}
TRACK_ERR, TRACK_SEC = 0.3, 1.5   # tour watchdog: freeze when any joint is this far (rad) off its reference this long
TOUR_HOLD = 2.0         # s standing still at every pose
TOUR_RATE = 0.015       # rad/s: half the park rate for a first pass
TURN_RATE = 0.05        # rad/s for joint 1 at 'high' (the arm is vertical there: the gripper moves about 1 mm/s)
NAMES = ('joint1', 'joint2', 'joint3', 'joint4')


def tour_plan(rest, first='left', rate=TOUR_RATE, turn_rate=TURN_RATE):
    """[(label, positions, seconds from the previous pose)] ending at rest; every leg at most `rate` on its slowest joint."""
    sgn = 1.0 if first == 'left' else -1.0
    r = tuple(rest[n] for n in NAMES)
    hi = TOUR_POSES['high']
    turn = lambda a: (a,) + hi[1:]  # noqa: E731
    legs = [('home', TOUR_POSES['home'], rate), ('long', TOUR_POSES['long'], rate), ('high', hi, rate),
            (f'turn {first} 90', turn(sgn * math.pi / 2), turn_rate),
            ('turn other way 90', turn(-sgn * math.pi / 2), turn_rate), ('centre', turn(0.0), turn_rate),
            ('home', TOUR_POSES['home'], rate), ('rest', r, rate)]
    plan, cur = [], r
    for label, pos, rt in legs:
        plan.append((label, pos, max(MIN_SEC, max(abs(a - b) for a, b in zip(pos, cur)) / rt)))
        cur = pos
    return plan


def tour(args):
    rest = load_rest()
    plan = tour_plan(rest, args.first, args.rate, args.turn_rate)
    total = sum(sec + TOUR_HOLD for _, _, sec in plan)
    print(f'tour: {len(plan)} legs, about {total / 60:.1f} min. Ctrl+C freezes the arm where it is (torque stays on).')
    for label, pos, sec in plan:
        print(f'  -> {label:<18} ' + ' '.join(f'{n[-1]}:{v:+.2f}' for n, v in zip(NAMES, pos)) + f'   {sec:4.0f} s')
    if args.dry:
        return 0
    for sig in (signal.SIGTERM, signal.SIGHUP):          # a closed terminal must freeze the arm, not leave a goal running
        signal.signal(sig, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Tool()
    try:
        cur = node.fresh_joints(list(NAMES))
        if cur is None:
            print('TOUR REFUSED: no fresh /joint_states (driver down?)')
            return 1
        if max(abs(cur[n] - rest[n]) for n in NAMES) > 0.2:
            print('TOUR REFUSED: the arm is not at the rest pose; run `arm_tool.py park` first')
            return 1
        prev = {n: rest[n] for n in NAMES}                # every leg must stay inside the taught free-space region
        for label, pos, _ in plan:
            nxt = dict(zip(NAMES, pos))
            ok, why = path_verdict(prev, nxt)
            if not ok:
                print(f'TOUR REFUSED at leg "{label}": ' + ('no free-space map: ' if ok is None else 'free-space map: ') + why)
                return 1
            prev = nxt
        cli = ActionClient(node, FollowJointTrajectory, ACTION)
        if not cli.wait_for_server(timeout_sec=5.0):
            print('TOUR REFUSED: no', ACTION)
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(NAMES)
        t = 0.0
        for _, pos, sec in plan:
            for dt in (sec, TOUR_HOLD):               # arrive, then stand still (zero velocity and acceleration at each pose)
                t += dt
                goal.trajectory.points.append(JointTrajectoryPoint(
                    positions=list(pos), velocities=[0.0] * 4, accelerations=[0.0] * 4,
                    time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print('TOUR REFUSED: goal rejected')
            return 1
        bad = [None]    # when the tracking error first exceeded TRACK_ERR; the arm is lagging, blocked or lying down

        def on_state(m):
            e = max(abs(x) for x in m.error.positions)
            bad[0] = (bad[0] or time.monotonic()) if e > TRACK_ERR else None

        node.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', on_state, 10)
        res, t0, last = handle.get_result_async(), time.monotonic(), 0.0
        try:
            while not res.done():
                rclpy.spin_once(node, timeout_sec=0.2)
                now = time.monotonic() - t0
                if now - last >= 5.0:
                    last = now
                    print(f'  t={now:4.0f}/{t:.0f} s  ' + ' '.join(f'{n[-1]}:{node.js.get(n, float("nan")):+.2f}' for n in NAMES))
                if bad[0] and time.monotonic() - bad[0] > TRACK_SEC:
                    raise RuntimeError(f'arm is {TRACK_ERR} rad off its reference for {TRACK_SEC} s (blocked, lagging or fallen)')
                if now > t + 30.0:
                    raise TimeoutError('tour overran its schedule')
        except BaseException as e:                    # Ctrl+C, SIGTERM, SIGHUP, overrun: cancel = the controller holds here
            held = freeze(node, handle, list(NAMES))
            print(f'\nFROZEN ({type(e).__name__}: {e}): ' + ('the arm is HELD where it is, torque on'
                  if held else 'STOP NOT CONFIRMED, support the arm'))
            return 1
        end = node.fresh_joints(list(NAMES))
        if end is None or max(abs(end[n] - rest[n]) for n in NAMES) > TOL:
            print('TOUR FAILED: did not end at the rest pose')
            return 1
        print('tour finished, arm at rest')
        return 0
    finally:
        node.destroy_node()
        shutdown_ros()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    pk = sub.add_parser('park')
    pk.add_argument('--force', action='store_true', help='park even from outside the tested corridor')
    r = sub.add_parser('record')
    r.add_argument('--sec', type=float, default=30.0)
    r.add_argument('--out', default='arm_run.csv')
    tr = sub.add_parser('tour')
    tr.add_argument('--first', choices=('left', 'right'), default='left')
    tr.add_argument('--rate', type=float, default=TOUR_RATE)
    tr.add_argument('--turn-rate', type=float, default=TURN_RATE)
    tr.add_argument('--dry', action='store_true', help='print the plan, touch nothing')
    g = sub.add_parser('gains')
    for k in 'pid':
        g.add_argument('--' + k, type=int)
    g.add_argument('--force', action='store_true')
    a = ap.parse_args()
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if uri.startswith('file://') and not os.path.isfile(uri[7:]):   # a stale path crashes every node at start
        print(f'warn: ignoring CYCLONEDDS_URI, file missing: {uri}')
        del os.environ['CYCLONEDDS_URI']
    sys.exit({'park': lambda: park(a.force), 'record': lambda: record(a.sec, a.out),
              'gains': lambda: gains(a), 'tour': lambda: tour(a)}[a.cmd]())


if __name__ == '__main__':
    main()
