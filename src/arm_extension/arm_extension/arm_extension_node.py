"""Arm BT nodes on the guarded stack (arm_control): every motion is checked against the taught free-space map first, runs at a fixed slow
rate, and freezes on cancel. No MoveIt. Moves nothing unless the `live` parameter is true (dry-run plans and checks only)."""
import json
import threading
import time

import numpy as np
from arm_control.free_map import JOINTS, FreeMap
from arm_control.gripper import joint_m_for_gap
from arm_control.ik import map_limits, solve
from arm_control.motion import RATE, Refused, plan_route, schedule
from arm_control.omx_fk import Arm
from arm_control.path_check import default_map
from arm_control.pick import approach_poses, grasp_from_point
from bt_common.executor_node import ExtensionExecutor, call_action, call_service, port_float, port_str, run_executor
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_interfaces.srv import GetDataFromDxl
from object_detection_msgr.msg import ObjectArray
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

GRIPPER_ID = 15


class ArmError(Exception):
    """A refusal or failure with a message for the BT."""


class NoCancel:
    cancelled = False


class ArmExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('arm_extension')
        for k, v in dict(live=False, map_file='', require_health=True, health_max_age_sec=5.0, rate=RATE,
                         joint_action='/arm_controller/follow_joint_trajectory',
                         gripper_action='/gripper_controller/gripper_cmd', dxl_get='/dynamixel_hardware_interface/get_dxl_data',
                         joint_topic='/joint_states', state_topic='/arm_controller/controller_state',
                         objects_topic='/arm_eye/objects', depth_topic='/arm_cam/stereo/image_raw', max_age_sec=1.0,
                         track_cycles=4, track_max_cycles=20, track_period_sec=0.4, track_scan_after=3, track_give_up=12,
                         track_max_sigma_m=0.02, scan_fov_rad=0.5, target_source='arm_eye', fixed_cam_xyz=[0.0, -0.08, -0.02], fixed_cam_info_topic='',
                         approach_m=0.06, grasp_pitch_deg=0.0, grasp_depth_offset_m=0.03,
                         workspace_min=[0.05, -0.4, -0.3], workspace_max=[0.45, 0.4, 0.4],
                         gripper_open_gap_mm=60.0, gripper_max_gap_mm=72.0, gripper_max_effort=20.0, goal_current_cap=250,
                         object_width_mm=[''], places=[''], track_err=0.3, track_sec=1.5, rest_pose=[0.0, -1.85, 1.52, 0.45]).items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        self.js, self.js_t, self.objs, self.objs_t, self.depth, self.depth_t = {}, 0.0, None, 0.0, None, 0.0
        self.err_since = None
        self.traj = ActionClient(self, FollowJointTrajectory, self.p('joint_action'))
        self.grip = ActionClient(self, GripperCommand, self.p('gripper_action'))
        self.dxl = self.create_client(GetDataFromDxl, self.p('dxl_get'))
        self.create_subscription(JointState, self.p('joint_topic'), self._on_js, 10)
        self.create_subscription(JointTrajectoryControllerState, self.p('state_topic'), self._on_state, 10)
        self.create_subscription(ObjectArray, self.p('objects_topic'), self._on_objs, qos_profile_sensor_data)
        self.create_subscription(Image, self.p('depth_topic'), self._on_depth, qos_profile_sensor_data)
        self.fixed_k = None
        if self.p('target_source') == 'base_camera':
            self.create_subscription(CameraInfo, self.p('fixed_cam_info_topic'), self._on_info, qos_profile_sensor_data)
        self.health, self.health_t = None, 0.0
        self.create_subscription(String, 'arm/health', self._on_health, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._fk = self._fmap = None
        self.handlers = {'ArmPark': self._park, 'MoveArmToPose': self._move, 'OpenGripper': self._open,
                         'CloseGripper': self._close, 'PickObject': self._pick, 'PlaceObject': self._place}

    # ---- sensors
    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):      # the Kobuki driver publishes wheel-only /joint_states on the same topic
            self.js, self.js_t = d, time.monotonic()

    def _on_state(self, m):
        e = max(abs(x) for x in m.error.positions) if m.error.positions else 0.0
        self.err_since = (self.err_since or time.monotonic()) if e > self.p('track_err') else None

    def _on_health(self, m):
        self.health, self.health_t = json.loads(m.data), time.monotonic()

    def _health_ok(self):
        """Live motion needs a recent, clean arm/health (fail closed: no health node = no motion)."""
        if not self.p('require_health'):
            return
        if self.health is None or time.monotonic() - self.health_t > self.p('health_max_age_sec'):
            raise ArmError('no recent arm/health: is arm_health_node running? not moving the arm')
        if not self.health.get('ok'):
            raise ArmError('arm health: ' + '; '.join(self.health.get('problems', ['unknown'])) + '. Not moving the arm')

    def _on_objs(self, m):
        self.objs, self.objs_t = m, time.monotonic()

    def _on_depth(self, m):
        if m.encoding in ('16UC1', 'mono16') and m.height * m.width * 2 == len(m.data):
            self.depth, self.depth_t = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width), time.monotonic()

    def _on_info(self, m):
        self.fixed_k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _fresh(self, t):
        return time.monotonic() - t <= self.p('max_age_sec')

    def _joints(self):
        if not self._fresh(self.js_t) or not all(j in self.js for j in JOINTS):
            raise ArmError('no fresh /joint_states (arm driver down?)')
        return {j: self.js[j] for j in JOINTS}

    def _still(self, ctl):
        a = self._joints()
        ctl.sleep(0.4)
        if max(abs(self._joints()[j] - a[j]) for j in JOINTS) > 0.01:
            raise ArmError('the arm is moving: refusing to measure or plan from a moving pose')

    def _models(self):
        if self._fk is None:
            path = self.p('map_file') or default_map()
            if path is None:
                raise ArmError('no free-space map (~/arm_free_space*.npz): the arm may not be moved without one')
            self._fk, self._fmap = Arm(), FreeMap.load(path)
        return self._fk, self._fmap

    # ---- motion
    def _legs(self, start, goals):
        """Check every leg before anything moves: list of joint-dict paths. Raises ArmError."""
        out, cur = [], start
        for g in goals:
            try:
                out.append(plan_route(cur, g, map_path=self.p('map_file') or None))
            except Refused as e:
                raise ArmError(f'refused: {e}') from None
            cur = g
        return out

    def _drive(self, points, ctl):
        self._health_ok()
        sched = schedule(points, self.p('rate'))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for k, (pos, t) in enumerate(sched):
            last = k == len(sched) - 1
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=pos, velocities=[0.0] * 4 if last else [], accelerations=[0.0] * 4 if last else [],
                time_from_start=Duration(sec=int(t), nanosec=int((t % 1) * 1e9))))
        total = sched[-1][1]
        if not self.traj.wait_for_server(2.0):
            raise ArmError('no arm_controller follow_joint_trajectory server')
        done, box = threading.Event(), {}
        self.traj.send_goal_async(goal).add_done_callback(lambda f: (box.__setitem__('gh', f.result()), done.set()))
        if not done.wait(5.0) or not box['gh'].accepted:
            raise ArmError('arm trajectory rejected')
        fin = threading.Event()
        box['gh'].get_result_async().add_done_callback(lambda f: (box.__setitem__('res', f.result()), fin.set()))
        t0, self.err_since = time.monotonic(), None
        while not fin.wait(0.05):
            why = ('cancelled' if ctl.cancelled else f'overran its {total:.0f} s schedule' if time.monotonic() - t0 > total + 15.0
                   else 'joint tracking error (blocked or lagging)'
                   if self.err_since and time.monotonic() - self.err_since > self.p('track_sec') else '')
            if why:
                self._hold(box['gh'])
                raise ArmError(f'{why}: arm held where it is' if why != 'cancelled' else 'cancelled: arm held')
        end = self._joints()
        if max(abs(end[j] - points[-1][j]) for j in JOINTS) > 0.05:
            raise ArmError('arm did not arrive at the target pose')

    def _hold(self, gh):
        """Cancel the goal; the controller then holds the current pose. Confirmed by its reply, else preempt with a hold goal."""
        fut = gh.cancel_goal_async()
        end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if fut.done() and fut.result() is not None and fut.result().return_code == 0:
            return
        cur = self._joints()
        hold = FollowJointTrajectory.Goal()
        hold.trajectory.joint_names = list(JOINTS)
        hold.trajectory.points = [JointTrajectoryPoint(positions=[cur[j] for j in JOINTS], velocities=[0.0] * 4,
                                                       time_from_start=Duration(sec=1))]
        res, err = call_action(self.traj, hold, NoCancel(), 5.0)
        if res is None:
            raise ArmError(f'STOP NOT CONFIRMED ({err}): support the arm')

    def _go(self, goals, ctl, label):
        """Plan and check all legs, then (if live) drive them. Returns the planned time in seconds."""
        legs = self._legs(self._joints(), goals)
        secs = sum(schedule(p, self.p('rate'))[-1][1] for p in legs)
        if self.p('live'):
            for p in legs:
                self._drive(p, ctl)
        return secs

    # ---- gripper
    def _check_current(self, ctl):
        r, err = call_service(self.dxl, GetDataFromDxl.Request(id=GRIPPER_ID, item_name='Goal Current', timeout_sec=1.0), ctl, 3.0)
        cap = self.p('goal_current_cap')
        if r is None or not r.result:
            raise ArmError(f'cannot read the gripper Goal Current ({err or "driver refused"}): not moving it')
        if r.item_data > cap:
            raise ArmError(f'gripper Goal Current is {r.item_data} (cap {cap}): set it lower first '
                           '(a power cycle resets it to the 3.2 A maximum)')

    def _gripper(self, gap, effort, ctl):
        pos = joint_m_for_gap(gap)
        if not self.p('live'):
            return True, f'DRY: gripper to a {gap:.0f} mm gap'
        self._health_ok()
        self._check_current(ctl)
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = pos, min(effort, self.p('gripper_max_effort'))
        res, err = call_action(self.grip, goal, ctl, 15.0)
        if res is None:
            return False, err
        return (res.reached_goal or res.stalled), ('stalled' if res.stalled else 'reached' if res.reached_goal else 'not reached')

    def _open(self, ports, ctl):
        gap = min(port_float(ports, 'width', self.p('gripper_open_gap_mm'), lo=20.0), self.p('gripper_max_gap_mm'))
        return self._gripper(gap, self.p('gripper_max_effort'), ctl)

    def _close(self, ports, ctl):
        mx = self.p('gripper_max_effort')
        return self._gripper(0.0, min(port_float(ports, 'force', mx, lo=0.0), mx), ctl)

    # ---- BT nodes
    def _park(self, ports, ctl):
        try:
            secs = self._go([dict(zip(JOINTS, self.p('rest_pose')))], ctl, 'park')
        except ArmError as e:
            return False, str(e)
        return True, ('parked' if self.p('live') else f'DRY: park checked, would take {secs:.0f} s')

    def _move(self, ports, ctl):
        if any(float(ports.get(k) or 0) != 0 for k in ('roll', 'yaw')):
            return False, '4-joint arm: only pitch can be set, not roll or yaw'
        xyz = tuple(port_float(ports, k) for k in 'xyz')
        lo, hi = self.p('workspace_min'), self.p('workspace_max')
        if not all(a <= v <= b for v, a, b in zip(xyz, lo, hi)):
            return False, f'target {xyz} outside configured workspace {lo}..{hi}'
        pitch = port_float(ports, 'pitch') if ports.get('pitch') else None
        try:
            arm, fmap = self._models()
            q = solve(arm, xyz, pitch, seeds=[tuple(self._joints().values())], limits=map_limits(fmap))
            if q is None:
                return False, 'no arm pose inside the taught joint range reaches that point'
            secs = self._go([q], ctl, 'move')
        except (ArmError, ValueError) as e:
            return False, str(e)
        return True, ('moved' if self.p('live') else f'DRY: move checked, would take {secs:.0f} s')

    def _width_ok(self, label):
        for item in self.p('object_width_mm'):
            if item.startswith(label + ':') and float(item.split(':')[1]) > self.p('gripper_max_gap_mm'):
                raise ArmError(f'{label} is {item.split(":")[1]} mm wide: wider than the {self.p("gripper_max_gap_mm"):.0f} mm gripper')

    def _points(self, label, arm, cur):
        """Front-surface point (arm base frame) of every `label` detection, with its confidence. Raises ArmError when none is usable."""
        if self.objs is None or not self._fresh(self.objs_t) or self.depth is None or not self._fresh(self.depth_t):
            raise ArmError('no fresh detections or depth from the camera')
        hits = [o for o in self.objs.objects if o.label == label]
        if not hits:
            raise ArmError(f'{label} not seen by the camera')
        from arm_control.hand_eye import box_depth_m, fixed_cam_point, load, upright_pixel_to_arm
        fixed = self.p('target_source') == 'base_camera'
        w, h = self.objs.image_width, self.objs.image_height
        out = []
        for o in hits:
            d = box_depth_m(self.depth, (o.x_min, o.y_min, o.x_max, o.y_max), w, h, 0 if fixed else load()[2])   # a fixed camera is upright
            if d is None:
                continue
            u, v = (o.x_min + o.x_max) / 2, (o.y_min + o.y_max) / 2
            if fixed:
                if self.fixed_k is None:
                    raise ArmError(f"no camera_info on {self.p('fixed_cam_info_topic')}")
                out.append((fixed_cam_point(u, v, d, self.fixed_k, self.p('fixed_cam_xyz')), o.confidence))
            else:
                out.append((np.array(upright_pixel_to_arm(arm, cur, u, v, d, w, h), float), o.confidence))
        if not out:
            raise ArmError(f'no valid depth at the {label}')
        return out

    def _track_target(self, label, arm, fmap, ctl):
        """Look, filter, look again until the pinned target (id 1) is steady; returns (front-surface point, text).

        Detections go through a Kalman filter (arm_control.track). A missed look coasts on the prediction; after `track_scan_after` misses in
        a row the eye-in-hand camera is turned (joint 1 only, checked like any other move) toward the predicted point, clipped to the workable
        box; after `track_give_up` misses it refuses. A camera fixed to the robot cannot be turned, so it only coasts.
        """
        from arm_control.track import Track, scan_yaws
        need, cap, period = int(self.p('track_cycles')), int(self.p('track_max_cycles')), self.p('track_period_sec')
        scan_after, give_up = int(self.p('track_scan_after')), int(self.p('track_give_up'))
        tr, scans, why = None, [], ''
        for _ in range(cap):
            if ctl.cancelled:
                raise ArmError('cancelled')
            cur, t = self._joints(), time.monotonic()
            try:
                cands = self._points(label, arm, cur)
            except ArmError as e:
                cands, why = [], str(e)
            if tr is None:
                if cands:
                    tr = Track(max(cands, key=lambda c: c[1])[0], t)       # first look: the most confident one is pinned as id 1
            else:
                tr.step(t, [c[0] for c in cands])
            if tr is not None and tr.seen >= need and tr.missed == 0 and tr.sigma <= self.p('track_max_sigma_m'):
                note = f'{tr.seen} looks, sigma {tr.sigma * 100:.1f} cm' + (f', {len(scans)} scan(s)' if scans else '')
                return tr.pos, note
            if tr is not None and tr.missed >= give_up:
                raise ArmError(f'lost the {label} ({why or "not matching the pinned object"}) after {tr.missed} looks, last at {np.round(tr.pos, 3).tolist()}')
            if tr is not None and tr.missed >= scan_after and self.p('target_source') != 'base_camera':
                j1 = map_limits(fmap)['joint1']
                yaws = scan_yaws(tr.pos, self.p('workspace_min'), self.p('workspace_max'), j1[0], j1[1], self.p('scan_fov_rad'),
                                 len(scans) + 2)
                nxt = yaws[len(scans)] if len(scans) < len(yaws) else None
                if nxt is None:
                    raise ArmError(f'lost the {label}: nothing left to scan toward {np.round(tr.pos, 3).tolist()}')
                scans.append(nxt)
                self._go([{**cur, 'joint1': nxt}], ctl, 'scan')       # moves only when live; checked against the free-space map either way
                if self.p('live'):
                    self._still(ctl)
            ctl.sleep(period)
        raise ArmError(f'the {label} never held steady in {cap} looks' + (f': {why}' if why else ''))

    def _pick(self, ports, ctl):
        import math
        label = port_str(ports, 'label')
        try:
            self._width_ok(label)
            arm, fmap = self._models()
            self._still(ctl)
            cur = self._joints()
            point, tnote = self._track_target(label, arm, fmap, ctl)
            cur = self._joints()
            lim, off, app, pit = map_limits(fmap), self.p('grasp_depth_offset_m'), self.p('approach_m'), math.radians(self.p('grasp_pitch_deg'))
            g = grasp_from_point(arm, cur, point, lim, self.p('workspace_min'), self.p('workspace_max'), off, app, pit)
            legs = self._legs(cur, [g['pre'], g['grasp'], g['pre']])
            secs = sum(schedule(p, self.p('rate'))[-1][1] for p in legs)
            if not self.p('live'):
                return True, (f'DRY: {label} at {np.round(g["point"], 3).tolist()} m ({tnote}; source {self.p("target_source")}' +
                              (f', fixed_cam_xyz {self.p("fixed_cam_xyz")}' if self.p('target_source') == 'base_camera' else '') +
                              f'), 3 legs checked, would take {secs:.0f} s + gripper')
            ok, msg = self._gripper(self.p('gripper_open_gap_mm'), self.p('gripper_max_effort'), ctl)
            if not ok:
                return False, f'could not open the gripper: {msg}'
            self._drive(legs[0], ctl)
            self._drive(legs[1], ctl)
            ok, msg = self._gripper(0.0, self.p('gripper_max_effort'), ctl)
            if not ok or msg != 'stalled':
                return False, f'nothing in the gripper after closing ({msg}); arm left at the grasp pose'
            self._drive(legs[2], ctl)
            return True, f'holding {label}'
        except (ArmError, Refused) as e:
            return False, str(e)

    def _place(self, ports, ctl):
        """Put the held object down at a named place (arm base frame, from the `places` parameter 'name:x,y,z'), release it, back off."""
        import math
        name = port_str(ports, 'place')
        spec = next((i.split(':', 1)[1] for i in self.p('places') if i.startswith(name + ':')), None)
        try:
            if spec is None:
                raise ArmError(f'unknown place {name!r}: add "name:x,y,z" (arm base frame, metres) to the places parameter')
            xyz = [float(v) for v in spec.split(',')]
            arm, fmap = self._models()
            self._still(ctl)
            cur = self._joints()
            g = approach_poses(arm, cur, xyz, map_limits(fmap), self.p('workspace_min'), self.p('workspace_max'),
                               self.p('approach_m'), math.radians(self.p('grasp_pitch_deg')))
            legs = self._legs(cur, [g['pre'], g['grasp'], g['pre']])
            secs = sum(schedule(p, self.p('rate'))[-1][1] for p in legs)
            if not self.p('live'):
                return True, f'DRY: place at {name} {xyz}, 3 legs checked, would take {secs:.0f} s + gripper'
            self._drive(legs[0], ctl)
            self._drive(legs[1], ctl)
            ok, msg = self._gripper(self.p('gripper_open_gap_mm'), self.p('gripper_max_effort'), ctl)
            if not ok:
                return False, f'could not release at {name}: {msg}'
            self._drive(legs[2], ctl)
            return True, f'placed at {name}'
        except (ArmError, Refused, ValueError) as e:
            return False, str(e)


def main(args=None):
    run_executor(ArmExtension, args)


if __name__ == '__main__':
    main()
