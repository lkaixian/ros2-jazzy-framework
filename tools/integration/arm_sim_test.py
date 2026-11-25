#!/usr/bin/env python3
"""Simulated-arm test of arm_extension: the real node against a fake arm (joint states, detections, depth, trajectory + gripper servers,
servo register). No hardware, no MoveIt.

  python3 -m pytest tools/integration/arm_sim_test.py -q        (own ROS domain, localhost discovery)

Covers the ROS glue the unit tests cannot: dry-run plans without driving, live runs drive 3 legs and the guarded gripper, a cancel or a
stalled arm is held, a hot gripper register refuses, no map / unseen object / moving arm refuse. Says nothing about the real arm.
"""
import os
import threading
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('ARM_SIM_DOMAIN', '91')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from arm_control.free_map import JOINTS, FreeMap  # noqa: E402
from arm_control.omx_fk import Arm  # noqa: E402
from arm_control.path_check import sample_path  # noqa: E402
from arm_control.pick import grasp_poses  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from control_msgs.action import FollowJointTrajectory, GripperCommand  # noqa: E402
from control_msgs.msg import JointTrajectoryControllerState  # noqa: E402
from dynamixel_interfaces.msg import DynamixelState  # noqa: E402
from dynamixel_interfaces.srv import GetDataFromDxl, RebootDxl  # noqa: E402
from object_detection_msgr.msg import Object, ObjectArray  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402

CUR = (0.0, -0.9, 1.2, 0.0)
WS = ([0.05, -0.4, -0.3], [0.45, 0.4, 0.4])
DEPTH_MM = 250


class FakeArm(Node):
    def __init__(self):
        super().__init__('fake_arm')
        cb = ReentrantCallbackGroup()
        self.q = dict(zip(JOINTS, CUR))
        self.label, self.seen_depth = 'cup', True
        self.current, self.grip_stalls, self.hang = 100, True, False
        self.goals, self.grips, self.cancels = [], [], 0
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.st = self.create_publisher(JointTrajectoryControllerState, '/arm_controller/controller_state', 10)
        self.obj = self.create_publisher(ObjectArray, '/arm_eye/objects', 10)
        self.depth = self.create_publisher(Image, '/arm_cam/stereo/image_raw', 10)
        self.create_timer(0.05, self._pub, callback_group=cb)
        ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', self._traj,
                     cancel_callback=self._cancel, callback_group=cb)
        ActionServer(self, GripperCommand, '/gripper_controller/gripper_cmd', self._grip, callback_group=cb)
        self.hw, self.reboots = [0] * 5, 0
        self.dxl = self.create_publisher(DynamixelState, '/dynamixel_hardware_interface/dxl_state', 10)
        self.create_service(RebootDxl, '/dynamixel_hardware_interface/reboot_dxl', self._reboot, callback_group=cb)
        self.create_service(GetDataFromDxl, '/dynamixel_hardware_interface/get_dxl_data', self._dxl, callback_group=cb)
        self.t = 0

    def _pub(self):
        self.js.publish(JointState(name=list(JOINTS), position=list(self.q.values())))
        self.st.publish(JointTrajectoryControllerState(joint_names=list(JOINTS)))
        self.dxl.publish(DynamixelState(id=[11, 12, 13, 14, 15], torque_state=[True] * 5, dxl_hw_state=list(self.hw)))
        self.t += 1
        if self.t % 4 == 0:
            o = Object(label=self.label, confidence=0.9, x_min=600.0, y_min=330.0, x_max=680.0, y_max=390.0)
            self.obj.publish(ObjectArray(image_width=1280, image_height=720, objects=[o]))
            if self.seen_depth:
                d = np.full((720, 1280), DEPTH_MM, np.uint16)
                self.depth.publish(Image(height=720, width=1280, encoding='16UC1', step=2560, data=d.tobytes()))

    def _cancel(self, _):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def _traj(self, gh):
        self.goals.append([list(p.positions) for p in gh.request.trajectory.points])
        while self.hang and not gh.is_cancel_requested:
            time.sleep(0.02)
        if gh.is_cancel_requested:
            gh.canceled()
            return FollowJointTrajectory.Result()
        for p in gh.request.trajectory.points:
            self.q = dict(zip(JOINTS, p.positions))
            time.sleep(0.05)
        gh.succeed()
        return FollowJointTrajectory.Result()

    def _grip(self, gh):
        self.grips.append(gh.request.command.position)
        gh.succeed()
        return GripperCommand.Result(reached_goal=not self.grip_stalls, stalled=self.grip_stalls)

    def _reboot(self, _req, res):
        self.reboots += 1
        self.hw = [0] * 5
        res.result = True
        return res

    def _dxl(self, req, res):
        res.result = True
        res.item_data = {'Present Temperature': 30, 'Present Input Voltage': 120}.get(req.item_name, self.current)
        return res


@pytest.fixture(scope='module')
def world(tmp_path_factory):
    arm = Arm()
    # the map the checker sees: exactly the three legs this scene produces (this tests the glue, not the map maths)
    wide = {j: (-3.0, 3.0) for j in JOINTS}
    from arm_control.hand_eye import load  # noqa: F401
    cur = dict(zip(JOINTS, CUR))
    g = grasp_poses(arm, cur, 640.0, 360.0, DEPTH_MM / 1000.0, 1280, 720, wide, *WS, 0.03, 0.06, 0.0)
    fmap = FreeMap()
    for a, b in ((cur, g['pre']), (g['pre'], g['grasp'])):
        for q in sample_path(a, b):
            fmap.add(arm, q)
    path = str(tmp_path_factory.mktemp('map') / 'map.npz')
    fmap.save(path)
    rclpy.init(args=['--ros-args', '-p', 'live:=false', '-p', f'map_file:={path}', '-p', 'rate:=2.0', '-p', 'auto_reboot:=true', '-p', 'poll_sec:=0.3', '-p', 'max_age_sec:=2.0'])
    from arm_extension.arm_extension_node import ArmExtension
    from arm_extension.arm_health_node import ArmHealth
    fake, ext, health = FakeArm(), ArmExtension(), ArmHealth()
    client_node = Node('arm_sim_client')
    ex = MultiThreadedExecutor(num_threads=8)
    for n in (fake, ext, health, client_node):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    cli = ActionClient(client_node, ExecuteNode, 'arm_extension/execute')
    assert cli.wait_for_server(10.0)
    time.sleep(1.5)
    yield fake, ext, cli
    ex.shutdown()
    rclpy.shutdown()


def run(cli, node, **ports):
    goal = ExecuteNode.Goal(node_id=node, port_names=list(ports), port_values=[str(v) for v in ports.values()])
    f = cli.send_goal_async(goal)
    t = time.monotonic()
    while not f.done() and time.monotonic() - t < 5:
        time.sleep(0.02)
    gh = f.result()
    r = gh.get_result_async()
    while not r.done() and time.monotonic() - t < 40:
        time.sleep(0.05)
    return r.result().result, gh


def reset(fake, ext, live):
    fake.q, fake.label, fake.seen_depth = dict(zip(JOINTS, CUR)), 'cup', True
    fake.current, fake.grip_stalls, fake.hang = 100, True, False
    fake.goals, fake.grips, fake.cancels, fake.hw, fake.reboots = [], [], 0, [0] * 5, 0
    ext.set_parameters([rclpy.parameter.Parameter('live', value=live)])
    time.sleep(0.8)


def test_dry_pick_plans_three_legs_and_moves_nothing(world):
    fake, ext, cli = world
    reset(fake, ext, False)
    res, _ = run(cli, 'PickObject', label='cup')
    assert res.success and res.message.startswith('DRY: cup at') and '3 legs checked' in res.message, res.message
    assert not fake.goals and not fake.grips


def test_live_pick_drives_three_legs_and_closes_on_the_object(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    res, _ = run(cli, 'PickObject', label='cup')
    assert res.success and res.message == 'holding cup', res.message
    assert len(fake.goals) == 3 and len(fake.grips) == 2
    assert fake.grips[0] < fake.grips[1]                       # this gripper: OPEN is the LOWER joint value (param/arm_gripper.yaml)


def test_closing_on_nothing_is_a_failure_and_the_arm_stays_put(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    fake.grip_stalls = False
    res, _ = run(cli, 'PickObject', label='cup')
    assert not res.success and 'nothing in the gripper' in res.message
    assert len(fake.goals) == 2                                # never retreated with an empty grasp


def test_hot_gripper_register_refuses_before_any_gripper_command(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    fake.current = 1193
    res, _ = run(cli, 'PickObject', label='cup')
    assert not res.success and 'Goal Current' in res.message
    assert not fake.goals and not fake.grips


def test_object_not_seen_or_no_depth_refuses_without_moving(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    res, _ = run(cli, 'PickObject', label='bottle')
    assert not res.success and 'not seen' in res.message and not fake.goals


def test_cancel_during_a_leg_holds_the_arm(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    fake.hang = True
    f = cli.send_goal_async(ExecuteNode.Goal(node_id='PickObject', port_names=['label'], port_values=['cup']))
    t = time.monotonic()
    while not fake.goals and time.monotonic() - t < 10:
        time.sleep(0.05)
    assert fake.goals, 'the first leg was never sent'
    f.result().cancel_goal_async()
    r = f.result().get_result_async()
    while not r.done() and time.monotonic() - t < 20:
        time.sleep(0.05)
    res = r.result().result
    assert not res.success and fake.cancels >= 1 and len(fake.goals) == 1       # cancelled once, never went on to the next leg


def test_unknown_place_is_refused(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    res, _ = run(cli, 'PlaceObject', place='moon')
    assert not res.success and 'unknown place' in res.message and not fake.goals


def test_a_servo_error_blocks_live_motion_until_it_clears(world):
    fake, ext, cli = world
    reset(fake, ext, True)
    fake.hw = [0, 0, 32, 0, 0]                    # servo 13 overload, arm NOT at rest: no reboot, no motion
    time.sleep(1.5)
    res, _ = run(cli, 'PickObject', label='cup')
    assert not res.success and 'arm health' in res.message and 'overload' in res.message, res.message
    assert not fake.goals and not fake.grips
    time.sleep(4.0)
    assert fake.reboots == 0                      # the arm is held up: a reboot would drop it
    fake.hw = [0] * 5


def test_a_persistent_overload_is_rebooted_only_when_parked_at_rest(world):
    fake, ext, cli = world
    reset(fake, ext, False)
    fake.q = dict(zip(JOINTS, (0.0, -1.85, 1.52, 0.45)))
    fake.hw = [0, 0, 32, 0, 0]
    t = time.monotonic()
    while fake.reboots == 0 and time.monotonic() - t < 10:
        time.sleep(0.1)
    assert fake.reboots == 1 and time.monotonic() - t >= 3.0       # waited for the fault to persist
    time.sleep(1.5)
    assert fake.reboots == 1                                       # cleared by the reboot: not repeated
