import math

from bt_common.executor_node import (ExtensionExecutor, call_action, port_float,
                                     run_executor)
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

from arm_extension.moveit_goal import build_goal

SUCCESS = 1   # moveit_msgs/MoveItErrorCodes.SUCCESS


class ArmExtension(ExtensionExecutor):
    """Needs a running MoveIt move_group. Group/link/frame are parameters: set them for
    the actual arm. Velocity scaling is capped by max_velocity_scale (default 0.1)."""

    def __init__(self):
        super().__init__('arm_extension')
        for k, v in dict(move_group_action='move_action', group='', ee_link='',
                         base_frame='', max_velocity_scale=0.1, acceleration_scale=0.1,
                         planning_time=5.0, execute_timeout_sec=60.0, pos_tol=0.005,
                         ori_tol=0.05, workspace_min=[-1.0, -1.0, 0.0],
                         workspace_max=[1.0, 1.0, 1.2],
                         gripper_action='', gripper_open=0.08, gripper_closed=0.0,
                         gripper_max_effort=20.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.move = ActionClient(self, MoveGroup, p('move_group_action'))
        self.grip = (ActionClient(self, GripperCommand, p('gripper_action'))
                     if p('gripper_action') else None)
        self.handlers = {'MoveArmToPose': self._move, 'OpenGripper': self._open,
                         'CloseGripper': self._close}

    def _move(self, ports, ctl):
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        for k in ('group', 'ee_link', 'base_frame'):
            if not p(k):
                return False, f'parameter {k!r} not configured for this arm'
        xyz = (port_float(ports, 'x'), port_float(ports, 'y'), port_float(ports, 'z'))
        lo, hi = p('workspace_min'), p('workspace_max')
        if not all(l <= v <= h for v, l, h in zip(xyz, lo, hi)):
            return False, f'target {xyz} outside configured workspace {lo}..{hi}'
        rpy = None
        if any(ports.get(k) for k in ('roll', 'pitch', 'yaw')):
            rpy = tuple(port_float(ports, k, 0.0, lo=-math.pi * 2, hi=math.pi * 2)
                        for k in ('roll', 'pitch', 'yaw'))
        frame = ports.get('frame') or p('base_frame')
        vel = min(port_float(ports, 'speed', p('max_velocity_scale'), lo=0.01, hi=1.0),
                  p('max_velocity_scale'))
        goal = build_goal(p('group'), p('ee_link'), frame, xyz, rpy, vel,
                          p('acceleration_scale'), p('planning_time'), p('pos_tol'),
                          p('ori_tol'))
        res, err = call_action(self.move, goal, ctl, p('execute_timeout_sec'))
        if res is None:
            return False, err
        code = res.error_code.val
        return code == SUCCESS, 'moved' if code == SUCCESS else f'MoveIt error code {code}'

    def _gripper(self, ctl, position, effort):
        if self.grip is None:
            return False, 'gripper_action parameter not configured'
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = position, effort
        res, err = call_action(self.grip, goal, ctl, 15.0)
        if res is None:
            return False, err
        return (res.reached_goal or res.stalled), \
            'reached' if res.reached_goal else ('stalled (object held)' if res.stalled else 'not reached')

    def _open(self, ports, ctl):
        width = port_float(ports, 'width', self.get_parameter('gripper_open').value, lo=0.0)
        return self._gripper(ctl, width, self.get_parameter('gripper_max_effort').value)

    def _close(self, ports, ctl):
        mx = self.get_parameter('gripper_max_effort').value
        force = min(port_float(ports, 'force', mx, lo=0.0), mx)
        return self._gripper(ctl, self.get_parameter('gripper_closed').value, force)


def main(args=None):
    run_executor(ArmExtension, args)


if __name__ == '__main__':
    main()
