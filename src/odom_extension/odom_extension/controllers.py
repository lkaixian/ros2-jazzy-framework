"""Pure motion controllers on (x, y, yaw) odometry poses. No ROS imports."""
import math
from dataclasses import dataclass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lim):
    return max(-lim, min(lim, x))


@dataclass
class Limits:
    max_v: float = 0.2
    max_w: float = 0.6
    kv: float = 0.8
    kw: float = 1.5
    yaw_tol: float = 0.05
    align_rad: float = 0.5     # drive only when heading error is below this


def goto_step(pose, goal_xy, goal_yaw, tol, lim, max_v=None):
    """One control step -> (v, w, done)."""
    x, y, yaw = pose
    dx, dy = goal_xy[0] - x, goal_xy[1] - y
    dist = math.hypot(dx, dy)
    vmax = min(lim.max_v, max_v) if max_v else lim.max_v
    if dist > tol:
        err = wrap(math.atan2(dy, dx) - yaw)
        w = clamp(lim.kw * err, lim.max_w)
        v = clamp(lim.kv * dist, vmax) * max(0.0, math.cos(err)) if abs(err) < lim.align_rad else 0.0
        return v, w, False
    if goal_yaw is None:
        return 0.0, 0.0, True
    err = wrap(goal_yaw - yaw)
    if abs(err) <= lim.yaw_tol:
        return 0.0, 0.0, True
    return 0.0, clamp(lim.kw * err, lim.max_w), False


class DriveDistance:
    def __init__(self, start, meters, lim, speed=None, tol=0.01):
        self.start, self.meters, self.lim, self.tol = start, meters, lim, tol
        self.vmax = min(lim.max_v, speed) if speed else lim.max_v

    def step(self, pose):
        travelled = math.hypot(pose[0] - self.start[0], pose[1] - self.start[1])
        remaining = abs(self.meters) - travelled
        if remaining <= self.tol:
            return 0.0, 0.0, True
        v = clamp(self.lim.kv * remaining, self.vmax)
        return (v if self.meters >= 0 else -v), 0.0, False


class RotateBy:
    def __init__(self, start_yaw, radians, lim, speed=None, tol=0.03):
        self.last, self.target, self.acc = start_yaw, radians, 0.0
        self.lim, self.tol = lim, tol
        self.wmax = min(lim.max_w, speed) if speed else lim.max_w

    def step(self, pose):
        self.acc += wrap(pose[2] - self.last)     # unwrapped: supports |angle| > pi
        self.last = pose[2]
        remaining = self.target - self.acc
        if abs(remaining) <= self.tol:
            return 0.0, 0.0, True
        return 0.0, clamp(self.lim.kw * remaining, self.wmax), False
