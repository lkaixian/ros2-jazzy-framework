"""Geometry for RetraceTrail: drive back along the crumbs FollowTarget recorded, newest to oldest. ROS-free.

shortcut: odometry only (drifts over tens of metres), nearest-crumb progress limited to a small window so a path that loops back past the robot
does not skip ahead; no obstacle detour (the caller stops and gives up when blocked).
"""
import math

TURN_IN_PLACE_RAD = 0.8     # heading error beyond this: no forward speed, turn on the spot first


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def retrace_step(pose, path, idx, lookahead=0.6, arrive_m=0.4, window=10):
    """pose = (x, y, yaw); path = crumbs in driving order (newest first); idx = progress so far.
    Returns (idx, target, err, done): the progress index, the crumb to steer at (None when done or empty), the heading error to it
    (rad, + = target to the left), and whether the robot is within arrive_m of the last crumb."""
    if not path:
        return idx, None, 0.0, False
    x, y, yaw = pose
    d = lambda c: math.hypot(c[0] - x, c[1] - y)   # noqa: E731
    idx = min(range(idx, min(len(path), idx + window)), key=lambda i: d(path[i]))
    last = len(path) - 1
    target = next((c for c in path[idx:] if d(c) >= lookahead), path[last])
    if target is path[last] and d(target) <= arrive_m:
        return idx, None, 0.0, True
    return idx, target, wrap(math.atan2(target[1] - y, target[0] - x) - yaw), False


def forward_speed(err, speed):
    """Full speed straight at the crumb, falling to 0 at TURN_IN_PLACE_RAD."""
    return speed * max(0.0, 1.0 - abs(err) / TURN_IN_PLACE_RAD)
