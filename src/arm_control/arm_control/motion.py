"""Guarded arm motion planning (no ROS): which joint-space path may be driven, and when each point is reached.

A path is allowed only if the free-space map says so, else if a route made of hand-taught poses does. No map, or no such route, is a
refusal: this is the pick path, so there is no "corridor" fallback.
"""
from arm_control.free_map import JOINTS

RATE = 0.03          # rad/s on the joint that travels furthest: the speed the park was run at on the real arm. Raise only after validation.
MIN_LEG_SEC = 1.0


class Refused(Exception):
    pass


def plan_route(cur, goal, verdict=None, taught=None, map_path=None):
    """List of joint dicts from `cur` to `goal`. Raises Refused.
    `verdict(q0, q1)` -> (ok, text); `taught(cur, goal)` -> (points|None, text)."""
    if verdict is None:
        from arm_control.path_check import verdict
    cur, goal = {j: cur[j] for j in JOINTS}, {j: goal[j] for j in JOINTS}
    ok, why = verdict(cur, goal, map_path)
    if ok is True:
        return [cur, goal]
    if ok is None:
        raise Refused(why)
    pts, how = (taught or _taught)(cur, goal, map_path)
    if pts is None:
        raise Refused(f'the straight path leaves the free-space map ({why}) and there is no taught route: {how}')
    return pts


def _taught(cur, goal, map_path=None):
    from arm_control.path_check import verdict_route
    from arm_control.route import load_roadmap, plan
    wps, how = plan(*load_roadmap(), cur, goal)
    if wps is None:
        return None, how
    pts = [dict(zip(JOINTS, w)) for w in wps]
    ok, text = verdict_route(pts, map_path)
    return (pts, how) if ok is True else (None, f'the taught route is not inside the free-space map ({text})')


def schedule(points, rate=RATE):
    """[(positions, t_sec)] for the legs of `points` at a constant `rate` (rad/s, largest joint) starting from points[0]."""
    t, out = 0.0, []
    for a, b in zip(points, points[1:]):
        t += max(MIN_LEG_SEC, max(abs(b[j] - a[j]) for j in JOINTS) / rate)
        out.append(([b[j] for j in JOINTS], t))
    return out
