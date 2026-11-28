"""Pointing with the arm: the nearest pose to "pointing that way" that is inside the taught joint range and on a checked route. No ROS, no bus.

The wanted direction may be out of what the arm was taught (joint 1 only travels so far), or unreachable at one height. Instead of refusing, the search shrinks the
turn (100, 75, 50, 25, 0 %), the reach (0.30 to 0.15 m), the height and the finger pitch, and takes the FIRST pose that the caller's own checks accept (IK limited to
the taught range, route inside the free-space map). It never relaxes those checks: a pose they refuse is never used; it just looks for another one.
"""
import math


def candidates(want_rad):
    """(turn rad, (x, y, z) in the arm base frame, finger pitch) in order of preference: the exact direction first, then less and less turn."""
    for shrink in (1.0, 0.75, 0.5, 0.25, 0.0):
        a = want_rad * shrink
        for r in (0.30, 0.25, 0.20, 0.15):
            for z in (0.12, 0.18, 0.06, 0.24):
                for pitch in (0.0, 0.35, -0.35, None):
                    yield a, (r * math.cos(a), r * math.sin(a), z), pitch


def find_pose(want_rad, solve, route_ok, max_tries=400):
    """(joint dict, info) of the first candidate with solve(xyz, pitch) -> joints and route_ok(joints) -> True, else (None, info). info: turn, xyz, pitch, tries."""
    tries = 0
    for a, xyz, pitch in candidates(want_rad):
        tries += 1
        q = solve(xyz, pitch)
        if q is not None and route_ok(q):
            return q, {'turn': a, 'xyz': xyz, 'pitch': pitch, 'tries': tries}
        if tries >= max_tries:
            break
    return None, {'tries': tries}
