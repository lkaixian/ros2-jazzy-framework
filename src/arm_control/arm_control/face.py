"""How far to turn the base so an object lands where a camera fixed on the base sees it well. No ROS imports.

The Kobuki turns about its own centre, not about the arm axis (`pivot_x` m ahead of it), so turning also slides the arm sideways; the solve
includes that. Frames: the arm frame (x forward, y left, z up, origin on the joint-1 axis) and a turn `a` (rad, left positive).
"""
import math


def turn_to_face(x, y, y_target, pivot_x=0.16, max_turn=0.6, dead_band=0.05):
    """Turn (rad, left positive) that moves an object at arm-frame (x, y) to sideways `y_target`; 0.0 inside `dead_band`; None if it needs
    more than `max_turn` (or no turn gets there)."""
    px = x + pivot_x

    def ny(a):          # the object's sideways position in the arm frame after the base has turned by a
        return -math.sin(a) * px + math.cos(a) * y

    lo, hi = -1.0, 1.0
    if (ny(lo) - y_target) * (ny(hi) - y_target) > 0:
        return None
    for _ in range(60):
        mid = (lo + hi) / 2
        if (ny(lo) - y_target) * (ny(mid) - y_target) <= 0:
            hi = mid
        else:
            lo = mid
    a = (lo + hi) / 2
    if abs(a) > max_turn:
        return None
    return 0.0 if abs(a) < dead_band else a
