"""Where to grasp: a detection pixel plus depth -> approach and grasp poses (joint angles) that stay inside the taught range.

Pure maths. Whether the arm may drive there is motion.plan_route's decision, taken for every leg before anything moves.
IDEAL CASE: depends on the unverified hand-eye numbers (param/arm_hand_eye.yaml `verified: false`) and a depth reading of the front surface.
"""
import math

import numpy as np

from arm_control.hand_eye import upright_pixel_to_arm
from arm_control.ik import SEEDS, solve
from arm_control.motion import Refused


def grasp_poses(arm, cur, u, v, depth_m, width, height, limits, ws_min, ws_max, offset=0.03, approach=0.06, pitch=0.0):
    """{'point': xyz, 'pre': joint dict, 'grasp': joint dict}. `offset` pushes the front-surface point back to the object's middle,
    `approach` is how far the pre-grasp stands off toward the base, `pitch` is the finger angle above horizontal (rad)."""
    p = np.array(upright_pixel_to_arm(arm, cur, u, v, depth_m, width, height), float)
    r = math.hypot(p[0], p[1])
    if r < 0.05:
        raise Refused(f'target {np.round(p, 3).tolist()} is above the arm base')
    h = np.array([p[0] / r, p[1] / r, 0.0])
    return approach_poses(arm, cur, p + h * offset, limits, ws_min, ws_max, approach, pitch)


def approach_poses(arm, cur, grasp, limits, ws_min, ws_max, approach=0.06, pitch=0.0):
    """{'point', 'pre', 'grasp'} for a target point: the pre pose stands off `approach` m toward the base, horizontally."""
    grasp = np.array(grasp, float)
    r = math.hypot(grasp[0], grasp[1])
    if r < 0.05:
        raise Refused(f'target {np.round(grasp, 3).tolist()} is above the arm base')
    pre = grasp - np.array([grasp[0] / r, grasp[1] / r, 0.0]) * approach
    for name, q in (('grasp', grasp), ('pre-grasp', pre)):
        if not all(lo <= x <= hi for x, lo, hi in zip(q, ws_min, ws_max)):
            raise Refused(f'{name} point {np.round(q, 3).tolist()} is outside the configured workspace {list(ws_min)}..{list(ws_max)}')
    seeds = [tuple(cur[j] for j in ('joint1', 'joint2', 'joint3', 'joint4'))] + SEEDS
    q_pre = solve(arm, pre, pitch, seeds=seeds, limits=limits)
    q_grasp = q_pre and solve(arm, grasp, pitch, seeds=[tuple(q_pre.values())] + seeds, limits=limits)
    if not q_pre or not q_grasp:
        raise Refused(f'no arm pose inside the taught joint range reaches {np.round(grasp, 3).tolist()} at finger pitch {math.degrees(pitch):.0f} deg')
    return {'point': grasp.tolist(), 'pre': q_pre, 'grasp': q_grasp}
