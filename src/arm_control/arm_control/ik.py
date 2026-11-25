"""Numeric inverse kinematics for the OpenMANIPULATOR-X (4 joints: a position plus the gripper pitch). Pure maths on omx_fk.

Gripper pitch = angle of the finger axis (gripper x) above the horizontal; negative points down. IK only proposes joint angles:
whether the arm may move there is path_check's call (free-space map), never this module's.
"""
import math

import numpy as np

from arm_control.free_map import JOINTS

EE = 'end_effector_link'
LIMITS = {'joint1': (-3.14, 3.14), 'joint2': (-1.5, 1.5), 'joint3': (-1.4, 1.4), 'joint4': (-1.7, 1.7)}   # URDF, rad
SEEDS = [(0.0, 0.0, 0.0, 0.0), (0.0, -0.5, 0.5, 0.0), (0.0, 0.5, -0.5, 0.0), (0.0, -1.0, 1.0, 0.0)]
POS_TOL = 0.003   # m: a solution further than this from the target is rejected


def map_limits(fmap):
    """Joint limits = exactly the range the arm was hand-taught through (no margin), clipped to the URDF. Raises if the map has none."""
    if not all(j in fmap.lo and j in fmap.hi for j in JOINTS):
        raise ValueError('the free-space map has no joint range: refusing to solve IK without one')
    return {j: (max(fmap.lo[j], LIMITS[j][0]), min(fmap.hi[j], LIMITS[j][1])) for j in JOINTS}


def _task(arm, q):
    t = arm.pose(EE, dict(zip(JOINTS, q)))
    x = t[:3, 0]
    return np.array([t[0, 3], t[1, 3], t[2, 3], math.atan2(x[2], math.hypot(x[0], x[1]))])


def solve(arm, xyz, pitch=None, seeds=None, limits=None, iters=120):
    """Pass `limits=map_limits(fmap)` on the real arm so a solution can never leave the taught range. Joint dict reaching `xyz` (m, arm base frame) at finger pitch `pitch` (rad; None = any, the closest to the seed), or None."""
    lim = np.array([(limits or LIMITS)[j] for j in JOINTS])
    want = np.array(list(xyz) + [0.0])
    rows = 4 if pitch is not None else 3
    if pitch is not None:
        want[3] = pitch
    best = None
    for s in (seeds or SEEDS):
        q = np.clip(np.array(s, float), lim[:, 0], lim[:, 1])
        q[0] = math.atan2(xyz[1], xyz[0])                 # the base joint just faces the target
        for _ in range(iters):
            err = (want - _task(arm, q))[:rows]
            if np.linalg.norm(err[:3]) < 1e-4 and (rows == 3 or abs(err[3]) < 1e-3):
                break
            jac = np.empty((rows, 4))
            for i in range(4):
                d = np.zeros(4)
                d[i] = 1e-5
                jac[:, i] = (_task(arm, q + d) - _task(arm, q))[:rows] / 1e-5
            q = np.clip(q + jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(rows), err), lim[:, 0], lim[:, 1])
        e = np.linalg.norm((want - _task(arm, q))[:3])
        if e < POS_TOL and (best is None or e < best[0]):
            best = (e, q)
    return None if best is None else dict(zip(JOINTS, (float(v) for v in best[1])))
