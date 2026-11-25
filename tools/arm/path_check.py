#!/usr/bin/env python3
"""Check a straight joint-space motion against the hand-taught collision-free map and the gravity-load estimate, BEFORE moving.

    python3 tools/arm/path_check.py --from rest --to home
    python3 tools/arm/path_check.py --from 1.19,2.06,-0.49,-0.03 --to rest      # names: rest home long high, or 4 angles (rad)

A motion is refused when any sample of it puts the arm outside the swept (free-space) region by more than TOLERANCE skeleton points,
leaves the joint ranges the map has seen, or when the estimated gravity load reaches the hard limit. IDEAL CASE: it knows nothing about
cables, things that moved since the map was made, motion or blocked joints. Unvisited space is "unknown", and unknown is refused.
"""
import argparse
import os
import sys

import numpy as np
from free_map import JOINTS, FreeMap, skeleton

TOLERANCE = 2          # skeleton points (of 40 per sample) allowed outside the map: interpolated poses graze its edge
STEP = 0.03            # rad between checked samples (largest joint)
RANGE_MARGIN = 0.10    # rad beyond the joint range the map has seen
STALL_NM, SOFT, HARD = 4.1, 0.4, 0.8
NAMED = {'home': (0.0, -1.0, 1.0, 0.0), 'long': (0.0, 1.38, -1.38, -0.04), 'high': (0.0, -0.14, -1.38, -0.04)}


def default_map():
    for name in ('~/arm_free_space_baseline.npz', '~/arm_free_space.npz'):
        if os.path.exists(os.path.expanduser(name)):
            return os.path.expanduser(name)
    return None


def sample_path(q0, q1, step=STEP):
    n = max(2, int(np.ceil(max(abs(q1[j] - q0[j]) for j in JOINTS) / step)) + 1)
    return [{j: q0[j] + s * (q1[j] - q0[j]) for j in JOINTS} for s in np.linspace(0, 1, n)]


def check_path(arm, fmap, model, q0, q1, tolerance=TOLERANCE, soft=SOFT, hard=HARD, stall=STALL_NM):
    """{'ok', 'reasons', 'worst_outside', 'first_bad' (fraction of the path or None), 'max_load', 'cost'} for the straight q0 -> q1."""
    from torque import cost
    samples = sample_path(q0, q1)
    reasons, first_bad, worst, max_u, total = [], None, 0, 0.0, 0.0
    for j in JOINTS:
        lo, hi = fmap.lo.get(j, 0.0) - RANGE_MARGIN, fmap.hi.get(j, 0.0) + RANGE_MARGIN
        if min(q0[j], q1[j]) < lo or max(q0[j], q1[j]) > hi:
            reasons.append(f'{j} leaves the range the map has seen ({fmap.lo.get(j, 0):+.2f}..{fmap.hi.get(j, 0):+.2f})')
    if any(abs(q[JOINTS[0]]) > np.pi for q in (q0, q1)):
        reasons.append('joint1 is more than half a turn from zero')
    for i, q in enumerate(samples):
        idx = np.floor(skeleton(arm, q) / fmap.v).astype(int)
        out = sum(tuple(c) not in fmap.cells for c in idx.tolist())
        if out > worst:
            worst = out
        if out > tolerance and first_bad is None:
            first_bad = i / (len(samples) - 1)
        u = max(abs(t) for k, t in model.torques(q).items() if k != JOINTS[0]) / stall
        max_u, total = max(max_u, u), total + cost(min(u, hard - 1e-6), 'poly', soft, hard)
    if first_bad is not None:
        reasons.append(f'leaves the swept free-space region {100 * first_bad:.0f}% along the path ({worst} of 40 skeleton points outside)')
    if max_u >= hard:
        reasons.append(f'estimated gravity load {100 * max_u:.0f}% of stall reaches the {100 * hard:.0f}% limit')
    return {'ok': not reasons, 'reasons': reasons, 'worst_outside': worst, 'first_bad': first_bad, 'max_load': max_u,
            'cost': total / len(samples)}


def parse_pose(text, rest):
    if text == 'rest':
        return dict(rest)
    if text in NAMED:
        return dict(zip(JOINTS, NAMED[text]))
    vals = [float(v) for v in text.split(',')]
    if len(vals) != 4:
        raise SystemExit(f'pose {text!r}: use rest, home, long, high or 4 comma-separated angles')
    return dict(zip(JOINTS, vals))


def verdict(q0, q1, map_path=None):
    """(ok, text) for q0 -> q1, or (None, why) when no map is available. Used by arm_tool before it moves the arm."""
    path = map_path or default_map()
    if path is None:
        return None, 'no free-space map found (~/arm_free_space*.npz)'
    from omx_fk import Arm
    from torque import TorqueModel
    arm = Arm()
    r = check_path(arm, FreeMap.load(path), TorqueModel(arm=arm), q0, q1)
    return r['ok'], '; '.join(r['reasons']) or f"inside the free-space map, load <= {100 * r['max_load']:.0f}% of stall"


def verdict_route(points, map_path=None, first_leg_max=0.35):
    """(ok, text) for a chain of straight legs through `points` (dicts of the 4 joints), or (None, why) without a map.
    The first leg starts at the arm's real pose, which may sit on the map's edge: it is only length-checked."""
    path = map_path or default_map()
    if path is None:
        return None, 'no free-space map found (~/arm_free_space*.npz)'
    from omx_fk import Arm
    from torque import TorqueModel
    arm = Arm()
    fmap, model = FreeMap.load(path), TorqueModel(arm=arm)
    for i, (a, b) in enumerate(zip(points, points[1:])):
        if i == 0:
            if max(abs(b[j] - a[j]) for j in JOINTS) > first_leg_max:
                return False, f'the first leg is longer than {first_leg_max} rad'
            continue
        r = check_path(arm, fmap, model, a, b)
        if not r['ok']:
            return False, f'leg {i + 1} of {len(points) - 1}: ' + '; '.join(r['reasons'])
    return True, f'{len(points) - 1} legs inside the free-space map'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--from', dest='src', required=True)
    ap.add_argument('--to', required=True)
    ap.add_argument('--map', default=None)
    a = ap.parse_args()
    from arm_tool import load_rest
    rest = load_rest()
    path = a.map or default_map() or sys.exit('no free-space map found')
    ok, text = verdict(parse_pose(a.src, rest), parse_pose(a.to, rest), path)
    print(('OK      ' if ok else 'REFUSED ') + f'{a.src} -> {a.to}: {text}   [map {os.path.basename(path)}]')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
