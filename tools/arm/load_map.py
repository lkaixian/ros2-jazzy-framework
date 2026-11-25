#!/usr/bin/env python3
"""Load map over the collision-free region: for every 2 cm cell the gripper centre has visited (hand-guided, no contact), the BEST (min)
and WORST (max) estimated gravity load of the poses that put it there, as a fraction of the servo stall torque.

    python3 tools/arm/load_map.py                       # uses ~/arm_stream*.csv and ~/arm_range*.csv
    python3 tools/arm/load_map.py a.csv b.csv --payload 0.2

MODEL ESTIMATE from torque.py, NOT a measurement (torque was off while the arm was hand-guided).
It ignores motion, friction, cable pull and blocked joints, which is how the arm actually
overloaded, so a low number here is necessary, not sufficient. Cells hold only postures that were really visited: a cell with one
awkward posture shows a high 'best' load although a better posture might exist.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

STALL_NM = 4.1      # XM430-W350 stall torque at 12 V, from memory: verify against the datasheet
VOXEL = 0.02


def load_cells(files, model, stall, stride=2):
    """{(i, j, k): [min, max, n]} of the gripper cell -> load of the poses seen there."""
    cells = {}
    for path in files:
        for n, r in enumerate(csv.DictReader(open(path))):
            if n % stride:
                continue
            q = {j: float(r[j]) for j in ('joint1', 'joint2', 'joint3', 'joint4')}
            u = max(abs(t) for k, t in model.torques(q).items() if k != 'joint1') / stall
            c = tuple(int(np.floor(float(r[a]) / VOXEL)) for a in ('ee_x', 'ee_y', 'ee_z'))
            e = cells.setdefault(c, [u, u, 0])
            e[0], e[1], e[2] = min(e[0], u), max(e[1], u), e[2] + 1
    return cells


def summarise(cells, soft=0.4, hard=0.8):
    a = np.array([[v[0], v[1]] for v in cells.values()]).reshape(-1, 2)
    return {'cells': len(a), 'volume_l': len(a) * VOXEL ** 3 * 1000, 'best_max': float(a[:, 0].max()) if len(a) else 0.0,
            'worst_max': float(a[:, 1].max()) if len(a) else 0.0, 'best_under_soft': float((a[:, 0] < soft).mean()) if len(a) else 0.0,
            'worst_under_soft': float((a[:, 1] < soft).mean()) if len(a) else 0.0, 'best_over_hard': int((a[:, 0] >= hard).sum())}


def plot(cells, path, st):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    keys = np.array(list(cells)) * VOXEL + VOXEL / 2
    mn, mx = (np.array([v[i] for v in cells.values()]) for i in (0, 1))
    fig, ax = plt.subplots(2, 3, figsize=(15, 9))
    for row, (val, title) in enumerate(((mn, 'BEST posture load'), (mx, 'WORST posture load'))):
        for col, (i, j, xl, yl, t) in enumerate(((1, 0, 'y (left +) m', 'x (forward +) m', 'TOP'), (0, 2, 'x (forward +) m', 'z (up +) m', 'SIDE'),
                                                 (1, 2, 'y (left +) m', 'z (up +) m', 'FRONT'))):
            a = ax[row][col]
            o = np.argsort(val)                       # draw the worst cell last so it is not hidden
            s = a.scatter(keys[o, i], keys[o, j], c=val[o], s=14, marker='s', cmap='RdYlGn_r', vmin=0, vmax=0.8, linewidths=0)
            a.plot(0, 0, 'k+', ms=14)
            a.set_aspect('equal')
            a.grid(alpha=.3)
            a.set_xlabel(xl)
            a.set_ylabel(yl)
            a.set_title(f'{t}: {title}')
            if t == 'TOP':
                a.invert_xaxis()
    fig.colorbar(s, ax=ax, label='gravity torque / stall torque (4.1 N*m)', shrink=.7)
    plt.suptitle(f"MODEL ESTIMATE, NOT MEASURED (the servos had torque off while the arm was hand-guided): gravity load over {st['cells']} "
                 f"collision-free gripper cells, {st['volume_l']:.0f} L.\nIgnores motion, cable pull, blocked joints and the true payload")
    plt.savefig(path, dpi=100, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('files', nargs='*')
    ap.add_argument('--payload', type=float, default=0.15, help='kg on the gripper: OAK + bracket + share of cable (a guess)')
    ap.add_argument('--stall', type=float, default=STALL_NM)
    ap.add_argument('--soft', type=float, default=0.4)
    ap.add_argument('--hard', type=float, default=0.8)
    ap.add_argument('--out', default='~/arm_load_map.npz')
    ap.add_argument('--maps-dir', default='~/arm_maps')
    a = ap.parse_args()
    files = a.files or sorted(glob.glob(os.path.expanduser('~/arm_stream*.csv')) + glob.glob(os.path.expanduser('~/arm_range*.csv')))
    if not files:
        sys.exit('no trace files found (run teach.py --map or --range first)')
    from torque import TorqueModel
    cells = load_cells(files, TorqueModel(payload_kg=a.payload), a.stall)
    st = summarise(cells, a.soft, a.hard)
    keys = np.array(list(cells), dtype=np.int32)
    np.savez_compressed(os.path.expanduser(a.out), cells=keys, mn=[v[0] for v in cells.values()], mx=[v[1] for v in cells.values()],
                        n=[v[2] for v in cells.values()], voxel=VOXEL, payload=a.payload, stall=a.stall)
    os.makedirs(os.path.expanduser(a.maps_dir), exist_ok=True)
    png = os.path.join(os.path.expanduser(a.maps_dir), 'load_map.png')
    plot(cells, png, st)
    print(f"{len(files)} file(s), {st['cells']} gripper cells = {st['volume_l']:.0f} L (payload {a.payload} kg, stall {a.stall} N*m)")
    print(f"best-posture load: max {100 * st['best_max']:.0f}% of stall; worst-posture load: max {100 * st['worst_max']:.0f}%")
    print(f"cells under {100 * a.soft:.0f}% load: {100 * st['best_under_soft']:.0f}% by best posture, {100 * st['worst_under_soft']:.0f}% by worst posture; "
          f"cells at or over {100 * a.hard:.0f}% (best posture): {st['best_over_hard']}")
    print('saved', os.path.expanduser(a.out), '| picture', png)


if __name__ == '__main__':
    main()
