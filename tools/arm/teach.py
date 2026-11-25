#!/usr/bin/env python3
"""Digitise the robot body by hand: move the arm with torque OFF and read where the gripper centre is.

    python3 tools/arm/teach.py                      # print the pose now
    python3 tools/arm/teach.py board_front_top ...  # for each label: touch that spot with the gripper, press Enter
    python3 tools/arm/teach.py --out points.yaml board_front_top ...
    python3 tools/arm/teach.py --map                        # CONTINUOUS: live collision-free-region map, nothing to press, Ctrl+C = finish
    python3 tools/arm/teach.py --range [--trace path.csv]   # by hand: q = name this pose (saved at once), Ctrl+C = finish

Reads the servos directly (read-only, never writes, never enables torque), so the ROS driver must NOT be running.
Positions are in the arm base frame (x forward, y left, z up, metres) from the URDF kinematics,
good to about 1-2 cm (hand placement, gripper centre).
"""
import argparse
import collections
import csv
import glob
import subprocess
import sys
import time

import bus_lock
import yaml
from dynamixel_sdk import PacketHandler, PortHandler
from omx_fk import Arm, ticks_to_rad

_LOCK = None     # held for the whole run so no other tool talks to the servos at the same time
IDS = {'joint1': 11, 'joint2': 12, 'joint3': 13, 'joint4': 14}
STALL_NM = 4.1   # XM430-W350 stall torque at 12 V, from memory: verify against the datasheet
URDF_LIMITS = {'joint1': (-3.14, 3.14), 'joint2': (-1.5, 1.5), 'joint3': (-1.5, 1.4), 'joint4': (-1.7, 1.97)}


def read(port):
    pk = PacketHandler(2.0)
    q, torque = {}, []
    for name, i in IDS.items():
        pos, res, _ = pk.read4ByteTxRx(port, i, 132)          # Present Position
        if res != 0:
            raise SystemExit(f'ID {i} did not answer (12 V on? driver still running?)')
        q[name] = ticks_to_rad(pos - (1 << 32 if pos >= 1 << 31 else 0))
        torque.append(pk.read1ByteTxRx(port, i, 64)[0])       # Torque Enable
    return q, any(torque)


def unique(path):
    """path, or path with _2, _3 ... when it already exists: a rerun must not overwrite the previous run's trace."""
    import os
    root, ext = os.path.splitext(os.path.expanduser(path))
    n, cand = 2, root + ext
    while os.path.exists(cand):
        cand, n = f'{root}_{n}{ext}', n + 1
    return cand


HELP = """
  Move the arm BY HAND (torque is off).  Live readout below.
    q       you are at a limit / a touching point: the pose is captured NOW, then you are asked for a name
            (and what it touches). It is saved to {points} immediately, so nothing is lost if something goes wrong later.
    Ctrl+C  finish: prints the summary and saves the full trace to {trace}
"""


def save_point(points_file, name, note, q, p):
    """Append one pose to the points file the moment it is named (a YAML list, never overwritten)."""
    entry = {'name': name, 'note': note, 'joints': [round(v, 3) for v in q.values()],
             'gripper': [round(float(v), 3) for v in p], 'when': time.strftime('%F %T')}
    with open(points_file, 'a') as f:
        f.write('- ' + yaml.safe_dump(entry, default_flow_style=True, width=1000).strip() + '\n')   # appends stay one valid list


def sweep(port, arm, trace, points_file):
    """Sample the joints at ~10 Hz. `q` names the current pose; Ctrl+C finishes. The visited gripper positions are free space."""
    import os
    import select
    import termios
    import tty
    points_file, lo, hi, rows, saved = os.path.expanduser(points_file), {}, {}, [], []
    t0, last = time.monotonic(), 0.0
    print(HELP.format(points=points_file, trace=trace or '(no --trace given: not saved)'))
    interactive = sys.stdin.isatty()
    fd = sys.stdin.fileno() if interactive else None
    old = termios.tcgetattr(fd) if interactive else None
    if not interactive:
        print('stdin is not a terminal: `q` is unavailable here, Ctrl+C still finishes')
    try:
        if interactive:
            tty.setcbreak(fd)                       # keys arrive at once; Ctrl+C still raises KeyboardInterrupt
        while True:
            q, torque_on = read(port)
            if torque_on and not rows:
                print('WARNING: torque is ON on some servo: the arm will resist the hand')
            p = arm.xyz('end_effector_link', q)
            rows.append([round(time.monotonic() - t0, 2)] + [round(v, 4) for v in q.values()] + [round(float(v), 4) for v in p])
            for n, v in q.items():
                lo[n], hi[n] = min(lo.get(n, v), v), max(hi.get(n, v), v)
            if time.monotonic() - last > 0.25:
                last = time.monotonic()
                print('\r  ' + '  '.join(f'j{i + 1} {v:+.2f}' for i, v in enumerate(q.values()))
                      + f'  | gripper x {p[0]:+.3f} y {p[1]:+.3f} z {p[2]:+.3f}  | {len(saved)} saved   ', end='', flush=True)
            if interactive and select.select([sys.stdin], [], [], 0)[0] and os.read(fd, 1) in (b'q', b'Q'):
                frozen_q, frozen_p = dict(q), p     # the pose at the moment of the key press, before any typing
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                try:
                    name = input('\n  Name this point (Enter = point_N): ').strip() or f'point_{len(saved) + 1}'
                    note = input('  What does it touch / why is it a limit? (Enter to skip): ').strip()
                except KeyboardInterrupt:           # Ctrl+C while naming: keep the pose, then finish
                    name, note = f'point_{len(saved) + 1}', ''
                    save_point(points_file, name, note, frozen_q, frozen_p)
                    saved.append((name, list(frozen_q.values())))
                    raise
                save_point(points_file, name, note, frozen_q, frozen_p)
                saved.append((name, list(frozen_q.values())))
                print(f'  saved "{name}" -> {points_file}   (keep moving; q = another point, Ctrl+C = finish)')
                tty.setcbreak(fd)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if interactive:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print(f'\n\n{len(rows)} samples. joint: reached min .. max (rad)   URDF limits')
    for n in IDS:
        out = lo[n] < URDF_LIMITS[n][0] - 0.02 or hi[n] > URDF_LIMITS[n][1] + 0.02
        flag = '   <- beyond the URDF limit' if out else ''
        print(f'  {n}: {lo[n]:+.2f} .. {hi[n]:+.2f}   {URDF_LIMITS[n][0]:+.2f} .. {URDF_LIMITS[n][1]:+.2f}{flag}')
    for name, qs in saved:
        print(f'  saved point "{name}": joints ' + ' '.join(f'{v:+.2f}' for v in qs))
    if saved:
        print('points file:', points_file)
    if trace and rows:
        trace = unique(trace)
        with open(trace, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', *IDS, 'ee_x', 'ee_y', 'ee_z'])
            w.writerows(rows)
        print('trace saved', trace)


def stream_map(port, arm, map_file, fresh, stream_path='~/arm_stream.csv', maps_dir='~/arm_maps'):
    """Continuous mode: sample at ~15 Hz while you move the arm by hand, grow the cumulative collision-free-region map, show it live."""
    import os
    from free_map import FreeMap
    from torque import TorqueModel
    map_file = os.path.expanduser(map_file)
    if fresh and os.path.exists(map_file):
        os.rename(map_file, unique(map_file))
    m = FreeMap.load(map_file) if os.path.exists(map_file) else FreeMap()
    start, stream = m.stats(), unique(stream_path)
    tm, peak = TorqueModel(arm=arm), 0.0
    recent, t0, last_draw, last_save, last_flush = collections.deque(), time.monotonic(), 0.0, time.monotonic(), time.monotonic()
    f = open(stream, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', *IDS, 'ee_x', 'ee_y', 'ee_z', 'load_est'])
    print('\x1b[2J', end='')
    try:
        while True:
            q, torque_on = read(port)
            p = arm.xyz('end_effector_link', q)
            now = time.monotonic()
            recent.append((now, m.add(arm, q)))
            load = max(abs(t) for k, t in tm.torques(q).items() if k != 'joint1') / STALL_NM
            peak = max(peak, load)
            while recent and now - recent[0][0] > 30:
                recent.popleft()
            w.writerow([round(now - t0, 2)] + [round(v, 4) for v in q.values()] + [round(float(v), 4) for v in p] + [round(load, 3)])
            if now - last_flush > 1:
                f.flush()
                last_flush = now
            if now - last_save > 5:
                m.save(map_file)
                last_save = now
            if now - last_draw > 0.5:
                last_draw, st = now, m.stats()
                lines = m.render(cur=tuple(p)) + ['',
                    f"free volume {st['volume_l']:6.1f} L  (+{st['volume_l'] - start['volume_l']:.1f} this run)   top-view area {st['top_cm2']:6.0f} cm2"
                    f"   side-view area {st['side_cm2']:6.0f} cm2",
                    f"samples {st['samples']}   newly opened in the last 30 s: {sum(n for _, n in recent)} voxels (0 = nothing new, you are repeating yourself)",
                    f'gravity load estimate {100 * load:.0f}% of stall (peak {100 * peak:.0f}%): ideal case, ignores motion, cable pull, blocked joints.'
                    ' The map is collision-free only, NOT a load limit.',
                    'joints now ' + ' '.join(f'{v:+.2f}' for v in q.values()) + '   seen ' + '  '.join(
                        f'j{i + 1} {m.lo[n]:+.2f}..{m.hi[n]:+.2f}' for i, n in enumerate(IDS)),
                    'WARNING: torque is ON: the arm resists the hand' if torque_on else 'torque off: free to move by hand. Ctrl+C = finish and save',
                    'WARNING: a joint is more than half a turn from zero: turn it back before the driver is started' if any(abs(v) > 3.14 for v in q.values()) else '',
                    f'autosaving to {map_file}']
                print('\x1b[H' + '\n'.join(l + '\x1b[K' for l in lines), end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        m.save(map_file)
    st = m.stats()
    print(f"\n\nfree-space region: {st['volume_l']:.1f} L (+{st['volume_l'] - start['volume_l']:.1f} this run), top-view area {st['top_cm2']:.0f} cm2, "
          f"side-view area {st['side_cm2']:.0f} cm2, {st['samples']} samples")
    if st['extent']:
        print('extent (m): ' + '  '.join(f'{k} {lo:+.2f}..{hi:+.2f}' for k, (lo, hi) in st['extent'].items()))
    png = plot_map(m, st, maps_dir)
    print('map saved', map_file, '| stream', stream, '| picture', png)


def plot_map(m, st, maps_dir='~/arm_maps'):
    import os

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    c = (np.array(list(m.cells)) + 0.5) * m.v
    fig, ax = plt.subplots(1, 3, figsize=(15, 5.5))
    for a, (i, j, xl, yl, t) in zip(ax, ((1, 0, 'y (left +) m', 'x (forward +) m', 'TOP'), (0, 2, 'x (forward +) m', 'z (up +) m', 'SIDE'),
                                         (1, 2, 'y (left +) m', 'z (up +) m', 'FRONT'))):
        a.scatter(c[:, i], c[:, j], s=6, marker='s', c='#2a78d6', alpha=.5, linewidths=0)
        a.plot(0, 0, 'k+', ms=14)
        a.set_aspect('equal')
        a.grid(alpha=.3)
        a.set_xlabel(xl)
        a.set_ylabel(yl)
        a.set_title(t)
        if t == 'TOP':
            a.invert_xaxis()
    plt.suptitle(f"Collision-free region swept by the whole arm (ideal case: no load, no overload considered)\n"
                f"{st['volume_l']:.1f} L, top-view area {st['top_cm2']:.0f} cm2   (blue = swept without contact, white = unknown; NOT a load limit)")
    plt.tight_layout()
    out = os.path.expanduser(maps_dir)
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, 'free_space_' + time.strftime('%H%M%S') + '.png')
    plt.savefig(path, dpi=100)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('labels', nargs='*')
    ap.add_argument('--out')
    ap.add_argument('--map', action='store_true', help='continuous live collision-free-region map (nothing to press; Ctrl+C finishes)')
    ap.add_argument('--map-file', default='~/arm_free_space.npz', help='cumulative map, grows across runs')
    ap.add_argument('--stream', default='~/arm_stream.csv', help='with --map: raw sample stream of this run (never overwritten)')
    ap.add_argument('--maps-dir', default='~/arm_maps', help='with --map: where the picture is written')
    ap.add_argument('--fresh', action='store_true', help='with --map: keep the old map under a new name and start empty')
    ap.add_argument('--range', action='store_true', help='log the joint extremes while you move the arm by hand')
    ap.add_argument('--trace', help='with --range: CSV of every sample (joints and gripper position)')
    ap.add_argument('--points', default='~/arm_points.yaml', help='with --range: named points are appended here')
    a = ap.parse_args()
    if subprocess.run(['pgrep', '-f', 'lib/controller_manager/[r]os2_control_node'], capture_output=True).returncode == 0:
        sys.exit('the ROS driver is running: stop it first (this tool needs the serial port, and torque off)')
    global _LOCK
    _LOCK = bus_lock.acquire()
    if _LOCK is None:
        sys.exit('the arm serial port is busy (another arm tool or the GUI polling): try again')
    dev = (glob.glob('/dev/serial/by-id/*OpenCR*') or [''])[0]
    port = PortHandler(dev)
    if not dev or not port.openPort() or not port.setBaudRate(1000000):
        sys.exit('cannot open the OpenCR port')
    arm, pts = Arm(), {}
    try:
        if a.map:
            stream_map(port, arm, a.map_file, a.fresh, a.stream, a.maps_dir)
            return
        if a.range:
            sweep(port, arm, a.trace, a.points)
            return
        for label in a.labels or [None]:
            if label:
                input(f'Touch "{label}" with the gripper centre, then press Enter... ')
            q, torque_on = read(port)
            p = arm.xyz('end_effector_link', q)
            tag = '  (TORQUE IS ON: the arm is not free to guide)' if torque_on else ''
            print(f'{label or "now"}: joints ' + ' '.join(f'{v:+.2f}' for v in q.values())
                  + f' -> gripper x {p[0]:+.3f} y {p[1]:+.3f} z {p[2]:+.3f} m{tag}')
            if label:
                pts[label] = [round(float(v), 3) for v in p]
    finally:
        port.closePort()
    if a.out and pts:
        with open(a.out, 'w') as f:
            yaml.safe_dump({'points': pts, 'frame': 'arm base (URDF world): x forward, y left, z up, m'}, f)
        print('saved', a.out)


if __name__ == '__main__':
    main()
