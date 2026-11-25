#!/usr/bin/env python3
"""Staged STATIC load test: hold the arm at poses of rising predicted load and read what each servo really draws, to check the torque model.

    python3 tools/arm/load_test.py              # DRY RUN: read-only preflight + the plan + the abort rules. Writes nothing to the servos.
    python3 tools/arm/load_test.py --fake       # run the whole procedure against a SIMULATED bus (no hardware)
    python3 tools/arm/load_test.py --go         # real run: needs the ROS driver stopped, the arm at rest, a support under the gripper

Safety design: poses come only from hand-taught streams (never near a recorded contact or a mechanical stop), the arm travels only along
taught routes, slowly, lightest pose first. Every 50 ms each servo is checked; ANY of these freezes the arm where it is (torque stays ON,
nothing drops): hardware error flag, current above 50 % of the limit, position sag, over-temperature, low voltage, a read failure, or a
current far above the model's prediction. Then you choose: [p] park along a taught route, [q] torque off (a raised arm FALLS).
Gravity only: it says nothing about motion, cable snags or blocked joints. Needs the Dynamixel SDK (ROS Jazzy install).
"""
import argparse
import csv
import os
import subprocess
import sys
import tempfile
import time

import bus_lock
import numpy as np
import telemetry
from free_map import JOINTS

TICK = 0.05                      # s between control ticks
A_PER_UNIT = 0.00269             # XM430 Present Current unit, A (datasheet, from memory)
LIMIT_UNITS = 1193               # servo Current Limit register (read from your servos)
ABORT_UNITS = int(0.5 * LIMIT_UNITS)
KT_SPEC = 4.1 / 2.3              # N*m per A from stall torque / stall current (from memory: verify against the datasheet)
ERR_HOLD, ERR_MOVE = 0.08, 0.25  # rad of position error that counts as sag
SETTLE = 1.5                     # s allowed to settle before the hold checks apply
TEMP_MAX, TEMP_RISE, V_MIN = 55, 8, 10.5
HEAVY_RATIO, HEAVY_MARGIN_A = 3.0, 0.35
IDS = dict(zip(JOINTS, (11, 12, 13, 14)))
TICK_RAD = 2 * np.pi / 4096


class Abort(Exception):
    pass


class BusError(Exception):
    pass


# ----------------------------------------------------------------------------------------------------- buses
class RealBus:
    """Dynamixel SDK on the OpenCR serial port (the ROS driver must NOT be running)."""

    def __init__(self, port, lock=True):
        import glob

        from dynamixel_sdk import PacketHandler, PortHandler
        self.lock = bus_lock.acquire() if lock else True        # the GUI polls with its own short-lived lock
        if self.lock is None:
            raise BusError('the arm serial port is busy (another arm tool or the GUI polling): try again')
        dev = port or (glob.glob('/dev/serial/by-id/*OpenCR*') or [''])[0]
        self.port, self.pk = PortHandler(dev), PacketHandler(2.0)
        if not dev or not self.port.openPort() or not self.port.setBaudRate(1000000):
            raise BusError('cannot open the OpenCR port')

    def _rd(self, i, addr, n):
        data, res, err = self.pk.readTxRx(self.port, i, addr, n)
        if res != 0:
            raise BusError(f'ID {i}: no answer')
        return data, err

    def read(self):
        pos, cur, hw, temp, volt = ([0] * 4 for _ in range(5))
        for k, i in enumerate(IDS.values()):
            blk, _ = self._rd(i, 126, 21)               # 126 current .. 146 temperature in one transaction
            cur[k] = int.from_bytes(bytes(blk[0:2]), 'little', signed=True)
            pos[k] = int.from_bytes(bytes(blk[6:10]), 'little', signed=True)
            volt[k] = int.from_bytes(bytes(blk[18:20]), 'little') / 10.0
            temp[k] = blk[20]
            hw[k] = self._rd(i, 70, 1)[0][0]
        return {'pos': (np.array(pos) - 2048) * TICK_RAD, 'cur': np.array(cur), 'err': np.array(hw), 'temp': np.array(temp), 'volt': np.array(volt)}

    def setup(self, vel=15, acc=5):
        for i in IDS.values():
            self.pk.write4ByteTxRx(self.port, i, 112, vel)
            self.pk.write4ByteTxRx(self.port, i, 108, acc)

    def goal(self, q):
        for i, v in zip(IDS.values(), q):
            self.pk.write4ByteTxRx(self.port, i, 116, int(round(2048 + v / TICK_RAD)))

    def enable(self):
        self.goal(self.read()['pos'])                   # goal = where it is NOW, so torque-on cannot jump
        for i in IDS.values():
            self.pk.write1ByteTxRx(self.port, i, 64, 1)

    def disable(self):
        for i in IDS.values():
            self.pk.write1ByteTxRx(self.port, i, 64, 0)

    def close(self):
        self.port.closePort()


class FakeBus:
    """Simulated servos: follows goals at a profile speed, draws current from the torque model times `scale`; optional overload fault."""

    def __init__(self, model, q0, scale=1.0, fault_units=None):
        self.model, self.q, self.g, self.scale, self.fault, self.on = model, np.array(q0, float), np.array(q0, float), scale, fault_units, False
        self.err = np.zeros(4, int)

    def read(self):
        d = np.clip(self.g - self.q, -0.4 * TICK, 0.4 * TICK) if self.on else 0
        self.q = self.q + d
        tau = self.model.torques(dict(zip(JOINTS, self.q)))
        cur = np.array([abs(tau[j]) / KT_SPEC / A_PER_UNIT * self.scale for j in JOINTS])
        if self.fault and self.on:
            hit = cur > self.fault
            if hit.any():
                self.err[hit], self.on = 32, False
        return {'pos': self.q.copy(), 'cur': cur, 'err': self.err.copy(), 'temp': np.full(4, 30), 'volt': np.full(4, 12.0)}

    def setup(self, vel=15, acc=5):
        pass

    def goal(self, q):
        self.g = np.array(q, float)

    def enable(self):
        self.g, self.on = self.q.copy(), True

    def disable(self):
        self.on = False

    def close(self):
        pass


def send_arm(st):
    """Stream one servo reading to the GUI."""
    telemetry.send('arm', pos=[round(float(v), 4) for v in st['pos']], cur=[int(v) for v in st['cur']], temp=[int(v) for v in st['temp']],
                   volt=[float(v) for v in st['volt']], err=[int(v) for v in st['err']])


# ------------------------------------------------------------------------------------------- safety rules
def predicted_amps(model, pos):
    return np.array([abs(model.torques(dict(zip(JOINTS, pos)))[j]) / KT_SPEC for j in JOINTS])


def abort_reason(st, goal, kind, pred_a=None, start_temp=None):
    """Why the test must stop now, or None. kind: 'move' | 'hold'."""
    for i, j in enumerate(JOINTS):
        if st['err'][i]:
            return f'{j}: hardware error flag {int(st["err"][i])}'
        if st['volt'][i] < V_MIN:
            return f'{j}: supply voltage {st["volt"][i]:.1f} V is below {V_MIN} V'
        if st['temp'][i] > TEMP_MAX or (start_temp is not None and st['temp'][i] - start_temp[i] > TEMP_RISE):
            return f'{j}: temperature {int(st["temp"][i])} C'
        if abs(st['cur'][i]) > ABORT_UNITS:
            return f'{j}: current {abs(st["cur"][i]) * A_PER_UNIT * 1000:.0f} mA is above 50 % of the {LIMIT_UNITS * A_PER_UNIT * 1000:.0f} mA limit'
        if abs(goal[i] - st['pos'][i]) > (ERR_HOLD if kind == 'hold' else ERR_MOVE):
            return f'{j}: position error {abs(goal[i] - st["pos"][i]):.2f} rad (sagging or blocked)'
        if kind == 'hold' and pred_a is not None and i > 0:
            a = abs(st['cur'][i]) * A_PER_UNIT
            if a > max(HEAVY_RATIO * pred_a[i], pred_a[i] + HEAVY_MARGIN_A):
                return f'{j}: draws {a * 1000:.0f} mA but the model predicts {pred_a[i] * 1000:.0f} mA (far heavier than expected)'
    return None


# ----------------------------------------------------------------------------------------------- procedure
def follow(bus, waypoints, rate, start_temp, sleep):
    st = bus.read()
    for a, b in zip(waypoints, waypoints[1:]):
        dur, t = max(0.5, float(np.abs(b - a).max()) / rate), 0.0
        while t < dur:
            t = min(dur, t + TICK)
            target = a + (b - a) * (t / dur)
            bus.goal(target)
            sleep(TICK)
            st = bus.read()
            send_arm(st)
            why = abort_reason(st, target, 'move', None, start_temp)
            if why:
                raise Abort(why)
    return st


def hold(bus, pose, seconds, model, start_temp, sleep):
    """Hold `pose`, return (median |current| units per joint, mean position error) over the second half of the hold."""
    samples = []
    for k in range(int(seconds / TICK)):
        bus.goal(pose)
        sleep(TICK)
        st = bus.read()
        send_arm(st)
        settled = k * TICK >= SETTLE
        why = abort_reason(st, pose, 'hold' if settled else 'move', predicted_amps(model, st['pos']) if settled else None, start_temp)
        if why:
            raise Abort(why)
        if settled:
            samples.append(st)
    half = samples[len(samples) // 2:]
    return np.median([np.abs(s['cur']) for s in half], axis=0), float(np.mean([np.abs(pose - s['pos']).max() for s in half]))


def build_ladder(nodes, edges, rest, model, bad_configs, k=6, min_gap=0.3):
    """Up to k taught poses of rising predicted load, each reachable from rest by a short taught route, away from contacts and stops."""
    from route import plan
    q_rest = np.array([rest[j] for j in JOINTS])
    stops = {1: (-1.97, 1.91), 2: (-1.95, 1.35)}                    # j2, j3: keep 0.15 rad off the mechanical stops
    cand = []
    for q in nodes[::3]:
        if any(not lo < q[i] < hi for i, (lo, hi) in stops.items()):
            continue
        if any(np.linalg.norm(q - b) < 0.2 for b in bad_configs):
            continue
        tau = model.torques(dict(zip(JOINTS, q)))
        cand.append((max(abs(t) for j, t in tau.items() if j != 'joint1'), q))
    if not cand:
        return []
    loads = np.array([c[0] for c in cand])
    chosen = []
    for target in np.linspace(loads.min() + 0.15 * np.ptp(loads), loads.max(), k):
        for idx in np.argsort(np.abs(loads - target)):
            u, q = cand[idx]
            if any(np.linalg.norm(q - c[1]) < min_gap for c in chosen):
                continue
            wps, why = plan(nodes, edges, q_rest, q)
            if wps is not None and sum(np.abs(b - a).max() for a, b in zip(wps, wps[1:])) < 3.5:
                chosen.append((u, q, wps))
                break
    return sorted(chosen, key=lambda c: c[0])


def run(bus, model, ladder, rest, nodes, edges, rate, hold_sec, sleep, ask):
    """Execute the ladder. Returns (results, abort reason or None)."""
    from route import plan
    q_rest = np.array([rest[j] for j in JOINTS])
    st0 = bus.read()
    start_temp, results, why = st0['temp'].copy(), [], None
    bus.setup()
    bus.enable()
    try:
        here = st0['pos']
        for n, (u, q, _) in enumerate(ladder, 1):
            wps, how = plan(nodes, edges, here, q)
            if wps is None:
                raise Abort(f'no taught route to pose {n}: {how}')
            st = follow(bus, wps, rate, start_temp, sleep)
            cur, err = hold(bus, q, hold_sec, model, start_temp, sleep)
            tau = model.torques(dict(zip(JOINTS, q)))
            results.append({'n': n, 'q': q, 'pred_load': u, 'pred_tau': np.array([abs(tau[j]) for j in JOINTS]), 'cur_units': cur, 'pos_err': err})
            print(f'  pose {n}/{len(ladder)} held: predicted j2 {abs(tau["joint2"]):.2f} N*m, measured ' + ' '.join(
                f'{c * A_PER_UNIT * 1000:.0f}' for c in cur) + ' mA (j1..j4)')
            here = st['pos']
        wps, how = plan(nodes, edges, here, q_rest)
        follow(bus, wps, rate, start_temp, sleep)
    except (Abort, BusError) as e:
        why = str(e)
        try:                                                        # freeze: hold where it is, torque ON
            bus.goal(bus.read()['pos'])
        except BusError:
            pass
        print(f'\nABORT: {why}\nThe arm is HELD where it is (torque on).')
        while True:
            a = ask('[p] park back along a taught route   [q] torque off now (a raised arm FALLS): ')
            if a == 'q':
                break
            if a == 'p':
                try:
                    wps, how = plan(nodes, edges, bus.read()['pos'], q_rest)
                    if wps is None:
                        print('no taught route from here:', how)
                        continue
                    follow(bus, wps, rate, start_temp, sleep)
                    break
                except (Abort, BusError) as e2:
                    print('ABORT while parking:', e2)
    bus.disable()
    return results, why


def report(results, out_csv, out_png):
    """Compare measured current with the model's torque; fit amps per N*m and compare with the spec."""
    rows = [(r['n'], j, r['pred_tau'][i], r['cur_units'][i] * A_PER_UNIT) for r in results for i, j in enumerate(JOINTS) if i > 0]
    tau, amp = np.array([r[2] for r in rows]), np.array([r[3] for r in rows])
    big = tau > 0.1
    slope = float((tau[big] * amp[big]).sum() / (tau[big] ** 2).sum()) if big.any() else float('nan')
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pose', 'joint', 'model_torque_Nm', 'measured_current_A'])
        w.writerows(rows)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6, 5))
        plt.scatter(tau, amp, c='#2a78d6')
        xs = np.linspace(0, max(tau.max(), 0.1) * 1.1, 2)
        plt.plot(xs, xs / KT_SPEC, 'g--', label=f'spec ({1 / KT_SPEC:.2f} A per N*m)')
        plt.plot(xs, xs * slope, 'r-', label=f'measured fit ({slope:.2f} A per N*m)')
        plt.xlabel('model gravity torque (N*m)')
        plt.ylabel('measured holding current (A)')
        plt.legend()
        plt.grid(alpha=.3)
        plt.title('Does the torque model match the servos?')
        plt.savefig(out_png, dpi=100, bbox_inches='tight')
    except Exception as e:  # noqa: BLE001 - the numbers matter more than the picture
        print('picture skipped:', e)
    return slope


def preflight(bus, rest):
    """Problems that forbid a real run (empty list = clear), from one read-only sample."""
    st = bus.read()
    bad = []
    if st['err'].any():
        bad.append(f'hardware error flags {st["err"].tolist()}: power-cycle the arm first')
    if st['volt'].min() < V_MIN + 0.5:
        bad.append(f'supply {st["volt"].min():.1f} V is low')
    if st['temp'].max() > 45:
        bad.append(f'a servo is at {int(st["temp"].max())} C: let it cool')
    q_rest = np.array([rest[j] for j in JOINTS])
    if np.abs(st['pos'] - q_rest).max() > 0.15:
        bad.append('the arm is not at the rest pose (within 0.15 rad): hand-guide it there first')
    return bad, st


def main():
    from arm_tool import RATE, load_rest
    from route import load_roadmap
    from torque import TorqueModel
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--go', action='store_true', help='REAL run (default is a dry run)')
    ap.add_argument('--fake', action='store_true', help='simulated bus: no hardware at all')
    ap.add_argument('--fake-scale', type=float, default=1.0, help='simulated arm draws this many times the model current')
    ap.add_argument('--fake-fault', type=int, default=0, help='simulated overload above this current (units)')
    ap.add_argument('--k', type=int, default=6)
    ap.add_argument('--hold', type=float, default=6.0)
    ap.add_argument('--rate', type=float, default=RATE)
    ap.add_argument('--payload', type=float, default=0.15)
    a = ap.parse_args()
    rest = load_rest()
    model = TorqueModel(payload_kg=a.payload)
    nodes, edges = load_roadmap()
    bad = []
    pts = os.path.expanduser('~/arm_points.yaml')
    if os.path.exists(pts):
        import yaml
        bad = [np.array(e['joints']) for e in (yaml.safe_load(open(pts)) or []) if 'hit' in e['name']]
    ladder = build_ladder(nodes, edges, rest, model, bad, a.k)
    print(f'taught roadmap: {len(nodes)} poses; contacts avoided: {len(bad)}; ladder of {len(ladder)} poses (lightest first):')
    t_total = 0.0
    for n, (u, q, wps) in enumerate(ladder, 1):
        tau = model.torques(dict(zip(JOINTS, q)))
        secs = sum(max(0.5, np.abs(b - x).max() / a.rate) for x, b in zip(wps, wps[1:])) + a.hold
        t_total += 2 * secs
        print(f'  {n}. joints ' + ' '.join(f'{v:+.2f}' for v in q) + f'  predicted load {100 * u / 4.1:3.0f}% of stall '
              f'(j2 {abs(tau["joint2"]):.2f}, j3 {abs(tau["joint3"]):.2f}, j4 {abs(tau["joint4"]):.2f} N*m)  route {len(wps)} waypoints, ~{secs:.0f} s each way')
    print(f'about {t_total / 60:.0f} min in total at {a.rate} rad/s. Abort rules (any one freezes the arm, torque stays on): hardware error flag; current > '
          f'{ABORT_UNITS * A_PER_UNIT * 1000:.0f} mA (50 % of limit); position error > {ERR_HOLD} rad when holding; temperature > {TEMP_MAX} C or +{TEMP_RISE}; '
          f'supply < {V_MIN} V; read failure; current > {HEAVY_RATIO}x the model (+{HEAVY_MARGIN_A * 1000:.0f} mA).')
    if not ladder:
        sys.exit('no usable poses: record more hand-guided streams first (teach.py --map)')
    if a.fake:
        bus = FakeBus(model, [rest[j] for j in JOINTS], a.fake_scale, a.fake_fault or None)
        sleep, ask = (lambda s: None), (lambda p: 'q')
    else:
        if subprocess.run(['pgrep', '-f', 'lib/controller_manager/[r]os2_control_node'], capture_output=True).returncode == 0:
            sys.exit('the ROS driver is running: stop it first (this test needs the serial port)')
        try:
            bus = RealBus(None)
            problems, st = preflight(bus, rest)
        except BusError as e:
            sys.exit(f'preflight: {e}')
        print('preflight (read-only): ' + ('CLEAR' if not problems else 'NOT CLEAR'))
        print('  servos: ' + ' | '.join(f'j{i + 1} {int(st["temp"][i])} C {st["volt"][i]:.1f} V err {int(st["err"][i])}' for i in range(4)))
        for p in problems:
            print('  - ' + p)
        if not a.go:
            bus.close()
            print('\nDRY RUN: nothing was written to the servos. Add --go for the real run (support under the gripper, hand at the 12 V supply).')
            return
        if problems:
            bus.close()
            sys.exit('refusing to start')
        if input('Support under the gripper, OAK and cable mounted as in normal use, hand on the 12 V supply? Type go: ').strip() != 'go':
            bus.close()
            sys.exit('not started')
        sleep, ask = time.sleep, (lambda p: input(p).strip().lower())
    results, why = run(bus, model, ladder, rest, nodes, edges, a.rate, a.hold, sleep, ask)
    bus.close()
    if results:
        stamp = time.strftime('%H%M%S')
        out = tempfile.mkdtemp(prefix='fake_load_test_') if a.fake else os.path.expanduser('~')     # simulated results never reach your data
        csv_path, png = os.path.join(out, f'arm_load_test_{stamp}.csv'), os.path.join(out, 'arm_maps', f'load_test_{stamp}.png')
        os.makedirs(os.path.dirname(png), exist_ok=True)
        slope = report(results, csv_path, png)
        print(f'\n{len(results)} pose(s) measured. fit: {slope:.2f} A per N*m (spec {1 / KT_SPEC:.2f}) -> the model '
              + ('UNDERESTIMATES' if slope > 1.2 / KT_SPEC else 'OVERESTIMATES' if slope < 0.8 / KT_SPEC else 'matches') + f' the load. {csv_path}, {png}')
    print('ended early: ' + why if why else 'completed all poses.')


if __name__ == '__main__':
    main()
