#!/usr/bin/env python3
"""Gripper (servo ID 15) helper, READ-ONLY so far: it never writes to the servo and never enables torque.

    python3 tools/arm/gripper_tool.py status           # mode, torque, error flags, current limit, position
    python3 tools/arm/gripper_tool.py range            # move the fingers by hand (torque off): live position, Ctrl+C saves the open/closed range
    python3 tools/arm/gripper_tool.py open|close|goto --gap 40     # DRY RUN: plan + read-only preflight. Add --go to move (nothing in the fingers)
    python3 tools/arm/gripper_tool.py close --go --current 150     # close until it touches something: prints the width it stopped at

Motion safety: the Goal Current is set AND READ BACK (<= 250 units, 0.67 A) before torque is enabled; it moves in 25-tick steps, stops
when it stalls (touched something, or reached the end), freezes on any hardware error / over-temperature / low voltage / read failure, and
releases (torque off) at the end unless --keep. Ctrl+C freezes it. Calibration: param/arm_gripper.yaml (open = LOWER tick number here).

Position -> metres uses the stock xacro unit info: joint_m = raw * 1.7627e-5 - 0.0361 (the joint value MoveIt / ros2_control report).
WARNING for any future motion code: after a power cycle the servo's Goal Current register is 1193 (the maximum, 3.2 A); the ROS driver
lowers it to 200 at start-up. Set Goal Current BEFORE enabling torque, or the gripper squeezes at full force.
The ROS driver must not be running (it owns the serial port). Needs the Dynamixel SDK.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

import yaml
import bus_lock
import telemetry

GRIPPER_ID = 15
_LOCK = None     # held for the whole run so no other tool talks to the servos at the same time
from arm_control.gripper import (CLOSED_TICKS, GAP_CLOSED_MM, GAP_OPEN_MM, M_OFFSET, M_PER_TICK, OPEN_TICKS, gap_mm,  # noqa: E402,F401
                                 ticks_for_gap, to_m)
A_PER_UNIT = 0.00269


def open_bus():
    from dynamixel_sdk import PacketHandler, PortHandler
    if subprocess.run(['pgrep', '-f', 'lib/controller_manager/[r]os2_control_node'], capture_output=True).returncode == 0:
        sys.exit('the ROS driver is running: stop it first (it owns the serial port)')
    global _LOCK
    _LOCK = bus_lock.acquire()
    if _LOCK is None:
        sys.exit('the arm serial port is busy (another arm tool or the GUI polling): try again')
    dev = (glob.glob('/dev/serial/by-id/*OpenCR*') or [''])[0]
    port = PortHandler(dev)
    if not dev or not port.openPort() or not port.setBaudRate(1000000):
        sys.exit('cannot open the OpenCR port')
    return port, PacketHandler(2.0)


def read_state(port, pk):
    """{'raw', 'current_units', 'torque', 'error', 'temp', 'volt'} of the gripper servo, read-only."""
    def r(fn, addr):
        v, res, _ = fn(port, GRIPPER_ID, addr)
        if res != 0:
            raise SystemExit('the gripper servo (ID 15) does not answer')
        return v
    raw = r(pk.read4ByteTxRx, 132)
    cur = r(pk.read2ByteTxRx, 126)
    return {'raw': raw - (1 << 32 if raw >= 1 << 31 else 0), 'current_units': cur - (1 << 16 if cur >= 1 << 15 else 0),
            'torque': r(pk.read1ByteTxRx, 64), 'error': r(pk.read1ByteTxRx, 70), 'temp': r(pk.read1ByteTxRx, 146),
            'volt': r(pk.read2ByteTxRx, 144) / 10.0, 'mode': r(pk.read1ByteTxRx, 11), 'goal_current': r(pk.read2ByteTxRx, 102),
            'current_limit': r(pk.read2ByteTxRx, 38)}


def status():
    port, pk = open_bus()
    try:
        s = read_state(port, pk)
    finally:
        port.closePort()
    print(f"mode {s['mode']} (5 = current-based position)   torque {'ON' if s['torque'] else 'off'}   hardware error {s['error']}   "
          f"{s['temp']} C   {s['volt']:.1f} V")
    print(f"position {s['raw']} ticks = finger gap about {gap_mm(s['raw']):.0f} mm (+-3)   present current {s['current_units'] * A_PER_UNIT * 1000:.0f} mA")
    print(f"current limit {s['current_limit']} ({s['current_limit'] * A_PER_UNIT:.2f} A)   goal current {s['goal_current']} ({s['goal_current'] * A_PER_UNIT:.2f} A)"
          + ('   <- MAXIMUM: set it before enabling torque' if s['goal_current'] >= s['current_limit'] else ''))


def range_mode(path):
    port, pk = open_bus()
    lo = hi = None
    n = 0
    print('Move the gripper fingers BY HAND to fully open and fully closed (torque is off). Ctrl+C = finish and save.\n')
    try:
        while True:
            s = read_state(port, pk)
            if n == 0 and s['torque']:
                print('WARNING: torque is ON on the gripper: it will resist the hand')
            raw = s['raw']
            lo, hi = (raw, raw) if lo is None else (min(lo, raw), max(hi, raw))
            n += 1
            print(f"\r  now {raw:5d} ticks, gap ~{gap_mm(raw):4.0f} mm   seen {lo}..{hi} ticks ({to_m(lo) * 1000:+.1f}..{to_m(hi) * 1000:+.1f} mm)"
                  f"   current {s['current_units'] * A_PER_UNIT * 1000:4.0f} mA   ", end='', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        port.closePort()
    if lo is None:
        return
    print(f'\n\nrange: {lo}..{hi} ticks = {to_m(lo) * 1000:+.1f}..{to_m(hi) * 1000:+.1f} mm of joint travel {(hi - lo) * M_PER_TICK * 1000:.1f} mm')
    entry = {'when': time.strftime('%F %T'), 'min_ticks': lo, 'max_ticks': hi, 'min_mm': round(to_m(lo) * 1000, 1), 'max_mm': round(to_m(hi) * 1000, 1)}
    path = os.path.expanduser(path)
    with open(path, 'a') as f:
        f.write('- ' + yaml.safe_dump(entry, default_flow_style=True, width=1000).strip() + '\n')
    print('appended to', path)


CURRENT_CAP = 250                  # units (0.67 A): more needs --force
from arm_control.gripper import SAFE_CLOSE, SAFE_OPEN  # noqa: E402
STEP, TICK, STALL_SEC, MOVE_LIMIT_SEC = 25, 0.05, 0.3, 12.0
V_MIN, TEMP_MAX = 10.5, 55


class GripperError(Exception):
    pass


class RealGripperBus:
    def __init__(self, port, pk):
        self.port, self.pk = port, pk

    def read(self):
        s = read_state(self.port, self.pk)
        return {'raw': s['raw'], 'cur': s['current_units'], 'err': s['error'], 'temp': s['temp'], 'volt': s['volt'], 'torque': s['torque'],
                'mode': s['mode']}

    def set_current(self, units):
        self.pk.write2ByteTxRx(self.port, GRIPPER_ID, 102, units)
        v, res, _ = self.pk.read2ByteTxRx(self.port, GRIPPER_ID, 102)
        return v if res == 0 else -1

    def setup(self):
        self.pk.write4ByteTxRx(self.port, GRIPPER_ID, 112, 50)      # profile velocity
        self.pk.write4ByteTxRx(self.port, GRIPPER_ID, 108, 20)      # profile acceleration

    def goal(self, ticks):
        self.pk.write4ByteTxRx(self.port, GRIPPER_ID, 116, int(ticks) & 0xFFFFFFFF)

    def enable(self):
        self.pk.write1ByteTxRx(self.port, GRIPPER_ID, 64, 1)

    def disable(self):
        self.pk.write1ByteTxRx(self.port, GRIPPER_ID, 64, 0)


class FakeGripperBus:
    """Simulated gripper: an obstacle at `block` ticks stops motion toward higher numbers (closing) and the servo then draws its goal current."""

    def __init__(self, raw=OPEN_TICKS + 40, block=CLOSED_TICKS + 20, fault_after=None, current_readback=None):
        self.raw, self.g, self.block, self.on, self.cur_limit, self.reads = float(raw), float(raw), block, False, 0, 0
        self.fault_after, self.readback, self.err = fault_after, current_readback, 0

    def read(self):
        self.reads += 1
        blocked = False
        if self.on:
            nxt = self.raw + max(-30, min(30, self.g - self.raw))
            blocked = nxt > self.block
            self.raw = min(nxt, self.block)
        if self.fault_after is not None and self.reads > self.fault_after:
            self.err = 32
        stall = blocked and self.g > self.raw + 1
        return {'raw': int(self.raw), 'cur': self.cur_limit if stall else 15, 'err': self.err, 'temp': 30, 'volt': 12.0, 'torque': int(self.on), 'mode': 5}

    def set_current(self, units):
        self.cur_limit = units
        return units if self.readback is None else self.readback

    def setup(self):
        pass

    def goal(self, ticks):
        self.g = float(ticks)

    def enable(self):
        self.g, self.on = self.raw, True

    def disable(self):
        self.on = False


def ready_problems(s):
    """Reasons a real move must not start (empty = clear), from one read-only sample."""
    bad = []
    if s['err']:
        bad.append(f"hardware error flag {s['err']}: power-cycle the arm first")
    if s['mode'] != 5:
        bad.append(f"operating mode {s['mode']} (expected 5, current-based position): refusing to change it")
    if s['volt'] < V_MIN + 0.5 or s['temp'] > 50:
        bad.append(f"supply {s['volt']:.1f} V / {s['temp']} C is out of range")
    if not OPEN_TICKS - 150 <= s['raw'] <= CLOSED_TICKS + 100:
        bad.append(f"position {s['raw']} is outside the calibrated range {OPEN_TICKS}..{CLOSED_TICKS}: re-run `range`")
    return bad


def move(bus, target, current, sleep, hold=1.0, keep=False, ask=None):
    """Step the gripper to `target` ticks at a limited current. Returns {'status': reached|stalled|aborted|frozen, 'raw', 'why'}."""
    s0 = bus.read()
    bad = ready_problems(s0)
    if bad:
        raise GripperError('; '.join(bad))
    if bus.set_current(current) != current:                       # the safety interlock: never enable torque at an unknown current
        raise GripperError('the servo did not confirm the goal current: torque NOT enabled')
    bus.setup()
    bus.goal(s0['raw'])                                           # goal = where it is, so torque-on cannot jump
    bus.enable()
    g, stall_t, t, why, status, peak = s0['raw'], 0.0, 0.0, '', 'reached', 0
    try:
        while True:
            g += max(-STEP, min(STEP, target - g))
            bus.goal(g)
            sleep(TICK)
            t += TICK
            st = bus.read()
            peak = max(peak, abs(st['cur']))
            telemetry.send('gripper', raw=st['raw'], cur=st['cur'], goal=g, gap=round(gap_mm(st['raw']), 1), temp=st['temp'], volt=st['volt'], err=st['err'])
            if st['err'] or st['temp'] > TEMP_MAX or st['volt'] < V_MIN:
                why, status = f"hardware error {st['err']}, {st['temp']} C, {st['volt']:.1f} V", 'aborted'
                break
            if abs(st['cur']) >= 0.9 * current and abs(g - st['raw']) > 20:
                stall_t += TICK
                if stall_t >= STALL_SEC:
                    why, status = 'stalled: touched something, or reached the end', 'stalled'
                    break
            else:
                stall_t = 0.0
            if g == target and abs(st['raw'] - target) <= 15:
                break
            if t > MOVE_LIMIT_SEC:
                why, status = 'did not arrive in time', 'aborted'
                break
    except KeyboardInterrupt:
        why, status = 'Ctrl+C', 'frozen'
    except Exception as e:  # noqa: BLE001 - any read/write failure: freeze, do not keep driving
        why, status = f'bus error: {e}', 'aborted'
    try:
        raw = bus.read()['raw']
        bus.goal(raw)                                             # freeze here, torque still on
        for _ in range(int(hold / TICK)):
            sleep(TICK)
            raw = bus.read()['raw']
    except Exception:  # noqa: BLE001
        raw = None
    if status == 'frozen' and ask is not None and ask('Frozen (held, torque on). Release now (torque off)? [Enter = yes, h = keep holding]: ') == 'h':
        return {'status': status, 'raw': raw, 'why': why, 'peak_ma': peak * A_PER_UNIT * 1000}
    if not keep:
        bus.disable()
    return {'status': status, 'raw': raw, 'why': why, 'peak_ma': peak * A_PER_UNIT * 1000}


def describe(res, target):
    raw = res['raw']
    if raw is None:
        return f"{res['status']}: {res['why']}"
    line = f"{res['status']} at {raw} ticks, finger gap about {gap_mm(raw):.0f} mm (+-3)"
    if res['status'] == 'stalled' and target > raw + 40:
        line += f'. Stopped {target - raw} ticks short of the closed target: it is HOLDING something about {gap_mm(raw):.0f} mm wide'
    return line + f", peak current {res.get('peak_ma', 0):.0f} mA" + (f" ({res['why']})" if res['why'] else '')


def motion(a):
    port, pk = open_bus() if not a.fake else (None, None)
    if a.cmd == 'open':
        target = SAFE_OPEN
    elif a.cmd == 'close':
        target = SAFE_CLOSE
    else:
        if a.gap is None:
            sys.exit('goto needs --gap MM')
        target = max(min(ticks_for_gap(a.gap), SAFE_CLOSE), SAFE_OPEN)
    cap = CURRENT_CAP if not a.force else 1193
    if a.current > cap:
        sys.exit(f'--current {a.current} is above the cap {cap} units ({cap * A_PER_UNIT:.2f} A); --force to override')
    bus = FakeGripperBus(block=a.fake_block) if a.fake else RealGripperBus(port, pk)
    s0 = bus.read()
    print(f"gripper now {s0['raw']} ticks (gap ~{gap_mm(s0['raw']):.0f} mm); target {target} ticks (gap ~{gap_mm(target):.0f} mm); goal current {a.current} units "
          f"({a.current * A_PER_UNIT * 1000:.0f} mA); 25 ticks per 50 ms; stops on a stall; torque off at the end" + ('; torque KEPT ON' if a.keep else ''))
    bad = ready_problems(s0)
    print('preflight: ' + ('CLEAR' if not bad else 'NOT CLEAR'))
    for b in bad:
        print('  - ' + b)
    if not (a.go or a.fake):
        print('DRY RUN: nothing was written to the servo. Add --go to move (nothing in the fingers, hand near the 12 V supply).')
        port.closePort()
        return
    if bad:
        sys.exit('refusing to start')
    if not a.fake and input('Fingers clear (or the object in place), hand near the supply. Type go: ').strip() != 'go':
        sys.exit('not started')
    try:
        res = move(bus, target, a.current, (lambda s: None) if a.fake else time.sleep, a.hold, a.keep, (lambda p: input(p).strip().lower()) if not a.fake else None)
    except GripperError as e:
        sys.exit(f'REFUSED: {e}')
    finally:
        if port is not None:
            port.closePort()
    print(describe(res, target))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('cmd', choices=('status', 'range', 'open', 'close', 'goto'))
    ap.add_argument('--file', default='~/arm_gripper.yaml')
    ap.add_argument('--gap', type=float, help='goto: finger gap in mm')
    ap.add_argument('--current', type=int, default=120, help='goal current in units (2.69 mA each); default 120 = 0.32 A')
    ap.add_argument('--go', action='store_true', help='really move (default is a dry run)')
    ap.add_argument('--hold', type=float, default=1.0, help='seconds to hold at the end before releasing')
    ap.add_argument('--keep', action='store_true', help='leave torque ON at the end (it holds at the goal current)')
    ap.add_argument('--force', action='store_true', help='allow a goal current above the cap')
    ap.add_argument('--fake', action='store_true', help='simulated gripper, no hardware')
    ap.add_argument('--fake-block', type=int, default=CLOSED_TICKS + 20, help='simulated obstacle position in ticks')
    a = ap.parse_args()
    if a.cmd == 'status':
        status()
    elif a.cmd == 'range':
        range_mode(a.file)
    else:
        motion(a)


if __name__ == '__main__':
    main()
