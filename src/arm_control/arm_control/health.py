"""Servo health assessment and the rule for when a servo reboot is allowed (no ROS, no bus).

A Dynamixel reboot clears a hardware-error flag but drops torque on EVERY servo (the driver's RebootDxl has no id), so a loaded arm falls.
It is therefore allowed only when the arm is parked at its rest pose and still, never for overheating (the flag would come straight back),
and at most MAX_REBOOTS times per WINDOW_SEC.
"""
TEMP_WARN, TEMP_MAX, V_MIN = 50, 55, 10.5      # C, C, V: refuse new motion above 50 C, abort above 55 C, refuse below 10.5 V (the gripper tool aborts at the same 10.5 V).
#                                              # Was 11.0: a 12 V supply sagging to ~11 V under load made the gate refuse; 10.5 V is still above the servos' own 10 V operating minimum
HW_BITS = {1: 'input voltage', 4: 'overheating', 8: 'encoder', 16: 'electrical shock', 32: 'overload'}
REBOOTABLE = {16, 32}                          # overheating and voltage come back until the cause is fixed; an encoder fault needs a look
MAX_REBOOTS, WINDOW_SEC = 2, 600.0
AT_REST_TOL = 0.1                              # rad
PERSIST_SEC = 3.0                              # a fault must last this long before a reboot (a blip clears itself)


def decode_hw(bits):
    return [name for b, name in HW_BITS.items() if bits & b]


def assess(ids, hw_state, comm_state, temps=None, volts=None):
    """-> (problems [str], rebootable bool). ids/hw_state from DynamixelState; temps/volts {id: value} from the register reads."""
    problems, reboot = [], True
    if comm_state != 0:
        problems.append(f'driver communication state {comm_state}')
        reboot = False
    for i, hw in zip(ids, hw_state):
        for b in HW_BITS:
            if hw & b:
                problems.append(f'servo {i}: {HW_BITS[b]} error')
                reboot &= b in REBOOTABLE
    for i, t in (temps or {}).items():
        if t > TEMP_MAX:
            problems.append(f'servo {i}: {t} C (limit {TEMP_MAX})')
            reboot = False
        elif t > TEMP_WARN:
            problems.append(f'servo {i}: {t} C is warm (refusing new motion above {TEMP_WARN})')
            reboot = False
    for i, v in (volts or {}).items():
        if v < V_MIN:
            problems.append(f'servo {i}: supply {v:.1f} V (minimum {V_MIN})')
            reboot = False
    return problems, bool(problems) and reboot and any(hw & b for hw in hw_state for b in REBOOTABLE)


class RebootGate:
    def __init__(self):
        self.since, self.done = None, []

    def allow(self, now, problems, rebootable, joints, rest, joint_speed):
        """(ok, why). Call once per health tick."""
        if not problems or not rebootable:
            self.since = None
            return False, 'nothing to reboot' if not problems else 'the fault is not one a reboot fixes'
        self.since = self.since if self.since is not None else now
        if now - self.since < PERSIST_SEC:
            return False, 'waiting to see if the fault persists'
        if joints is None or max(abs(joints[j] - rest[j]) for j in rest) > AT_REST_TOL:
            return False, 'the arm is not at its rest pose: a reboot drops torque and it would fall (park it first)'
        if joint_speed > 0.01:
            return False, 'the arm is moving'
        self.done = [t for t in self.done if now - t < WINDOW_SEC]
        if len(self.done) >= MAX_REBOOTS:
            return False, f'already rebooted {MAX_REBOOTS} times in {WINDOW_SEC / 60:.0f} min: needs a human'
        return True, 'ok'

    def record(self, now):
        self.done.append(now)
        self.since = None
