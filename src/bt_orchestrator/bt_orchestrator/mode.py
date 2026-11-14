"""Robot-level policies above a single task. Pure logic, no ROS (the node is robot_mode_node).

  ModeFsm        safety + init + mission budget -> one mode and a run_ok flag
  RecoveryPolicy what to do after a failed run (retry, replan, go home, ask a human, abort)

    INIT --required signals heard--> READY <--> ACTIVE (a task is running) / TELEOP (a human drives)
      |                                  ^
      '--timeout--> INIT_FAILED          '-- reset only when the cause is gone
    any state --stop latch / battery critical--> SAFE_STOP

SAFE_STOP and TELEOP halt a running plan (action 'halt'); run_ok is true only in READY/ACTIVE. The stop latch itself lives in
cmd_vel_mux (bumper/cliff); the mode node only mirrors it, so a reset goes through `safety_reset` first.
"""
INIT, INIT_FAILED, READY, ACTIVE, TELEOP, SAFE_STOP = 'init', 'init_failed', 'ready', 'active', 'teleop', 'safe_stop'

GO_HOME = {'goal': 'recovery: go home', 'steps': [{'macro': 'return_home'}]}


def ask_human(detail):
    return {'goal': 'recovery: ask for help',
            'steps': [{'macro': 'say', 'args': {'text': f'I could not finish: {detail[:80]}. Please help me.'}}]}


class ModeFsm:
    def __init__(self, required=('executor', 'llm'), init_timeout=60.0, teleop_hold=1.0, battery_clear=15.0, budget_sec=0.0):
        self.required, self.init_timeout, self.teleop_hold = set(required), init_timeout, teleop_hold
        self.battery_clear, self.budget = battery_clear, budget_sec
        self.t0 = self.battery_t = self.teleop_t = self.deadline = None
        self.heard_names, self.stop, self.task_state = set(), '', 'idle'
        self.init_failed, self.mode, self.over_budget = False, INIT, False

    # ---- inputs
    def heard(self, name):
        self.heard_names.add(name)

    def on_stop_state(self, text):
        """safety/stop_state from cmd_vel_mux: 'armed' or 'latched: <reason>'."""
        self.stop = text.split(':', 1)[1].strip() or 'stop latch' if text.startswith('latched') else ''

    def on_battery(self, level, now):
        if level == 'critical':
            self.battery_t = now

    def on_teleop(self, now):
        self.teleop_t = now

    def on_task_state(self, state, now):
        """task/state from the orchestrator; a new task starts the mission clock, a finished one stops it."""
        if state in ('planning',) and self.task_state not in ('planning', 'repairing', 'ready', 'running'):
            self.deadline, self.over_budget = (now + self.budget if self.budget > 0 else None), False
        elif state in ('idle', 'done', 'failed', 'halted'):
            self.deadline = None
        self.task_state = state

    def reset(self, now):
        """(ok, message): a human acknowledges. Refused while a hazard is still present."""
        hazards = self._hazards(now)
        if hazards:
            return False, 'refused: ' + '; '.join(hazards)
        self.init_failed, self.t0 = False, now
        return True, 'reset'

    # ---- decision
    def _hazards(self, now):
        h = [f'stop latch: {self.stop}'] if self.stop else []
        if self.battery_t is not None and now - self.battery_t < self.battery_clear:
            h.append('battery critical')
        return h

    def missing(self):
        return sorted(self.required - self.heard_names)

    def tick(self, now):
        """Recompute the mode; returns a list of actions: ('halt', why)."""
        self.t0 = now if self.t0 is None else self.t0
        if not self.missing():
            self.init_failed = False
        elif now - self.t0 > self.init_timeout:
            self.init_failed = True
        hazards = self._hazards(now)
        running = self.task_state == 'running'
        teleop = self.teleop_t is not None and now - self.teleop_t < self.teleop_hold
        old = self.mode
        if hazards:
            self.mode = SAFE_STOP
        elif self.init_failed:
            self.mode = INIT_FAILED
        elif self.missing():
            self.mode = INIT
        elif teleop:
            self.mode = TELEOP
        else:
            self.mode = ACTIVE if running else READY
        actions = []
        if running and self.mode in (SAFE_STOP, TELEOP) and old != self.mode:
            actions.append(('halt', hazards[0] if hazards else 'a human took over'))
        if running and self.deadline is not None and now > self.deadline and not self.over_budget:
            self.over_budget = True
            actions.append(('halt', 'mission time budget used up'))
        return actions

    def snapshot(self, now):
        left = None if self.deadline is None else max(0.0, round(self.deadline - now, 1))
        return {'mode': self.mode, 'run_ok': self.mode in (READY, ACTIVE), 'reasons': self._hazards(now),
                'missing': self.missing(), 'task_state': self.task_state, 'mission_left_sec': left}


class RecoveryPolicy:
    """Decision after a failed run, by where it broke and how many recoveries were already used.

    shortcut: a fixed table keyed on the failed node's name prefix; tune it with real runs (which failures are transient on this
    robot) and replace it with a learned or per-task table only when the table stops being enough."""
    TABLE = (
        (('NavigateTo', 'DriveDistance', 'RotateBy', 'FollowTarget'), ('retry', 'replan', 'go_home')),     # blocked / lost
        (('Detect', 'Find', 'Recognize', 'Count', 'Describe', 'IsPerson', 'GetPerson'), ('retry', 'replan', 'ask_human')),   # not seen
        (('WaitFor', 'Listen', 'Extract'), ('retry', 'ask_human')),                                         # speech
    )
    DEFAULT = ('replan', 'ask_human')

    def decide(self, result, failed_node, used):
        """-> 'retry' | 'replan' | 'go_home' | 'ask_human' | 'abort'."""
        if result.startswith('ERROR') or result == 'HALTED':
            return 'abort'                  # a crashed tree or a human stop must not trigger more motion
        if result == 'TIMEOUT':
            return 'go_home' if used == 0 else 'abort'
        seq = next((s for prefixes, s in self.TABLE if (failed_node or '').startswith(prefixes)), self.DEFAULT)
        return seq[used] if used < len(seq) else 'abort'
