"""Process supervisor used by the agent.

* each process of a job runs through cluster_core.guard in its own session/process group
* a crash (any process of the job exits) stops the whole job, then restarts it after an
  exponential backoff (base, 2*base, 4*base ... capped); more than `max_restarts` crashes inside
  `window_sec` puts the job in FAILED, where it stays until it is unassigned (stop())
* stop() is non-blocking: SIGTERM now, SIGKILL after the job's grace period (checked in poll())
* the clock is injectable so backoff/grace logic is testable without waiting
"""
import os
import signal
import subprocess
import sys
import time
from collections import deque

# numeric values match cluster_msgs/JobStatus
STOPPED, RUNNING, BACKOFF, FAILED, STOPPING = 0, 1, 2, 3, 4
NAMES = {STOPPED: 'stopped', RUNNING: 'running', BACKOFF: 'backoff', FAILED: 'failed',
         STOPPING: 'stopping'}

DEFAULT_RESTART = {'max_restarts': 5, 'window_sec': 60.0, 'backoff_base': 1.0, 'backoff_max': 30.0}


def _signal_group(proc, sig):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


class Job:
    def __init__(self, job_id, cmds, env, restart, grace):
        self.id = job_id
        self.cmds = cmds
        self.env = env
        self.restart = {**DEFAULT_RESTART, **(restart or {})}
        self.grace = grace
        self.state = STOPPED
        self.procs = []
        self.restarts = 0
        self.crashes = deque()
        self.next_start = 0.0
        self.term_deadline = None
        self.last_error = ''

    def live(self):
        return [p for p in self.procs if p.poll() is None]


class Supervisor:
    def __init__(self, log_dir, clock=time.monotonic, use_guard=True, python=None):
        self.log_dir = log_dir
        self.clock = clock
        self.use_guard = use_guard
        self.python = python or sys.executable
        self.jobs = {}
        os.makedirs(log_dir, exist_ok=True)

    # ------------------------------------------------------------------ public
    def start(self, job_id, cmds, env=None, restart=None, grace=5.0):
        """Start a job. False if it already exists in any state (incl. FAILED/STOPPING)."""
        if job_id in self.jobs:
            return False
        job = Job(job_id, cmds, env or {}, restart, grace)
        self.jobs[job_id] = job
        self._launch(job, self.clock())
        return True

    def stop(self, job_id):
        job = self.jobs.get(job_id)
        if job is None or job.state == STOPPING:
            return
        live = job.live()
        if not live:
            del self.jobs[job_id]
            return
        for p in live:
            _signal_group(p, signal.SIGTERM)
        job.state = STOPPING
        job.term_deadline = self.clock() + job.grace

    def stop_all(self):
        for jid in list(self.jobs):
            self.stop(jid)

    def poll(self):
        now = self.clock()
        for job in list(self.jobs.values()):
            if job.state == RUNNING:
                dead = [p for p in job.procs if p.poll() is not None]
                if dead:
                    rc = dead[0].returncode
                    job.last_error = f'process exited with code {rc} (see {self._log(job.id, 0)})'
                    self._on_crash(job, now)
            elif job.state in (BACKOFF, FAILED):
                self._reap_leftovers(job, now)
                if job.state == BACKOFF and not job.live() and now >= job.next_start:
                    self._launch(job, now, relaunch=True)
            elif job.state == STOPPING:
                if not job.live():
                    del self.jobs[job.id]
                elif now >= job.term_deadline:
                    for p in job.live():
                        _signal_group(p, signal.SIGKILL)

    def statuses(self):
        out = []
        for job in self.jobs.values():
            live = job.live()
            out.append({'job_id': job.id, 'state': job.state, 'restarts': job.restarts,
                        'pid': live[0].pid if live else -1, 'last_error': job.last_error})
        return sorted(out, key=lambda s: s['job_id'])

    def state_of(self, job_id):
        job = self.jobs.get(job_id)
        return job.state if job else STOPPED

    def active_ids(self):
        """Jobs that exist and are not being removed (RUNNING, BACKOFF, FAILED)."""
        return {j.id for j in self.jobs.values() if j.state != STOPPING}

    def shutdown(self, timeout=8.0):
        """Blocking: stop everything, escalating to SIGKILL. For process exit only."""
        self.stop_all()
        end = time.monotonic() + timeout
        while self.jobs and time.monotonic() < end:
            self.poll()
            time.sleep(0.05)
        for job in list(self.jobs.values()):
            for p in job.live():
                _signal_group(p, signal.SIGKILL)
        self.jobs.clear()

    # ----------------------------------------------------------------- internal
    def _log(self, job_id, i):
        return os.path.join(self.log_dir, f'{job_id}.{i}.log')

    def _argv(self, argv):
        return [self.python, '-m', 'cluster_core.guard', '--'] + argv if self.use_guard else argv

    def _launch(self, job, now, relaunch=False):
        env = dict(os.environ)
        env.update(job.env)
        started = []
        try:
            for i, argv in enumerate(job.cmds):
                with open(self._log(job.id, i), 'ab') as log:
                    started.append(subprocess.Popen(
                        self._argv(argv), stdin=subprocess.DEVNULL, stdout=log,
                        stderr=subprocess.STDOUT, env=env, start_new_session=True))
        except OSError as e:
            for p in started:
                _signal_group(p, signal.SIGKILL)
            job.procs = []
            job.last_error = f'cannot start {job.cmds[len(started)][0]!r}: {e}'
            if relaunch:
                job.restarts += 1
            self._on_crash(job, now)
            return
        if relaunch:
            job.restarts += 1
        job.procs = started
        job.state = RUNNING
        job.term_deadline = None

    def _on_crash(self, job, now):
        for p in job.live():                         # the rest of the job goes down too
            _signal_group(p, signal.SIGTERM)
        job.term_deadline = now + job.grace
        r = job.restart
        while job.crashes and now - job.crashes[0] > r['window_sec']:
            job.crashes.popleft()
        n = len(job.crashes)
        job.crashes.append(now)
        if len(job.crashes) > r['max_restarts']:
            job.state = FAILED
            job.last_error += f"; gave up: {len(job.crashes)} crashes within {r['window_sec']:.0f}s"
            return
        job.state = BACKOFF
        job.next_start = now + min(r['backoff_base'] * (2 ** n), r['backoff_max'])

    def _reap_leftovers(self, job, now):
        if job.live() and job.term_deadline is not None and now >= job.term_deadline:
            for p in job.live():
                _signal_group(p, signal.SIGKILL)
