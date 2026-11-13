import os
import signal
import subprocess
import sys
import textwrap
import time

import pytest

from cluster_core import supervisor as sv
from cluster_core.supervisor import Supervisor

PY = sys.executable
SLEEP = [PY, '-c', 'import time; time.sleep(60)']
CRASH = [PY, '-c', 'import sys; sys.exit(3)']
IGNORE_TERM = [PY, '-c', 'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
                         'print("ready", flush=True); time.sleep(60)']


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def wait(cond, timeout=10.0, step=0.02, sup=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if sup:
            sup.poll()
        if cond():
            return True
        time.sleep(step)
    return False


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    try:                                              # zombies still answer kill(0)
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


@pytest.fixture
def make(tmp_path):
    made = []

    def _make(**kw):
        s = Supervisor(str(tmp_path / 'logs'), **kw)
        made.append(s)
        return s
    yield _make
    for s in made:
        s.shutdown(timeout=3)


# ------------------------------------------------------------------- lifecycle
def test_start_run_and_graceful_stop(make):
    s = make(use_guard=False)
    assert s.start('j', [SLEEP]) and not s.start('j', [SLEEP])      # second start refused
    st = s.statuses()[0]
    assert st['state'] == sv.RUNNING and st['pid'] > 0 and st['restarts'] == 0
    assert os.path.exists(os.path.join(s.log_dir, 'j.0.log'))
    pid = st['pid']
    s.stop('j')
    assert s.state_of('j') == sv.STOPPING and 'j' not in s.active_ids()
    assert wait(lambda: 'j' not in s.jobs, sup=s) and not alive(pid)
    assert s.state_of('j') == sv.STOPPED


def test_output_goes_to_the_log_file(make):
    s = make(use_guard=False)
    s.start('j', [[PY, '-c', 'print("hello-from-job")']])
    assert wait(lambda: 'hello-from-job' in open(os.path.join(s.log_dir, 'j.0.log')).read(), sup=s)


# ------------------------------------------------------------- backoff / FAILED
def crash_cycle(s, clock, jid):
    """Wait (real time) until the crashed job is noticed; returns the backoff delay."""
    assert wait(lambda: s.state_of(jid) in (sv.BACKOFF, sv.FAILED), sup=s)
    return s.jobs[jid].next_start - clock()


def test_exponential_backoff_then_failed(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [CRASH], restart={'max_restarts': 3, 'window_sec': 1000.0,
                                   'backoff_base': 1.0, 'backoff_max': 30.0})
    delays = []
    for _ in range(3):
        d = crash_cycle(s, clock, 'j')
        delays.append(d)
        assert 'code 3' in s.jobs['j'].last_error
        clock.t += d - 0.01
        s.poll()
        assert s.state_of('j') == sv.BACKOFF                       # not yet
        clock.t += 0.02
        s.poll()
        assert s.state_of('j') == sv.RUNNING                       # relaunched on time
    assert delays == pytest.approx([1.0, 2.0, 4.0])
    assert s.jobs['j'].restarts == 3
    assert wait(lambda: s.state_of('j') == sv.FAILED, sup=s)       # 4th crash: > max_restarts
    assert 'gave up' in s.jobs['j'].last_error
    clock.t += 1000.0
    s.poll()
    assert s.state_of('j') == sv.FAILED and not s.start('j', [CRASH])   # sticks; start refused
    assert s.statuses()[0]['pid'] == -1
    s.stop('j')
    assert 'j' not in s.jobs                                        # unassigning clears FAILED


def test_backoff_is_capped(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [CRASH], restart={'max_restarts': 100, 'window_sec': 1e9, 'backoff_base': 1.0,
                                   'backoff_max': 4.0})
    delays = []
    for _ in range(5):
        d = crash_cycle(s, clock, 'j')
        delays.append(d)
        clock.t += d + 0.01
        s.poll()
    assert delays == pytest.approx([1.0, 2.0, 4.0, 4.0, 4.0])


def test_crashes_outside_the_window_do_not_count(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [CRASH], restart={'max_restarts': 1, 'window_sec': 10.0, 'backoff_base': 1.0,
                                   'backoff_max': 30.0})
    assert crash_cycle(s, clock, 'j') == pytest.approx(1.0)
    clock.t = 100.0                                                # long quiet time passes
    s.poll()
    assert s.state_of('j') == sv.RUNNING
    assert wait(lambda: s.state_of('j') == sv.BACKOFF, sup=s)       # 2nd crash: window pruned -> not FAILED
    assert s.state_of('j') != sv.FAILED


def test_unstartable_command_is_a_crash_not_an_exception(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [['/nonexistent/binary']], restart={'max_restarts': 1, 'window_sec': 100.0})
    assert s.state_of('j') == sv.BACKOFF and 'cannot start' in s.jobs['j'].last_error
    clock.t += 5
    s.poll()
    assert s.state_of('j') == sv.FAILED


def test_multi_process_job_restarts_as_a_whole(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [SLEEP, [PY, '-c', 'import time; time.sleep(0.3)']],
            restart={'max_restarts': 5, 'window_sec': 100.0, 'backoff_base': 1.0, 'backoff_max': 30.0},
            grace=1.0)
    first = [p.pid for p in s.jobs['j'].procs]
    assert len(first) == 2
    assert wait(lambda: s.state_of('j') == sv.BACKOFF, sup=s)       # the short process ended
    assert wait(lambda: not any(alive(p) for p in first), sup=s)    # ...and took its sibling down
    clock.t += 1.5
    s.poll()
    assert s.state_of('j') == sv.RUNNING and s.jobs['j'].restarts == 1
    assert not set(p.pid for p in s.jobs['j'].procs) & set(first)


# ---------------------------------------------------------- stop escalation
def test_stop_escalates_to_sigkill_after_grace(make):
    clock = Clock()
    s = make(use_guard=False, clock=clock)
    s.start('j', [IGNORE_TERM], grace=5.0)
    log = os.path.join(s.log_dir, 'j.0.log')
    assert wait(lambda: 'ready' in open(log).read())                # SIGTERM now ignored
    pid = s.statuses()[0]['pid']
    s.stop('j')
    clock.t = 4.0
    s.poll()
    time.sleep(0.3)
    assert alive(pid) and s.state_of('j') == sv.STOPPING            # grace not over: still there
    clock.t = 5.1
    s.poll()                                                        # SIGKILL
    assert wait(lambda: 'j' not in s.jobs, sup=s) and not alive(pid)


def test_shutdown_kills_everything_even_term_ignoring_jobs(make):
    s = make(use_guard=False)
    s.start('a', [IGNORE_TERM], grace=0.2)
    s.start('b', [SLEEP], grace=0.2)
    log = os.path.join(s.log_dir, 'a.0.log')
    assert wait(lambda: 'ready' in open(log).read())
    pids = [st['pid'] for st in s.statuses()]
    s.shutdown(timeout=5.0)
    assert not s.jobs and not any(alive(p) for p in pids)


# ------------------------------------------------------------------- the guard
def run_guard(*args, **kw):
    return subprocess.Popen([PY, '-m', 'cluster_core.guard', '--', *args], **kw)


def test_guard_exit_code_and_usage():
    assert run_guard(PY, '-c', 'import sys; sys.exit(5)').wait(10) == 5
    assert subprocess.run([PY, '-m', 'cluster_core.guard'], capture_output=True).returncode == 2
    assert run_guard('/nonexistent/binary', stderr=subprocess.DEVNULL).wait(10) == 127


def test_guard_forwards_sigterm_without_touching_our_own_process_group(tmp_path):
    marker = tmp_path / 'pid'
    g = run_guard(PY, '-c', f'import os,time; open({str(marker)!r},"w").write(str(os.getpid())); time.sleep(60)')
    assert wait(lambda: marker.exists() and marker.read_text())
    child = int(marker.read_text())
    assert os.getpgid(g.pid) == os.getpgrp()                        # NOT a group leader here
    g.send_signal(signal.SIGTERM)
    g.wait(10)
    assert wait(lambda: not alive(child))
    os.kill(os.getpid(), 0)                                         # the test runner is still alive


def test_job_dies_when_the_agent_is_killed_hard(tmp_path):
    """Fencing: a SIGKILLed agent must not leave its jobs running (they would be duplicated)."""
    marker = tmp_path / 'jobpid'
    job = tmp_path / 'job.py'
    job.write_text(f'import os, time\nopen({str(marker)!r}, "w").write(str(os.getpid()))\ntime.sleep(60)\n')
    script = textwrap.dedent(f'''
        import sys, time
        from cluster_core.supervisor import Supervisor
        s = Supervisor({str(tmp_path / 'logs')!r})
        s.start('j', [[sys.executable, {str(job)!r}]])
        print(s.statuses()[0]['pid'], flush=True)
        while True:
            time.sleep(0.1)
    ''')
    agent = subprocess.Popen([PY, '-c', script], stdout=subprocess.PIPE, text=True,
                             start_new_session=True)
    guard_pid = int(agent.stdout.readline())
    assert wait(lambda: marker.exists() and marker.read_text())
    job_pid = int(marker.read_text())
    assert alive(guard_pid) and alive(job_pid)
    t0 = time.monotonic()
    agent.kill()                                                     # SIGKILL: no cleanup code runs
    agent.wait()
    assert wait(lambda: not alive(job_pid) and not alive(guard_pid), timeout=5.0)
    assert time.monotonic() - t0 < 3.0                               # well inside orphan_timeout


def test_guarded_supervisor_job_runs_and_stops(make):
    s = make(use_guard=True)
    s.start('j', [SLEEP])
    pid = s.statuses()[0]['pid']
    assert alive(pid)
    s.stop('j')
    assert wait(lambda: 'j' not in s.jobs, sup=s, timeout=10.0) and not alive(pid)
