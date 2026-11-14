#!/usr/bin/python3
"""Integration test of the cluster platform on ONE machine (simulated hosts).

Runs a real scheduler and real agents as separate processes (each with its own host id, roles and
tags) on a private DDS domain, with harmless sleeper jobs plus one real ROS node, and measures:

  * convergence, and heartbeat -> placement latency for a host that joins later
  * hard failover (SIGKILL of an agent): time to decision / recovery, no duplicate job instances
  * pinned (immovable) jobs stay unplaceable instead of moving to a stranger, and recover
  * scheduler restart: adoption keeps every job (same pids), scheduler freeze (SIGSTOP/SIGCONT)
  * refusal of a mismatched catalog, of a wrong catalog hash, and of unknown job ids

NOT covered: a second physical machine, a real network (WiFi/Ethernet), packet loss.

    export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep off other robots
    python3 src/cluster_core/test/integration/it_cluster.py [--keep]
Exit status 0 only if every check passes.
"""
import argparse
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import time

import yaml

import rclpy
from cluster_core.catalog import parse_catalog
from cluster_core.qos import INFO_QOS, LATCHED_QOS, TOPIC_ASSIGN, TOPIC_INFO, TOPIC_STATUS
from cluster_msgs.msg import Assignment, AssignmentSet, ClusterStatus, JobStatus, NodeInfo
from rclpy.node import Node

PY = sys.executable
RESULTS = []
NUMBERS = {}


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok), detail))
    print(('PASS  ' if ok else 'FAIL  ') + name + (f'   [{detail}]' if detail else ''), flush=True)
    return ok


def info(msg):
    print(f'\n--- {msg}', flush=True)


# ------------------------------------------------------------------ workspace
class Work:
    def __init__(self, keep):
        self.keep = keep
        self.dir = tempfile.mkdtemp(prefix='it_cluster_')
        self.markers = os.path.join(self.dir, 'markers')
        self.logs = os.path.join(self.dir, 'logs')
        os.makedirs(self.markers)
        os.makedirs(self.logs)
        self.sleeper = os.path.join(self.dir, 'sleeper.py')
        with open(self.sleeper, 'w') as f:
            f.write(textwrap.dedent('''
                import os, sys, time
                name, markers = sys.argv[1], sys.argv[2]
                open(os.path.join(markers, f"{name}.{os.getpid()}.pid"), "w").write(str(os.getpid()))
                while True:
                    time.sleep(1)
            '''))
        self.jobs = self.write_catalog('jobs.yaml')
        self.odd_jobs = self.write_catalog('odd_jobs.yaml', extra={'zzz': self.sleep_job('zzz')})

    def sleep_job(self, name, **kw):
        return {'cmd': [PY, self.sleeper, name, self.markers], 'cpu': 0.2, 'mem_mb': 50, **kw}

    def write_catalog(self, fname, extra=None):
        jobs = {
            'core_job': self.sleep_job('core_job', role='core', movable=False),
            'cam_job': self.sleep_job('cam_job', tags=['camera']),
            'svc_a': self.sleep_job('svc_a'),
            'svc_b': self.sleep_job('svc_b'),
            'svc_c': self.sleep_job('svc_c'),
            'gpu_job': {'cmds': [[PY, self.sleeper, 'gpu_job_a', self.markers],
                                 [PY, self.sleeper, 'gpu_job_b', self.markers]],
                        'prefer_tags': ['gpu'], 'cpu': 0.5, 'mem_mb': 100},
            'validator': {'cmd': ['ros2', 'run', 'bt_validator', 'bt_validator_node'],
                          'cpu': 0.2, 'mem_mb': 100},
            'extra_job': self.sleep_job('extra_job', tags=['extra']),
            'probe_job': self.sleep_job('probe_job', tags=['probe']),     # only ever placed on ghost hosts
        }
        jobs.update(extra or {})
        path = os.path.join(self.dir, fname)
        with open(path, 'w') as f:
            yaml.safe_dump({'jobs': jobs}, f)
        return path

    def hash_of(self, path):
        with open(path) as f:
            return parse_catalog(yaml.safe_load(f)).hash

    def cleanup(self):
        if self.keep:
            print(f'\nkept {self.dir}')
        else:
            shutil.rmtree(self.dir, ignore_errors=True)


PROCS_OF = {'gpu_job': ['gpu_job_a', 'gpu_job_b']}
SLEEPERS = ['core_job', 'cam_job', 'svc_a', 'svc_b', 'svc_c', 'gpu_job_a', 'gpu_job_b', 'extra_job']


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except (ProcessLookupError, FileNotFoundError, OSError):
        return False


def live(work, name):
    out = []
    for fn in os.listdir(work.markers):
        m = re.fullmatch(re.escape(name) + r'\.(\d+)\.pid', fn)
        if m and pid_alive(int(m.group(1))):
            out.append(int(m.group(1)))
    return sorted(out)


def read(path):
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


# ------------------------------------------------------------------- processes
class Proc:
    def __init__(self, work, tag, argv, env):
        self.tag = tag
        self.log = os.path.join(work.logs, f'{tag}.log')
        self.f = open(self.log, 'ab')
        self.p = subprocess.Popen(argv, stdout=self.f, stderr=subprocess.STDOUT, env=env,
                                  start_new_session=True)

    def signal(self, sig):
        try:
            os.killpg(self.p.pid, sig)               # our own group only (we started it)
        except ProcessLookupError:
            pass

    def alive(self):
        return self.p.poll() is None

    def stop(self, sig=None, wait=6.0):
        self.signal(sig or signal.SIGTERM)       # (default here: the class attribute `signal` is a method)
        end = time.monotonic() + wait
        while self.alive() and time.monotonic() < end:
            time.sleep(0.05)
        if self.alive():
            self.signal(signal.SIGKILL)
            self.p.wait()
        self.f.close()


GUARD = True            # --control-no-guard flips this: the positive control for the fencing checks
STOP_AFTER_FAILOVER = False


def agent(work, env, host, roles, tags, jobs=None):
    return Proc(work, f'agent-{host}', [
        PY, '-m', 'cluster_core.agent_node', '--ros-args', '-p', f'host_id:={host}',
        '-p', f'roles:={roles}', '-p', f'tags:={tags}', '-p', f'jobs_file:={jobs or work.jobs}',
        '-p', f'use_guard:={"true" if GUARD else "false"}',
        '-p', f'log_dir:={os.path.join(work.logs, "jobs-" + host)}', '-p', 'params_dir:=/tmp'], env)


def scheduler(work, env, sid):
    return Proc(work, f'scheduler-{sid}', [
        PY, '-m', 'cluster_core.scheduler_node', '--ros-args', '-p', f'jobs_file:={work.jobs}',
        '-p', f'scheduler_id:={sid}', '-p', 'params_dir:=/tmp'], env)


# -------------------------------------------------------------------- observer
class Observer(Node):
    def __init__(self, work):
        super().__init__('it_observer')
        self.work = work
        self.t_first = {}
        self.hosts = {}                 # host -> (t_rx, {job: (state, pid, restarts)})
        self.assign_log = []            # (t, scheduler_id, epoch, {job: host}, hash)
        self.status = None
        self.dup_max = {n: 0 for n in SLEEPERS}
        self.create_subscription(NodeInfo, TOPIC_INFO, self._info, INFO_QOS)
        self.create_subscription(AssignmentSet, TOPIC_ASSIGN, self._assign, LATCHED_QOS)
        self.create_subscription(ClusterStatus, TOPIC_STATUS, self._status, LATCHED_QOS)
        self.pub = self.create_publisher(AssignmentSet, TOPIC_ASSIGN, LATCHED_QOS)
        self.ghost_pub = self.create_publisher(NodeInfo, TOPIC_INFO, INFO_QOS)
        self._last_dup = 0.0

    def _info(self, m):
        t = time.monotonic()
        self.t_first.setdefault(m.host_id, t)
        self.hosts[m.host_id] = (t, {j.job_id: (j.state, j.pid, j.restarts) for j in m.jobs})

    def _assign(self, m):
        if m.scheduler_id == 'rogue':
            return
        self.assign_log.append((time.monotonic(), m.scheduler_id, int(m.epoch),
                                {a.job_id: a.host_id for a in m.assignments}, m.catalog_hash))

    def _status(self, m):
        self.status = {'t': time.monotonic(), 'sid': m.scheduler_id, 'epoch': int(m.epoch),
                       'alive': list(m.hosts_alive), 'dead': list(m.hosts_dead),
                       'excluded': list(m.hosts_excluded), 'events': list(m.events),
                       'jobs': {j.job_id: (j.host_id, j.state, j.reason, j.job_state) for j in m.jobs}}

    # --- queries
    def running_on(self, job, max_age=1.5):
        now = time.monotonic()
        return sorted(h for h, (t, jobs) in self.hosts.items()
                      if now - t < max_age and jobs.get(job, (None,))[0] == JobStatus.RUNNING)

    def last_assignment(self):
        return self.assign_log[-1][3] if self.assign_log else {}

    def placement(self, job):
        return self.status['jobs'].get(job) if self.status else None

    def pids(self, job):
        return sorted(p for _, (t, jobs) in self.hosts.items() for j, (s, p, r) in jobs.items()
                      if j == job and s == JobStatus.RUNNING)

    # --- spinning
    def sample_dups(self):
        now = time.monotonic()
        if now - self._last_dup < 0.1:
            return
        self._last_dup = now
        for n in SLEEPERS:
            self.dup_max[n] = max(self.dup_max[n], len(live(self.work, n)))

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.sample_dups()

    def wait_until(self, cond, timeout, step=0.02):
        """Returns the monotonic time at which cond() became true, or None on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=step)
            self.sample_dups()
            if cond():
                return time.monotonic()
        return None

    def publish_ghost_heartbeat(self, host, tags, cat_hash):
        """Heartbeat of a host that does not really exist (to time the scheduler alone)."""
        m = NodeInfo(host_id=host, roles=['compute'], tags=tags, cpu_cores=4, cpu_load=0.05,
                     mem_total_mb=8000.0, mem_free_mb=6000.0, gpu_mem_total_mb=-1.0,
                     gpu_mem_free_mb=-1.0, gpu_util=-1.0, temp_c=-1.0, catalog_hash=cat_hash)
        m.header.stamp = self.get_clock().now().to_msg()
        self.ghost_seq = getattr(self, 'ghost_seq', 0) + 1
        m.seq = self.ghost_seq
        self.ghost_pub.publish(m)

    def publish_set(self, sid, epoch, cat_hash, pairs):
        m = AssignmentSet(scheduler_id=sid, epoch=epoch, catalog_hash=cat_hash)
        m.header.stamp = self.get_clock().now().to_msg()
        m.assignments = [Assignment(job_id=j, host_id=h) for j, h in pairs]
        self.pub.publish(m)


# -------------------------------------------------------------------- scenarios
ALL_JOBS = ['core_job', 'cam_job', 'svc_a', 'svc_b', 'svc_c', 'gpu_job', 'validator']


def converged(obs, jobs=ALL_JOBS):
    return all(len(obs.running_on(j)) == 1 for j in jobs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keep', action='store_true', help='keep logs/markers in a temp dir')
    ap.add_argument('--control-no-guard', action='store_true',
                    help='POSITIVE CONTROL: run agents without the process guard and stop after the '
                         'failover scenario; the fencing/duplicate checks are then EXPECTED to fail')
    a = ap.parse_args()
    global GUARD, STOP_AFTER_FAILOVER
    if a.control_no_guard:
        GUARD, STOP_AFTER_FAILOVER = False, True
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    env.setdefault('ROS_DOMAIN_ID', '42')
    env.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    print(f"ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']} discovery={env['ROS_AUTOMATIC_DISCOVERY_RANGE']}")

    work = Work(a.keep)
    procs = {}
    rclpy.init()
    obs = Observer(work)
    try:
        run_scenarios(work, env, procs, obs)
    finally:
        info('cleanup')
        for p in procs.values():
            p.stop()
        time.sleep(1.0)
        left = {n: live(work, n) for n in SLEEPERS if live(work, n)}
        check('no job process survives the shutdown of all agents', not left, str(left))
        for pids in left.values():                      # never leak test processes (our own markers)
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        obs.destroy_node()
        rclpy.try_shutdown()
        work.cleanup()

    print('\n================ measurements ================')
    for k, v in NUMBERS.items():
        print(f'{k:<58}{v}')
    bad = [r for r in RESULTS if not r[1]]
    print(f'\n{len(RESULTS) - len(bad)}/{len(RESULTS)} checks passed')
    return 1 if bad else 0


def run_scenarios(work, env, procs, obs):
    # ------------------------------------------------------------ 1. startup
    info('1. startup: scheduler first, then pi (core,camera) lap1 (gpu) lap2')
    procs['sched'] = scheduler(work, env, 'sched-A')
    time.sleep(1.0)
    t_start = time.monotonic()
    procs['pi'] = agent(work, env, 'pi', 'core', 'camera')
    procs['lap1'] = agent(work, env, 'lap1', 'compute', 'gpu')
    procs['lap2'] = agent(work, env, 'lap2', 'compute', 'cpu')
    t = obs.wait_until(lambda: converged(obs), 40.0)
    check('all 7 jobs run exactly once somewhere', t is not None,
          f'running: { {j: obs.running_on(j) for j in ALL_JOBS} }')
    if t is None:
        return
    NUMBERS['1. agents spawned -> every job RUNNING (s)'] = f'{t - t_start:.2f}'
    obs.spin(1.0)
    check('core_job (role core, immovable) runs on pi', obs.running_on('core_job') == ['pi'])
    check('cam_job (tag camera) runs on pi', obs.running_on('cam_job') == ['pi'])
    check('gpu_job (prefer gpu, 2 processes) runs on lap1', obs.running_on('gpu_job') == ['lap1'])
    check('both gpu_job processes are alive and unique',
          len(live(work, 'gpu_job_a')) == 1 and len(live(work, 'gpu_job_b')) == 1)
    check('the real ROS node (validator) runs and is reported RUNNING', len(obs.running_on('validator')) == 1)
    check('extra_job is unplaceable (no host has tag extra)',
          obs.placement('extra_job') and obs.placement('extra_job')[1] == 'unplaceable',
          str(obs.placement('extra_job')))
    check('no job was ever started twice at the same time', max(obs.dup_max.values()) <= 1, str(obs.dup_max))
    hb_alive = [h for h in ('pi', 'lap1', 'lap2') if h in obs.status['alive']]
    check('status lists all three hosts alive', hb_alive == ['pi', 'lap1', 'lap2'] or sorted(hb_alive) == ['lap1', 'lap2', 'pi'],
          str(obs.status['alive']))

    # ------------------------------------------------- 2. late joiner latency
    info('2. a host with tag "extra" joins: heartbeat -> placement latency')
    procs['lap3'] = agent(work, env, 'lap3', 'compute', 'extra')
    t_hb = obs.wait_until(lambda: 'lap3' in obs.t_first, 20.0)
    check('lap3 heartbeat seen', t_hb is not None)
    t_assign = obs.wait_until(lambda: obs.last_assignment().get('extra_job') == 'lap3', 20.0)
    t_run = obs.wait_until(lambda: obs.running_on('extra_job') == ['lap3'], 20.0)
    check('extra_job assigned to lap3 and running', t_assign and t_run)
    if t_assign and t_run:
        NUMBERS['2. lap3 first heartbeat -> assignment published (s)'] = f'{t_assign - obs.t_first["lap3"]:.2f}'
        NUMBERS['2. lap3 first heartbeat -> extra_job RUNNING (s)'] = f'{t_run - obs.t_first["lap3"]:.2f}'

    # ------------------------------------- 2b. scheduler reaction latency (ghosts)
    info('2b. heartbeat -> assignment latency, 6 joins of fake hosts at random phases of the 2 Hz tick')
    good_hash = work.hash_of(work.jobs)
    lat = []
    for k in range(6):
        host = f'ghost{k}'
        obs.spin(random.random() * 0.5)                      # random phase vs the scheduler tick
        t_pub = time.monotonic()
        t_next = t_pub
        got = None
        while time.monotonic() - t_pub < 15.0 and got is None:
            if time.monotonic() >= t_next:
                obs.publish_ghost_heartbeat(host, ['probe'], good_hash)
                t_next += 0.5
            obs.spin(0.01)
            if obs.last_assignment().get('probe_job') == host:
                got = time.monotonic()
        if got is None:
            check(f'{host}: probe_job gets assigned', False)
            break
        lat.append(got - t_pub)
        # stop heartbeating; the scheduler must notice the silence and unassign before the next join
        t_gone = obs.wait_until(lambda: obs.last_assignment().get('probe_job') != host, 20.0)
        if t_gone is None:
            check(f'{host}: silent ghost is unassigned', False)
            break
    check('every ghost join was answered with an assignment', len(lat) == 6, f'{len(lat)}/6')
    if lat:
        NUMBERS['2b. heartbeat published -> assignment received: min/mean/max (s)'] = (
            f'{min(lat):.2f} / {sum(lat) / len(lat):.2f} / {max(lat):.2f}   (n={len(lat)})')
        check('reaction latency stays within one plan tick (0.5 s) plus transport', max(lat) < 0.8, str([round(x, 2) for x in lat]))

    # ------------------------------------------------ 3. mismatched catalog
    info('3. a host with a different catalog')
    procs['odd'] = agent(work, env, 'odd', 'compute', 'cpu', jobs=work.odd_jobs)
    t = obs.wait_until(lambda: obs.status and 'odd' in obs.status['excluded'], 15.0)
    check('scheduler excludes the host whose catalog hash differs', t is not None,
          str(obs.status and obs.status['excluded']))
    obs.spin(2.5)
    check('nothing is ever assigned to the excluded host', 'odd' not in obs.last_assignment().values())
    check('the excluded host runs no jobs', obs.hosts['odd'][1] == {}, str(obs.hosts['odd'][1]))
    check('its agent logged the refusal', 'REFUSING assignments' in read(procs['odd'].log))
    procs['odd'].stop()
    del procs['odd']

    # ------------------------------------------------------ 4. hard failover
    info('4. SIGKILL the agent of lap1 (hosts gpu_job)')
    on_lap1 = [j for j in ALL_JOBS if obs.running_on(j) == ['lap1']]
    old = {n: live(work, n) for n in ('gpu_job_a', 'gpu_job_b')}
    check('precondition: the processes we are about to orphan exist', all(len(v) == 1 for v in old.values()), str(old))
    t_kill = time.monotonic()
    procs['lap1'].signal(signal.SIGKILL)
    t_dead = obs.wait_until(lambda: not any(pid_alive(p) for ps in old.values() for p in ps), 10.0)
    check('lap1 job processes die with their agent (guard)', t_dead is not None)
    t_decide = obs.wait_until(lambda: obs.last_assignment().get('gpu_job') not in (None, 'lap1'), 20.0)
    t_rec = obs.wait_until(lambda: all(len(obs.running_on(j)) == 1 and obs.running_on(j) != ['lap1']
                                       for j in on_lap1), 25.0)
    check('lap1 jobs recovered on other hosts', t_rec is not None, f'jobs {on_lap1}')
    if t_dead and t_decide and t_rec:
        NUMBERS['4. SIGKILL -> old job processes dead (s)'] = f'{t_dead - t_kill:.2f}'
        NUMBERS['4. SIGKILL -> new assignment published (s)'] = f'{t_decide - t_kill:.2f}'
        NUMBERS['4. SIGKILL -> all lap1 jobs RUNNING elsewhere (s)'] = f'{t_rec - t_kill:.2f}'
        check('old instance died BEFORE the new one was placed (no overlap)', t_dead <= t_decide,
              f'dead at +{t_dead - t_kill:.2f}s, decided at +{t_decide - t_kill:.2f}s')
    check('failover happened after, not before, failover_after_sec (5 s)',
          t_decide is not None and t_decide - t_kill >= 4.0, f'{t_decide - t_kill:.2f}s' if t_decide else '')
    obs.spin(1.0)
    check('still no duplicate job instance anywhere', max(obs.dup_max.values()) <= 1, str(obs.dup_max))
    check('status marks lap1 dead', obs.status and 'lap1' in obs.status['dead'], str(obs.status and obs.status['dead']))
    check('scheduler logged the failover', 'failover gpu_job: lap1 ->' in read(procs['sched'].log))
    procs['lap1'].stop()
    if STOP_AFTER_FAILOVER:
        return

    # ---------------------------------------- 5. pinned jobs, core host dies
    info('5. SIGKILL the agent of pi (core): immovable job must NOT move')
    t_kill = time.monotonic()
    procs['pi'].signal(signal.SIGKILL)
    t = obs.wait_until(lambda: obs.placement('core_job') and obs.placement('core_job')[1] == 'unplaceable', 20.0)
    check('core_job is reported unplaceable while pi is down', t is not None, str(obs.placement('core_job')))
    if t:
        NUMBERS['5. SIGKILL pi -> core_job reported unplaceable (s)'] = f'{t - t_kill:.2f}'
    check('...with a reason that names the pinned host',
          obs.placement('core_job') and 'pinned to pi' in obs.placement('core_job')[2], str(obs.placement('core_job')))
    check('...and its run state is "unknown", not the dead host\'s stale "running"',
          obs.placement('core_job') and obs.placement('core_job')[3] == 'unknown', str(obs.placement('core_job')))
    obs.spin(3.0)
    check('core_job was NOT started on any other machine', live(work, 'core_job') == [] and obs.running_on('core_job') == [])
    check('desired state is kept: core_job still assigned to pi', obs.last_assignment().get('core_job') == 'pi')
    check('cam_job (only pi has the camera) is unplaceable too',
          obs.placement('cam_job') and obs.placement('cam_job')[1] == 'unplaceable')
    procs['pi'].stop()
    info('5b. pi comes back')
    t_back = time.monotonic()
    procs['pi'] = agent(work, env, 'pi', 'core', 'camera')
    t = obs.wait_until(lambda: obs.running_on('core_job') == ['pi'] and obs.running_on('cam_job') == ['pi'], 30.0)
    check('core_job and cam_job resume on pi', t is not None)
    if t:
        NUMBERS['5b. pi agent restarted -> core_job and cam_job RUNNING (s)'] = f'{t - t_back:.2f}'

    # ------------------------------------------------- 6. scheduler restart
    info('6. scheduler restart (SIGTERM, immediately start a new one): adoption')
    obs.wait_until(lambda: converged(obs, [j for j in ALL_JOBS]), 25.0)
    obs.spin(2.0)
    before = {j: obs.pids(j) for j in ALL_JOBS}
    sizes = {k: len(read(p.log)) for k, p in procs.items()}
    n_sets = len(obs.assign_log)
    t_stop = time.monotonic()
    procs['sched'].stop()
    procs['sched'] = scheduler(work, env, 'sched-B')
    t_first = obs.wait_until(lambda: any(e[1] == 'sched-B' for e in obs.assign_log[n_sets:]), 20.0)
    check('the new scheduler publishes', t_first is not None)
    if t_first:
        NUMBERS['6. old scheduler stopped -> first set of the new one (s)'] = f'{t_first - t_stop:.2f}'
        check('blackout is shorter than the agents orphan timeout (4 s)', t_first - t_stop < 4.0,
              f'{t_first - t_stop:.2f}s')
    obs.spin(6.0)
    after = {j: obs.pids(j) for j in ALL_JOBS}
    check('adoption: every job keeps the very same process (no restart)', before == after, f'before {before} after {after}')
    check('adoption logged', 'adopted' in read(procs['sched'].log), '')
    new_agent_logs = ''.join(read(p.log)[sizes[k]:] for k, p in procs.items() if k.startswith(('pi', 'lap')))
    check('no agent became orphaned during the restart', 'ORPHANED' not in new_agent_logs)
    check('no host was declared down by the new scheduler', 'DOWN' not in read(procs['sched'].log)
          .split('adopted')[-1])
    check('status now comes from sched-B', obs.status and obs.status['sid'] == 'sched-B')

    # -------------------------------------------------- 7. scheduler frozen
    info('7. scheduler frozen for 7 s (SIGSTOP) then resumed')
    host_of = {j: obs.running_on(j)[0] for j in ALL_JOBS}
    core_pid = obs.pids('core_job')
    sizes = {k: len(read(p.log)) for k, p in procs.items()}
    procs['sched'].signal(signal.SIGSTOP)
    obs.spin(7.0)
    stopped_logs = ''.join(read(p.log)[sizes[k]:] for k, p in procs.items() if k.startswith(('pi', 'lap')))
    check('agents orphan their movable jobs while the scheduler is gone', 'ORPHANED' in stopped_logs)
    check('...but the immovable core_job keeps running (fail-static), same pid',
          obs.pids('core_job') == core_pid and len(live(work, 'core_job')) == 1)
    sched_size = len(read(procs['sched'].log))
    procs['sched'].signal(signal.SIGCONT)
    t = obs.wait_until(lambda: converged(obs), 40.0)
    check('after SIGCONT everything runs again', t is not None)
    obs.spin(2.0)
    after_host = {j: (obs.running_on(j) or [None])[0] for j in ALL_JOBS}
    check('each job is back on the SAME host (no failover storm)', after_host == host_of, f'{host_of} -> {after_host}')
    log = read(procs['sched'].log)[sched_size:]
    check('scheduler noticed its own stall', 'stalled' in log, '')
    check('no host was declared DOWN and nothing failed over after the freeze',
          'DOWN' not in log and 'failover' not in log, log.strip()[-300:])
    check('core_job process never restarted', obs.pids('core_job') == core_pid)
    check('no duplicate job instance during freeze/resume', max(obs.dup_max.values()) <= 1, str(obs.dup_max))

    # ----------------------------------------- 8. unknown ids / wrong hash
    info('8. refusal of unknown job ids and of a wrong catalog hash (rogue publisher)')
    for k in [k for k in procs if k != 'victim']:
        procs[k].stop()
    procs.clear()
    time.sleep(1.0)
    procs['victim'] = agent(work, env, 'victim', 'compute', 'cpu')
    good = work.hash_of(work.jobs)
    obs.wait_until(lambda: 'victim' in obs.hosts, 15.0)
    t_end = time.monotonic() + 3.5
    while time.monotonic() < t_end:                              # wrong hash, valid job id
        obs.publish_set('rogue', 1, 'deadbeefdeadbeef', [('svc_a', 'victim')])
        obs.spin(0.5)
    check('a set with the wrong catalog hash starts nothing', live(work, 'svc_a') == [])
    check('the agent logged REFUSING', 'REFUSING assignments from rogue' in read(procs['victim'].log))
    t_end = time.monotonic() + 3.5
    while time.monotonic() < t_end:                              # right hash, one valid and two bad ids
        obs.publish_set('rogue', 2, good, [('svc_a', 'victim'), ('rm -rf', 'victim'), ('../evil', 'victim')])
        obs.spin(0.5)
    check('a valid job id from the local catalog does start', len(live(work, 'svc_a')) == 1)
    check('ids that are not in the local catalog are ignored and logged',
          'ignoring job ids not in the local catalog' in read(procs['victim'].log))
    check('nothing except svc_a runs on the victim', all(live(work, n) == [] for n in SLEEPERS if n != 'svc_a'))
    obs.publish_set('rogue', 3, good, [])
    t = obs.wait_until(lambda: live(work, 'svc_a') == [], 15.0)
    check('an empty set (desired state) stops the job again', t is not None)


if __name__ == '__main__':
    sys.exit(main())
