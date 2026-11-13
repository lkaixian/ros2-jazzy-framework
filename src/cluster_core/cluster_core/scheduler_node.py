"""cluster_scheduler_node: the single orchestrator. Publishes the desired placement.

* hosts are tracked by heartbeat RECEIVE time (agents' clocks are not trusted)
* a host is dead after `failover_after_sec` of silence; this must exceed the agents'
  `orphan_timeout_sec` so a partitioned host has already stopped its movable jobs
* after a restart it waits `adopt_wait_sec`, then ADOPTS what the heartbeats say is running
  instead of restarting everything
* it is a single point of failure for CHANGES only: agents keep their jobs while it is away
"""
import os
import socket
import time
from collections import deque

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cluster_msgs.msg import Assignment, AssignmentSet, ClusterStatus, JobPlacement, JobStatus, NodeInfo
from rclpy.node import Node

from cluster_core import placement
from cluster_core.catalog import CatalogError, jobdef_from_spec, load_catalog
from cluster_core.qos import INFO_QOS, LATCHED_QOS, TOPIC_ASSIGN, TOPIC_INFO, TOPIC_STATUS
from cluster_core.supervisor import NAMES as JOB_STATE_NAMES


class SchedulerNode(Node):
    def __init__(self):
        super().__init__('cluster_scheduler_node')
        defaults = {'scheduler_id': '', 'jobs_file': '', 'params_dir': '',
                    'failover_after_sec': 5.0, 'agent_orphan_timeout_sec': 4.0,
                    'adopt_wait_sec': 1.0, 'plan_hz': 2.0, 'publish_hz': 1.0,
                    'overload': 0.85, 'underload': 0.6, 'hysteresis_sec': 10.0,
                    'cooldown_sec': 60.0, 'quarantine_sec': 60.0, 'settle_sec': 10.0,
                    'forget_after_sec': 300.0}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        if p('failover_after_sec') < p('agent_orphan_timeout_sec') + 0.5:
            raise ValueError(
                f"failover_after_sec ({p('failover_after_sec')}) must exceed the agents' "
                f"orphan_timeout_sec ({p('agent_orphan_timeout_sec')}) by at least 0.5 s: otherwise "
                'a partitioned host may still run a job when it is started elsewhere')
        if p('adopt_wait_sec') > p('agent_orphan_timeout_sec') - 2.5:
            self.get_logger().warn(
                f"adopt_wait_sec ({p('adopt_wait_sec')}) is too close to the agents' orphan timeout "
                f"({p('agent_orphan_timeout_sec')}): a scheduler restart will make agents stop their "
                'movable jobs before adoption can keep them')
        self.id = p('scheduler_id') or f'{socket.gethostname()}-{os.getpid()}'
        params_dir = p('params_dir')
        if not params_dir:
            try:
                params_dir = os.path.join(get_package_share_directory('framework_bringup'), 'param')
            except PackageNotFoundError:
                params_dir = ''
        self.catalog = load_catalog(p('jobs_file') or os.path.join(params_dir, 'cluster_jobs.yaml'))
        self.jobdefs = [jobdef_from_spec(self.catalog[j]) for j in self.catalog.ids()]
        self.cfg = placement.Config(p('overload'), p('underload'), p('hysteresis_sec'),
                                    p('cooldown_sec'), p('quarantine_sec'))
        self.pstate = placement.State()
        self.hosts = {}                 # id -> {'msg': NodeInfo, 'rx': monotonic}
        self.host_alive = {}
        self.current = None             # None until adopted
        self.assigned_at = {}           # job -> (host, monotonic)
        self.placements = {}
        self.epoch = 0
        self.events = deque(maxlen=20)
        self.t0 = time.monotonic()
        self.last_tick = None
        self.last_publish = 0.0
        self.foreign_warned = 0.0

        self.assign_pub = self.create_publisher(AssignmentSet, TOPIC_ASSIGN, LATCHED_QOS)
        self.status_pub = self.create_publisher(ClusterStatus, TOPIC_STATUS, LATCHED_QOS)
        self.create_subscription(NodeInfo, TOPIC_INFO, self._on_info, INFO_QOS)
        self.create_subscription(AssignmentSet, TOPIC_ASSIGN, self._on_foreign, LATCHED_QOS)
        self.create_timer(1.0 / p('plan_hz'), self._tick)
        self.get_logger().info(
            f'scheduler {self.id}: {len(self.jobdefs)} jobs, catalog {self.catalog.hash}, '
            f"failover after {p('failover_after_sec')}s, adopting in {p('adopt_wait_sec')}s")

    # --------------------------------------------------------------- inputs
    def _on_info(self, msg):
        if not msg.host_id:
            return
        if msg.host_id not in self.hosts:
            self.get_logger().info(f'host joined: {msg.host_id} roles={list(msg.roles)} '
                                   f'tags={list(msg.tags)} cores={msg.cpu_cores}')
        self.hosts[msg.host_id] = {'msg': msg, 'rx': time.monotonic()}

    def _on_foreign(self, msg):
        now = time.monotonic()
        if msg.scheduler_id != self.id and now - self.foreign_warned > 5.0:
            self.foreign_warned = now
            self.get_logger().error(
                f'another scheduler ({msg.scheduler_id}) is publishing assignments: run only one')

    # ---------------------------------------------------------------- planning
    def _host_infos(self, now):
        fa = self.get_parameter('failover_after_sec').value
        forget = self.get_parameter('forget_after_sec').value
        infos = []
        for hid in sorted(self.hosts):
            rec = self.hosts[hid]
            age = now - rec['rx']
            if age > forget:
                del self.hosts[hid]
                self.host_alive.pop(hid, None)
                continue
            m = rec['msg']
            alive = age <= fa
            if self.host_alive.get(hid) != alive:
                if hid in self.host_alive or not alive:
                    self._event(f"host {hid} {'UP' if alive else 'DOWN (silent %.1fs)' % age}")
                self.host_alive[hid] = alive
            infos.append(placement.HostInfo(
                id=hid, roles=frozenset(m.roles), tags=frozenset(m.tags), cores=int(m.cpu_cores),
                cpu_load=float(m.cpu_load), mem_total_mb=float(m.mem_total_mb),
                mem_free_mb=float(m.mem_free_mb), gpu_mem_free_mb=float(m.gpu_mem_free_mb),
                alive=alive, catalog_ok=(m.catalog_hash == self.catalog.hash),
                failed_jobs=frozenset(j.job_id for j in m.jobs if j.state == JobStatus.FAILED)))
        return infos

    def _event(self, text):
        self.events.append(text)
        self.get_logger().info(text)

    def _adopt(self):
        reported = {}
        for hid, rec in self.hosts.items():
            m = rec['msg']
            if m.catalog_hash == self.catalog.hash:
                reported[hid] = [j.job_id for j in m.jobs
                                 if j.state in (JobStatus.RUNNING, JobStatus.BACKOFF)]
        self.current = placement.adopt(self.catalog.ids(), reported)
        now = time.monotonic()
        for jid, hid in self.current.items():
            self.assigned_at[jid] = (hid, now - 1e6)     # already settled
        self._event(f'adopted {len(self.current)} running job(s) from heartbeats: '
                    + ', '.join(f'{j}@{h}' for j, h in sorted(self.current.items())))

    def _tick(self):
        now = time.monotonic()
        gap = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if gap > max(2.0, 3.0 / self.get_parameter('plan_hz').value):
            # We were frozen (VM pause, SIGSTOP, ...): every host looks silent only because WE
            # were not listening. Give the host timers back the time we could not observe, or the
            # first tick after the pause would declare the whole cluster dead.
            for rec in self.hosts.values():
                rec['rx'] = min(now, rec['rx'] + gap)
            self._event(f'scheduler stalled for {gap:.1f}s: host liveness timers extended')
        if now - self.t0 < self.get_parameter('adopt_wait_sec').value:
            return
        if self.current is None:
            self._adopt()
        infos = self._host_infos(now)
        settle = self.get_parameter('settle_sec').value
        unsettled = set()
        for jid, hid in self.current.items():
            rec = self.hosts.get(hid)
            running = rec is not None and any(
                j.job_id == jid and j.state == JobStatus.RUNNING for j in rec['msg'].jobs)
            at = self.assigned_at.get(jid)
            if not running and at and at[0] == hid and now - at[1] < settle:
                unsettled.add(jid)
        plan = placement.plan(self.jobdefs, infos, self.current, now, self.pstate, self.cfg,
                              frozenset(unsettled))
        for e in plan.events:
            self._event(e)
        changed = plan.assignment != self.current
        if changed:
            for jid, hid in plan.assignment.items():
                if self.current.get(jid) != hid:
                    self.assigned_at[jid] = (hid, now)
                    self._event(f'assign {jid} -> {hid}')
            for jid in set(self.current) - set(plan.assignment):
                self._event(f'unassign {jid}')
            self.epoch += 1
        self.current = dict(plan.assignment)
        self.placements = plan.placements
        if changed or now - self.last_publish >= 1.0 / self.get_parameter('publish_hz').value:
            self._publish(infos, now)

    def _publish(self, infos, now):
        self.last_publish = now
        stamp = self.get_clock().now().to_msg()
        a = AssignmentSet(scheduler_id=self.id, epoch=self.epoch, catalog_hash=self.catalog.hash)
        a.header.stamp = stamp
        a.assignments = [Assignment(job_id=j, host_id=h) for j, h in sorted(self.current.items())]
        self.assign_pub.publish(a)

        s = ClusterStatus(scheduler_id=self.id, epoch=self.epoch)
        s.header.stamp = stamp
        s.hosts_alive = [h.id for h in infos if h.alive and h.catalog_ok]
        s.hosts_dead = [h.id for h in infos if not h.alive]
        s.hosts_excluded = [h.id for h in infos if h.alive and not h.catalog_ok]
        for jid in self.catalog.ids():
            pl = self.placements.get(jid, placement.Placement('pending', '', 'not planned yet'))
            s.jobs.append(JobPlacement(job_id=jid, host_id=pl.host, state=pl.state,
                                       reason=pl.reason, job_state=self._job_state(jid, pl.host)))
        s.events = list(self.events)
        self.status_pub.publish(s)

    def _job_state(self, jid, hid):
        rec = self.hosts.get(hid) if hid else None
        if rec is None or not self.host_alive.get(hid, False):
            return 'unknown'              # a silent host's last heartbeat says nothing current
        for j in rec['msg'].jobs:
            if j.job_id == jid:
                return JOB_STATE_NAMES.get(j.state, 'unknown')
        return 'stopped'


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SchedulerNode()
    except (CatalogError, ValueError, OSError) as e:
        print(f'cluster_scheduler_node: cannot start: {e}')
        rclpy.try_shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
