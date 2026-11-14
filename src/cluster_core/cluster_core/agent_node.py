"""cluster_agent_node: one per machine. Heartbeats, and reconciles local jobs to the desired state.

Safety properties (see docs/cluster.md):
* runs ONLY jobs from the local catalog; assignments carry ids, never commands
* refuses assignment sets whose catalog hash differs from the local catalog
* acts on a set only after `min_rx_before_act` receptions, so a stale latched set replayed by DDS
  to a freshly started agent is not obeyed unless the scheduler is really alive
* ORPHAN: if no accepted set arrived for `orphan_timeout_sec`, movable jobs (stop_on_orphan) are
  stopped, so a partitioned host stops before the scheduler restarts them elsewhere. Immovable
  (hardware-bound) jobs keep running (fail-static) until a scheduler says otherwise.
"""
import os
import socket
import time

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cluster_msgs.msg import AssignmentSet, JobStatus, NodeInfo, ServiceEndpoint
from cluster_msgs.srv import HostInfo
from rclpy.node import Node

from cluster_core.catalog import CatalogError, expand, load_catalog
from cluster_core.endpoints import advertised, local_address
from cluster_core.qos import INFO_QOS, LATCHED_QOS, TOPIC_ASSIGN, TOPIC_INFO
from cluster_core.supervisor import Supervisor
from cluster_core.sysinfo import host_service_name, Sampler


def split_csv(text):
    return sorted({t.strip() for t in (text or '').split(',') if t.strip()})


def default_param_dir():
    try:
        return os.path.join(get_package_share_directory('framework_bringup'), 'param')
    except PackageNotFoundError:
        return ''


class AgentNode(Node):
    def __init__(self):
        super().__init__('cluster_agent_node')
        defaults = {'host_id': '', 'roles': '', 'tags': '', 'jobs_file': '', 'params_dir': '',
                    'log_dir': '', 'heartbeat_hz': 2.0, 'poll_hz': 10.0,
                    'orphan_timeout_sec': 4.0, 'min_rx_before_act': 2, 'use_guard': True,
                    'advertise_host': ''}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.host = p('host_id') or socket.gethostname()
        self.roles, self.tags = split_csv(p('roles')), split_csv(p('tags'))
        params_dir = p('params_dir') or default_param_dir()
        jobs_file = p('jobs_file') or os.path.join(params_dir, 'cluster_jobs.yaml')
        self.catalog = load_catalog(jobs_file)           # invalid catalog: fail at start, loudly
        self.vars = {'PARAMS_DIR': params_dir}
        for jid in self.catalog.ids():                   # also fail early on unknown ${VARS}
            for argv in self.catalog[jid]['cmds']:
                expand(argv, self.vars)
        log_dir = p('log_dir') or os.path.expanduser(f'~/.ros/cluster/{self.host}')
        # address other machines reach this host on (for jobs that declare `endpoint:`)
        self.address = p('advertise_host') or local_address()
        self.sup = Supervisor(log_dir, use_guard=p('use_guard'))
        self.sampler = Sampler()
        try:
            self.sampler.sample()                        # prime the cpu meter
        except OSError:
            pass
        self.desired = None            # set of job ids, None until a set was accepted
        self.epoch = 0
        self.last_rx = None
        self.rx_count = 0
        self.orphaned = False
        self.seq = 0

        self.info_pub = self.create_publisher(NodeInfo, TOPIC_INFO, INFO_QOS)
        self.last_info = None
        # one service per machine (names cannot clash across hosts); `ros2 run cluster_core host_info` reads them
        self.create_service(HostInfo, host_service_name(self.host), self._host_info)
        self.create_subscription(AssignmentSet, TOPIC_ASSIGN, self._on_assign, LATCHED_QOS)
        self.create_timer(1.0 / p('heartbeat_hz'), self._heartbeat)
        self.create_timer(1.0 / p('poll_hz'), self._tick)
        self.get_logger().info(
            f'agent {self.host} roles={self.roles} tags={self.tags} catalog={self.catalog.hash} '
            f'({len(self.catalog.jobs)} jobs) logs={log_dir}')

    # ------------------------------------------------------------ assignments
    def _on_assign(self, msg):
        if msg.catalog_hash != self.catalog.hash:
            self.get_logger().error(
                f'REFUSING assignments from {msg.scheduler_id}: catalog hash {msg.catalog_hash} '
                f'differs from local {self.catalog.hash}', throttle_duration_sec=10.0)
            return
        self.last_rx = time.monotonic()
        self.rx_count += 1
        if self.orphaned:
            self.get_logger().info('scheduler is back; resuming reconciliation')
            self.orphaned = False
        mine = {a.job_id for a in msg.assignments if a.host_id == self.host}
        unknown = sorted(j for j in mine if j not in self.catalog)
        if unknown:
            self.get_logger().warn(f'ignoring job ids not in the local catalog: {unknown}',
                                   throttle_duration_sec=10.0)
            mine -= set(unknown)
        if self.rx_count < self.get_parameter('min_rx_before_act').value:
            return
        self.desired, self.epoch = mine, msg.epoch
        self._reconcile()

    def _reconcile(self):
        if self.desired is None or self.orphaned:
            return
        for jid in sorted(self.desired):
            if jid not in self.sup.jobs:                 # absent in every state incl. FAILED
                spec = self.catalog[jid]
                cmds = [expand(a, self.vars) for a in spec['cmds']]
                self.sup.start(jid, cmds, spec['env'], spec['restart'], spec['grace_sec'])
                self.get_logger().info(f'start {jid}')
        for jid in sorted(self.sup.active_ids() - self.desired):
            self.sup.stop(jid)
            self.get_logger().info(f'stop {jid} (no longer assigned here)')

    # ------------------------------------------------------------------ timers
    def _tick(self):
        self.sup.poll()
        timeout = self.get_parameter('orphan_timeout_sec').value
        if (self.last_rx is not None and not self.orphaned
                and time.monotonic() - self.last_rx > timeout):
            self.orphaned = True
            movers = sorted(j for j in self.sup.active_ids()
                            if j in self.catalog and self.catalog[j]['stop_on_orphan'])
            self.get_logger().warn(
                f'no accepted assignment set for {timeout:.1f}s: ORPHANED; stopping movable jobs '
                f'{movers}; keeping immovable ones')
            for jid in movers:
                self.sup.stop(jid)
        elif not self.orphaned:
            self._reconcile()                            # restarts after backoff are in poll()

    def _host_info(self, _req, res):
        res.info = self.last_info or NodeInfo(host_id=self.host)
        return res

    def _heartbeat(self):
        m = NodeInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.host_id, m.roles, m.tags = self.host, self.roles, self.tags
        try:
            s = self.sampler.sample()
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'metrics unavailable: {e}', throttle_duration_sec=30.0)
            s = {'cores': os.cpu_count() or 1, 'cpu_load': 0.0, 'mem_total_mb': 0.0,
                 'mem_free_mb': 0.0, 'gpu_mem_total_mb': -1.0, 'gpu_mem_free_mb': -1.0,
                 'gpu_util': -1.0, 'temp_c': -1.0, 'battery_pct': -1.0, 'charging': -1}
        m.cpu_cores = int(s['cores'])
        m.cpu_load, m.mem_total_mb, m.mem_free_mb = (float(s['cpu_load']), float(s['mem_total_mb']),
                                                     float(s['mem_free_mb']))
        m.gpu_mem_total_mb, m.gpu_mem_free_mb = float(s['gpu_mem_total_mb']), float(s['gpu_mem_free_mb'])
        m.gpu_util, m.temp_c = float(s['gpu_util']), float(s['temp_c'])
        m.battery_pct, m.charging = float(s['battery_pct']), int(s['charging'])
        m.catalog_hash = self.catalog.hash
        self.seq += 1
        m.seq, m.assignment_epoch = self.seq, int(self.epoch)
        sts = self.sup.statuses()
        for st in sts:
            m.jobs.append(JobStatus(job_id=st['job_id'], state=st['state'],
                                    restarts=st['restarts'], pid=st['pid'],
                                    last_error=st['last_error'][:200]))
        for jid, kind, url, prio in advertised(self.catalog, sts, self.address):
            m.endpoints.append(ServiceEndpoint(job_id=jid, kind=kind, url=url, priority=prio))
        self.last_info = m
        self.info_pub.publish(m)

    def destroy_node(self):
        self.sup.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AgentNode()
    except (CatalogError, OSError) as e:
        print(f'cluster_agent_node: cannot start: {e}')
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
