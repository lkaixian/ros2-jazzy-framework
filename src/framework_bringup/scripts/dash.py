#!/usr/bin/python3
"""dash: terminal view of what the framework is doing.

  ros2 run framework_bringup dash            # live status table (1 Hz): nodes, LLM endpoints, cluster
  ros2 run framework_bringup dash --logs     # verbose: every /rosout line, coloured by level
  ros2 run framework_bringup dash --logs --level warn --node llm_bridge_node
  ros2 run framework_bringup dash --once     # one frame, no clearing (pipes, CI)

Node states: ONLINE (in the ROS graph), BUSY (bt_executor running a tree), ERROR (logged an
error in the last 30 s, or llm_bridge reports `down`), OFFLINE (expected but not in the graph).
Expected nodes = the table in check_params. Read-only: it publishes nothing.
"""
import argparse
import importlib.machinery
import json
import os
import sys
import time

import rclpy
from cluster_msgs.msg import ClusterStatus
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ERR_WINDOW = 30.0
COL = {'ONLINE': '32', 'BUSY': '36', 'ERROR': '31', 'OFFLINE': '90', 'WARN': '33'}
LEVEL = {10: ('DEBUG', '90'), 20: ('INFO', '0'), 30: ('WARN', '33'), 40: ('ERROR', '31'),
         50: ('FATAL', '1;31')}
LEVEL_NUM = {'debug': 10, 'info': 20, 'warn': 30, 'error': 40, 'fatal': 50}
TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)


def classify(online, err_age, busy=False, down=False):
    """err_age: seconds since the node's last ERROR/FATAL log, or None."""
    if not online:
        return 'OFFLINE'
    if down or (err_age is not None and err_age < ERR_WINDOW):
        return 'ERROR'
    return 'BUSY' if busy else 'ONLINE'


def paint(text, code, color=True):
    return f'\033[{code}m{text}\033[0m' if color else text


def expected_nodes():
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        mod = importlib.machinery.SourceFileLoader(
            'check_params', os.path.join(here, 'check_params')
            if os.path.exists(os.path.join(here, 'check_params')) else
            os.path.join(here, 'check_params.py')).load_module()
        return sorted(mod.NODES) + sorted(mod.PERCEPTION)
    except Exception:  # noqa: BLE001  shortcut: no expected list -> only show nodes that exist
        return []


class Dash:
    def __init__(self, node, color):
        self.n, self.color = node, color
        self.err = {}          # node name -> (monotonic time, message)
        self.llm = {}
        self.bt = {}
        self.task = {}
        self.cluster = None
        node.create_subscription(Log, '/rosout', self._log, 100)
        node.create_subscription(String, 'llm/status', lambda m: self._js('llm', m), TL)
        node.create_subscription(String, 'bt/status', lambda m: self._js('bt', m), 10)
        node.create_subscription(String, 'task/state', lambda m: self._js('task', m), TL)
        node.create_subscription(ClusterStatus, 'cluster/status',
                                 lambda m: setattr(self, 'cluster', m), TL)

    def _js(self, attr, m):
        try:
            setattr(self, attr, json.loads(m.data))
        except ValueError:
            pass

    def _log(self, m):
        if m.level >= 40:
            self.err[m.name] = (time.monotonic(), m.msg)

    def frame(self, expected):
        c, now = self.color, time.monotonic()
        online = {n for n, _ in self.n.get_node_names_and_namespaces()}
        online = {n for n in online if not n.startswith('_ros2cli')}   # the CLI's own daemon
        names = sorted(set(expected) | online - {self.n.get_name()})
        out = [paint(time.strftime('framework  %H:%M:%S'), '1', c), '']
        # LLM
        st = self.llm.get('state', '?')
        out.append(paint(f'LLM  {st}', COL['ERROR'] if st == 'down' else
                         COL['WARN'] if st == 'degraded' else COL['ONLINE'], c) +
                   f"   {self.llm.get('reason', '')}")
        for e in self.llm.get('endpoints', []):
            out.append('   ' + paint('up  ' if e['available'] else 'DOWN',
                                     COL['ONLINE'] if e['available'] else COL['ERROR'], c) +
                       f"  {e['url']}")
        # task lifecycle (orchestrator)
        t = self.task
        if t:
            ts = t.get('state', '?')
            out.append(paint(f'task {ts}', COL['ERROR'] if ts == 'failed' else
                             COL['BUSY'] if ts in ('running', 'repairing') else COL['ONLINE'], c) +
                       f"   repairs {t.get('repairs', 0)}/{t.get('max_repairs', 0)}  "
                       f"{t.get('detail', '') or t.get('task', '')[:50]}")
        # cluster
        if self.cluster:
            cs = self.cluster
            out += ['', f"cluster  scheduler {cs.scheduler_id}  alive: {', '.join(cs.hosts_alive) or '-'}"
                        f"  dead: {', '.join(cs.hosts_dead) or '-'}"]
            for j in cs.jobs:
                col = {'running': 'ONLINE', 'backoff': 'WARN', 'failed': 'ERROR'}.get(
                    j.job_state, 'OFFLINE' if j.state != 'placed' else 'ONLINE')
                out.append(f'   {j.job_id:<18} {j.host_id or "-":<12} ' +
                           paint(j.job_state if j.state == "placed" else j.state, COL[col], c))
        # nodes
        out += ['', 'nodes']
        busy_node = 'bt_executor_node' if self.bt.get('state') == 'running' else None
        for name in names:
            err = self.err.get(name)
            s = classify(name in online, now - err[0] if err else None,
                         busy=name == busy_node,
                         down=name == 'llm_bridge_node' and self.llm.get('state') == 'down')
            note = (err[1][:70] if err and s == 'ERROR' else
                    self.bt.get('message', '') if s == 'BUSY' else '')
            out.append(f'   {name:<30}' + paint(f'{s:<8}', COL[s], c) + note)
        return '\n'.join(out)


def logs(node, a, color):
    floor = LEVEL_NUM[a.level]

    def show(m):
        if m.level < floor or (a.node and a.node not in m.name):
            return
        lvl, code = LEVEL.get(m.level, (str(m.level), '0'))
        t = time.strftime('%H:%M:%S', time.localtime(m.stamp.sec))
        print(f'{t} ' + paint(f'{lvl:<5}', code, color) + f' {m.name}: {m.msg}', flush=True)
    node.create_subscription(Log, '/rosout', show, 1000)
    rclpy.spin(node)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--logs', action='store_true', help='verbose: stream /rosout')
    ap.add_argument('--level', default='info', choices=sorted(LEVEL_NUM, key=LEVEL_NUM.get))
    ap.add_argument('--node', default='', help='only logs from nodes whose name contains this')
    ap.add_argument('--once', action='store_true', help='print one frame and exit')
    a = ap.parse_args(argv)
    color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
    rclpy.init()
    node = rclpy.create_node('framework_dash')
    try:
        if a.logs:
            logs(node, a, color)
            return 0
        d, exp = Dash(node, color), expected_nodes()
        end = time.monotonic() + 2.5      # let latched topics and the graph arrive
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        while True:
            frame = d.frame(exp)
            print(frame if a.once else '\033[H\033[J' + frame, flush=True)
            if a.once:
                return 0
            t = time.monotonic() + 1.0
            while time.monotonic() < t:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
