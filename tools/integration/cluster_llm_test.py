#!/usr/bin/env python3
"""Real cluster agent + scheduler + llm_bridge: a host that runs an LLM job joins, then leaves.

No fake heartbeats: the job `fake_llm` (tools/integration/fake_llm_server.py) is placed by the real
scheduler, started by the real agent, advertised in its heartbeat; the bridge (cluster endpoints
only) follows it. Then the whole host is killed: the endpoint must disappear and, with nothing
left, the bridge must send the emergency halt. Domain CLUSTER_LLM_DOMAIN (default 79).
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('CLUSTER_LLM_DOMAIN', '79')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
import yaml  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

PORT = 18391
HERE = os.path.dirname(os.path.abspath(__file__))


def start(argv):
    return subprocess.Popen(argv, start_new_session=True)


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    work = tempfile.mkdtemp(prefix='cluster_llm_')
    jobs = os.path.join(work, 'jobs.yaml')
    yaml.safe_dump({'jobs': {'fake_llm': {
        'cmd': [sys.executable, os.path.join(HERE, 'fake_llm_server.py'), str(PORT), 'realagent'],
        'endpoint': {'kind': 'llm', 'port': PORT, 'priority': 10},
        'cpu': 0.1, 'mem_mb': 50}}}, open(jobs, 'w'))
    procs = []
    res = {}
    rclpy.init()
    n = None
    try:
        agent = start([sys.executable, '-m', 'cluster_core.agent_node', '--ros-args',
                       '-p', 'host_id:=strong-pc', '-p', 'roles:=compute', '-p', f'jobs_file:={jobs}',
                       '-p', 'advertise_host:=127.0.0.1', '-p', f'log_dir:={work}/logs',
                       '-p', 'params_dir:=/tmp'])
        sched = start([sys.executable, '-m', 'cluster_core.scheduler_node', '--ros-args',
                       '-p', f'jobs_file:={jobs}', '-p', 'scheduler_id:=s1', '-p', 'params_dir:=/tmp'])
        bridge = start(['ros2', 'run', 'llm_bridge', 'llm_bridge_node', '--ros-args',
                        '-p', 'use_static_urls:=false', '-p', 'probe_period_sec:=1.0',
                        '-p', 'startup_grace_sec:=0.0', '-p', 'connect_timeout_sec:=1.0'])
        procs = [agent, sched, bridge]
        n = rclpy.create_node('cluster_llm_probe')
        specs, states, halts = [], [], []
        n.create_subscription(String, 'bt/task_spec', lambda m: specs.append(json.loads(m.data)), 10)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(String, 'llm/status', lambda m: states.append(json.loads(m.data)), q)

        def halt(_req, resp):
            halts.append(time.monotonic())
            resp.success, resp.message = True, 'halted (test)'
            return resp
        n.create_service(Trigger, 'bt_executor/halt', halt)
        task = n.create_publisher(String, 'bt/task', 10)

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)

        def wait(cond, sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                spin(0.2)
                if cond():
                    return True
            return False

        def urls():
            return [e['url'] for e in (states[-1]['endpoints'] if states else [])]

        url = f'http://127.0.0.1:{PORT}'
        res['host joined: scheduler placed + agent started + endpoint advertised'] = wait(
            lambda: url in urls(), 40)
        before = len(specs)
        res['task served by the joined host'] = wait(
            lambda: (task.publish(String(data='go')) or True) and len(specs) > before, 10) and \
            specs[-1]['goal'] == 'realagent'
        t_served = time.monotonic()
        spin(3.0)
        res['no halt while the host is alive'] = not [h for h in halts if h >= t_served]
        t0 = time.monotonic()
        kill(agent)                      # whole host dies: agent and (guarded) job
        os.system(f"pkill -f 'fake_llm_server.py {PORT}'")
        res['host left: endpoint removed within ttl'] = wait(lambda: url not in urls(), 12)
        res['host left: nothing to serve -> emergency halt'] = wait(
            lambda: any(h > t0 for h in halts), 12)
        res['host left: state down'] = bool(states) and states[-1]['state'] == 'down'
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for p in procs:
            kill(p, signal.SIGINT)
        time.sleep(3)
        for p in procs:
            kill(p)
        os.system(f"pkill -f 'fake_llm_server.py {PORT}'")
        if n is not None:
            n.destroy_node()
        rclpy.try_shutdown()
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
