#!/usr/bin/env python3
"""Sim check of the task lifecycle + repair loop (fake LLM, dry-run executor, no robot).

The fake llama server always answers with a plan whose DetectPerson condition fails (nobody is
there). With max_repairs=1 the orchestrator must: plan -> run -> see the failure at DetectPerson ->
ask the LLM for a repaired plan on bt/repair -> run it -> give up (failed, repairs 1/1).
Domain REPAIR_DOMAIN (default 80). Exit code 0 only if every check passed.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('REPAIR_DOMAIN', '80')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = {'goal': 'find a person', 'steps': [{'node': 'DetectPerson', 'ports': {}}]}


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    work = tempfile.mkdtemp(prefix='repair_')
    params = os.path.join(work, 'param')
    shutil.copytree(os.path.join(get_package_share_directory('framework_bringup'), 'param'), params)
    f = os.path.join(params, 'pipeline.yaml')
    txt = open(f).read()
    assert 'max_repairs: 0' in txt
    # recovery_policy off: this test checks the LLM re-plan path; with the policy on, the first failure is a plain retry (mode_test.py)
    open(f, 'w').write(txt.replace('max_repairs: 0', 'max_repairs: 1\n    recovery_policy: false'))
    spec_file = os.path.join(work, 'spec.json')
    json.dump(SPEC, open(spec_file, 'w'))
    procs, res = [], {}
    rclpy.init()
    n = None
    try:
        procs.append(subprocess.Popen([sys.executable, os.path.join(HERE, 'fake_llama2.py')],
                                      env={**os.environ, 'SPEC': spec_file},
                                      stdout=subprocess.DEVNULL, start_new_session=True))
        procs.append(subprocess.Popen(
            ['ros2', 'launch', 'framework_bringup', 'pipeline.launch.py', 'dry_run:=true',
             'auto_run:=true', f'params_dir:={params}'], start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        n = rclpy.create_node('repair_probe')
        states, repairs = [], []
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(String, 'task/state', lambda m: states.append(json.loads(m.data)), tl)
        n.create_subscription(String, 'bt/repair', lambda m: repairs.append(m.data), 10)
        task = n.create_publisher(String, 'bt/task', 10)

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)
        spin(12.0)                                       # let the pipeline come up
        task.publish(String(data='find a person'))
        end = time.monotonic() + 40
        while time.monotonic() < end and not (states and states[-1]['state'] == 'failed'):
            spin(0.3)
        seq = [s['state'] for s in states]
        res['lifecycle: planning, ready, running'] = all(x in seq for x in ('planning', 'ready', 'running'))
        res['failed run triggers a repair request'] = 'repairing' in seq and len(repairs) == 1
        res['repair request names the failed node'] = bool(repairs) and 'DetectPerson' in repairs[0]
        res['repaired plan was re-planned and re-run'] = seq.count('running') == 2
        res['budget spent: failed with repairs 1/1'] = bool(states) and states[-1]['state'] == 'failed' \
            and states[-1]['repairs'] == 1
        res['no third attempt'] = len(repairs) == 1
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        if 'seq' in dir():
            print('states:', ' '.join(seq))
        for p in procs:
            kill(p, signal.SIGINT)
        time.sleep(3)
        for p in procs:
            kill(p)
        if n is not None:
            n.destroy_node()
        rclpy.try_shutdown()
        shutil.rmtree(work, ignore_errors=True)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
