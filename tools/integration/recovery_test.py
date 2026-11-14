#!/usr/bin/env python3
"""Recovery policy against a LIVE executor (dry_run:=false, fake extensions that fail on command), own DDS domain.
Real: bt_validator, bt_slm (rules only), orchestrator (max_repairs 3), bt_executor. Fake: nav + social extensions (fake_recovery_ext.py).
  A  one blocked goto (2 goals)  -> retry the same plan                          -> done after 1 recovery
  B  a place that stays blocked  -> retry, replan (this test answers as the LLM), go_home -> done after 3, `start` reached
  C  a detection that never sees -> retry, replan, ask_human (Speak 'Please help')      -> done after 3, the sentence is spoken
Workspace sourced: python3 tools/integration/recovery_test.py"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
GOTO = {'goal': 'go to the door', 'steps': [{'macro': 'go_to_place', 'args': {'place': 'door'}}]}
LOOK = {'goal': 'look for a person', 'steps': [{'node': 'DetectPerson'}]}


def main():
    os.environ['ROS_DOMAIN_ID'] = '90'
    ros = ['--ros-args', '-p']
    cmds = [['ros2', 'run', 'bt_validator', 'bt_validator_node'],
            ['ros2', 'run', 'bt_slm', 'bt_slm_node'] + ros + ['model_path:=none'],
            ['ros2', 'run', 'bt_orchestrator', 'orchestrator_node'] + ros + ['max_repairs:=3'],
            ['ros2', 'run', 'bt_executor', 'bt_executor_node'] + ros + ['dry_run:=false', '-p', 'auto_run:=true', '-p', 'max_runtime_sec:=30.0'],
            [sys.executable, os.path.join(HERE, 'fake_recovery_ext.py')]]
    procs = [subprocess.Popen(c, start_new_session=True, stdout=subprocess.DEVNULL) for c in cmds]
    fails = []
    try:
        rclpy.init()
        n = rclpy.create_node('recovery_test')
        pub = {t: n.create_publisher(String, t, 10) for t in ('bt/task', 'bt/task_spec', 'fake/fail')}
        box = {'state': {}, 'calls': {}, 'spoken': [], 'repairs': 0, 'answer': None}

        def on_repair(_m):
            box['repairs'] += 1
            if box['answer']:                          # this test plays the LLM: the same spec again
                pub['bt/task_spec'].publish(String(data=json.dumps(box['answer'])))
        n.create_subscription(String, 'task/state', lambda m: box.update(state=json.loads(m.data)), LATCHED)
        n.create_subscription(String, 'bt/repair', on_repair, 10)
        n.create_subscription(String, 'fake/calls', lambda m: box.update(calls=json.loads(m.data)), 10)
        n.create_subscription(String, 'fake/spoken', lambda m: box['spoken'].append(m.data), 10)

        def spin_until(pred, sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)
                if pred():
                    return True
            return False

        def check(name, ok):
            print(('ok   ' if ok else 'FAIL ') + name)
            if not ok:
                fails.append(name)

        spin_until(lambda: pub['fake/fail'].get_subscription_count() and box['state'], 60)
        spin_until(lambda: False, 8)                   # manifests: validator and executor register them on the 3 s republish

        def scenario(spec, fail, answer=None):
            box.update(repairs=0, answer=answer, state={})      # forget the last scenario's 'done': the same goal text would match it at once
            box['spoken'].clear()
            pub['fake/fail'].publish(String(data=json.dumps(fail)))
            spin_until(lambda: False, 0.5)
            pub['bt/task'].publish(String(data=spec['goal']))
            spin_until(lambda: False, 0.3)
            pub['bt/task_spec'].publish(String(data=json.dumps(spec)))
            spin_until(lambda: box['state'].get('state') in ('done', 'failed') and box['state'].get('task') == spec['goal'], 90)
            return box['state']

        before = dict(box['calls'])
        s = scenario(GOTO, {'door': 2})                 # one run = 2 goals (go_to_place clears the costmaps and tries once more)
        check('A retry: done after 1 recovery', s.get('state') == 'done' and s.get('repairs') == 1)
        check('A the door: 2 failed goals, then 1 good one', box['calls'].get('door', 0) - before.get('door', 0) == 3)

        before = dict(box['calls'])
        s = scenario(GOTO, {'door': 99}, answer=GOTO)
        check('B retry, replan, go_home: done after 3 recoveries', s.get('state') == 'done' and s.get('repairs') == 3)
        check('B the LLM was asked to replan once', box['repairs'] == 1)
        check('B the door was tried in 3 runs (6 goals), then start was reached', box['calls'].get('door', 0) - before.get('door', 0) == 6
              and box['calls'].get('start', 0) - before.get('start', 0) >= 1)

        s = scenario(LOOK, {}, answer=LOOK)
        check('C retry, replan, ask_human: done after 3 recoveries', s.get('state') == 'done' and s.get('repairs') == 3)
        check('C the human was asked for help out loud', any('Please help' in t for t in box['spoken']))
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGTERM)
        rclpy.try_shutdown()
    print('PASS' if not fails else f'FAIL: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
