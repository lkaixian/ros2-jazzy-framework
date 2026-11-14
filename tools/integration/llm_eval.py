#!/usr/bin/env python3
"""Plan quality against a REAL LLM: send spoken-style commands to a running pipeline (llm_bridge -> orchestrator ->
validator) and report, per command, valid/invalid, attempts, latency and the steps. Needs the workspace sourced and a
stack started with `llama_url:=...`, e.g. (own DDS domain):
  ROS_DOMAIN_ID=82 ros2 launch framework_bringup extensions.launch.py enable_arm:=false enable_pose:=false &
  ROS_DOMAIN_ID=82 ros2 launch framework_bringup pipeline.launch.py llama_url:=http://127.0.0.1:8081 &
  ROS_DOMAIN_ID=82 python3 tools/integration/llm_eval.py [--commands FILE] [--timeout 120]
FILE = one command per line. Exit 0 if at least 70 % of the plans are valid (a measurement, not a guarantee).
"""
import argparse
import json
import sys
import time

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from std_msgs.msg import String

COMMANDS = [
    'go to the door and say hello',
    'find a person in the left room and tell them hello',
    'bring me a cup',
    'follow me',
    'count the people in the right room',
    'ask the person at the door for their name',
    'patrol between the left room and the right room',
    'if somebody waves, wave back and greet them',
    'tell me what you see',
    'go home',
    'greet the new guest and ask for their name and favorite drink',
    'guide the guest to the right room',
    'find a bottle and tell me where it is',
    'tell me how many people are here',
    'look at me and say hello',
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commands')
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    cmds = [l.strip() for l in open(a.commands)] if a.commands else COMMANDS
    rclpy.init()
    n = rclpy.create_node('llm_eval')
    plans = []
    n.create_subscription(BtPlan, 'bt/plan', plans.append, 10)
    pub = n.create_publisher(String, 'bt/task', 10)
    end = time.monotonic() + 60
    while time.monotonic() < end and pub.get_subscription_count() < 2:
        rclpy.spin_once(n, timeout_sec=0.2)
    ok = 0
    for c in filter(None, cmds):
        plans.clear()
        t0 = time.monotonic()
        pub.publish(String(data=c))
        while time.monotonic() - t0 < a.timeout and not plans:
            rclpy.spin_once(n, timeout_sec=0.1)
        dt = time.monotonic() - t0
        if not plans:
            print(f'TIMEOUT {dt:5.1f}s  {c}')
            continue
        p = plans[-1]
        ok += bool(p.valid)
        steps = json.loads(p.task_spec).get('steps') if p.task_spec.startswith('{') else p.task_spec[:80]
        print(f"{'VALID  ' if p.valid else 'INVALID'} {dt:5.1f}s tries={p.attempts}  {c}\n         {json.dumps(steps)[:300]}"
              + ('' if p.valid else f"\n         errors: {list(p.errors)[:2]}"))
    print(f'{ok}/{len([c for c in cmds if c])} valid')
    rclpy.try_shutdown()
    return 0 if ok >= 0.7 * len([c for c in cmds if c]) else 1


if __name__ == '__main__':
    sys.exit(main())
