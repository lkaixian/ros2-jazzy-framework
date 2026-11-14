#!/usr/bin/env python3
"""Plan quality against a REAL LLM: send spoken-style commands to a running pipeline (llm_bridge -> orchestrator ->
validator) and report, per command, valid/invalid, attempts, latency and the steps. Needs the workspace sourced and a
stack started with `llama_url:=...`, e.g. (own DDS domain):
  ROS_DOMAIN_ID=82 ros2 launch framework_bringup extensions.launch.py enable_arm:=false enable_pose:=false &
  ROS_DOMAIN_ID=82 ros2 launch framework_bringup pipeline.launch.py llama_url:=http://127.0.0.1:8081 &
  ROS_DOMAIN_ID=82 python3 tools/integration/llm_eval.py [--commands FILE] [--timeout 120]
FILE = one command per line. Commands with a known right macro (GOLD below) are also scored on choosing it. `--gen N [--seed S]` or
`--jsonl FILE` (tools/gpsr/commands.py) score each plan on what the command asked, per category. Exit 0 if at least 70 % of the plans are
valid and right (a measurement, not a guarantee).
"""
import argparse
import json
import os
import sys
import time

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from std_msgs.msg import String

# command -> macro a good plan contains (same list as prompt_lab.py)
GOLD = {'go to the door and say hello': 'go_and_say', 'find a person in the left room and tell them hello': 'go_find_and_say',
        'bring me a cup': 'say', 'follow me': 'follow_person', 'go home': 'return_home',
        'guide the guest to the right room': 'escort_person_to',
        'greet the new guest and ask for their name and favorite drink': 'meet_guest', 'look at me and say hello': 'look_at_person',
        'patrol between the left room and the right room': 'patrol', 'if somebody waves, wave back and greet them': 'wave_back'}

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


def load_items(a):
    """-> [{'command', 'category', 'expect'}]: generated (--gen), from a JSONL file (--jsonl, tools/gpsr/commands.py format), a plain list
    of commands (--commands) or the built-in list. Items with an `expect` are scored on what the plan contains."""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gpsr'))
    if a.gen:
        import commands
        return commands.generate(a.gen, a.seed)
    if a.jsonl:
        return [json.loads(line) for line in open(a.jsonl) if line.strip()]
    cmds = [line.strip() for line in open(a.commands)] if a.commands else COMMANDS
    return [{'command': c, 'category': 'list', 'expect': None} for c in cmds if c]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commands')
    ap.add_argument('--jsonl', help='commands with expectations (tools/gpsr/commands.py --out)')
    ap.add_argument('--gen', type=int, default=0, help='generate N GPSR-style commands with expectations')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()
    items = load_items(a)
    checker = None
    if any(i.get('expect') for i in items):
        import commands
        from bt_common.macros import expand, load_macros, load_whitelist
        lib, wl = load_macros(), load_whitelist()
        checker = lambda spec, ex: commands.check(spec, ex, lambda s: expand(s, lib, wl))  # noqa: E731
    rclpy.init()
    n = rclpy.create_node('llm_eval')
    plans = []
    n.create_subscription(BtPlan, 'bt/plan', plans.append, 10)
    pub = n.create_publisher(String, 'bt/task', 10)
    end = time.monotonic() + 60
    while time.monotonic() < end and pub.get_subscription_count() < 2:
        rclpy.spin_once(n, timeout_sec=0.2)
    quiet = time.monotonic()                       # a plan still in flight from an earlier run would shift every score by one
    while time.monotonic() - quiet < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
        if plans:
            plans.clear()
            quiet = time.monotonic()
    ok = hit = gold = 0
    per_cat = {}                                   # category -> [right, total]
    misses = []
    for it in items:
        c = it['command']
        plans.clear()
        t0 = time.monotonic()
        pub.publish(String(data=c))
        while time.monotonic() - t0 < a.timeout and not plans:
            rclpy.spin_once(n, timeout_sec=0.1)
        dt = time.monotonic() - t0
        if it.get('expect'):
            per_cat.setdefault(it['category'], [0, 0])[1] += 1
        if not plans:
            print(f'TIMEOUT {dt:5.1f}s  {c}')
            continue
        p = plans[-1]
        ok += bool(p.valid)
        if c in GOLD:
            gold += 1
            hit += GOLD[c] in p.task_spec
        right = None
        if it.get('expect') and p.task_spec.startswith('{'):
            right = bool(p.valid) and checker(json.loads(p.task_spec), it['expect'])     # a plan that does not validate cannot run
            per_cat[it['category']][0] += bool(right)
            if not right:
                misses.append((it['category'], c, p.task_spec[:200]))
        steps = json.loads(p.task_spec).get('steps') if p.task_spec.startswith('{') else p.task_spec[:80]
        print(f"{'VALID  ' if p.valid else 'INVALID'} {dt:5.1f}s tries={p.attempts}"
              f"{'' if right is None else ' RIGHT' if right else ' WRONG'}  {c}\n         {json.dumps(steps)[:300]}"
              + ('' if p.valid else f"\n         errors: {list(p.errors)[:2]}"))
    total = len(items)
    print(f'{ok}/{total} valid; right macro {hit}/{gold}')
    if per_cat:
        tot = sum(t for _, t in per_cat.values())
        right_all = sum(r for r, _ in per_cat.values())
        print(f'correct (meets what the command asked): {right_all}/{tot} = {right_all / max(tot, 1):.0%}')
        for cat, (r, t) in sorted(per_cat.items()):
            print(f'  {cat:<11} {r}/{t} = {r / max(t, 1):.0%}')
        for cat, c, spec in misses[:15]:
            print(f'  MISS [{cat}] {c}\n       {spec}')
    rclpy.try_shutdown()
    good = ok >= 0.7 * total and (not gold or hit >= 0.7 * gold)
    if per_cat:
        good = good and sum(r for r, _ in per_cat.values()) >= 0.7 * sum(t for _, t in per_cat.values())
    return 0 if good else 1


if __name__ == '__main__':
    sys.exit(main())
