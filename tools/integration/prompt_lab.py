#!/usr/bin/env python3
"""Offline prompt/sampler lab against a running llama-server (no ROS graph): rebuilds the llm_bridge prompt from the installed
catalog + macros + knowledge and the constrained-decoding schema, then sweeps sampler settings over some commands.
With --body FILE it replays the prompt and schema of a body captured from the real bridge (the rebuilt prompt is only an approximation:
the bridge adds e.g. the known places to NavigateToPlace). Reports per setting: JSON ok, loops (a step repeated 3+ times in a row), median latency. Needs the workspace sourced.
  python3 tools/integration/prompt_lab.py [--url http://127.0.0.1:8081]
"""
import argparse
import json
import os
import statistics
import time
import urllib.request

from bt_common.catalog import discover_catalog, format_listing
from bt_common.macros import callable_macros, describe, load_macros, load_whitelist, spec_schema

# command -> macro that a good plan contains (objective score instead of reading plans by eye)
GOLD = {'go to the door and say hello': 'go_and_say', 'find a person in the left room and tell them hello': 'go_find_and_say',
        'bring me a cup': 'say', 'follow me': 'follow_person', 'go home': 'return_home',
        'guide the guest to the right room': 'escort_person_to',
        'greet the new guest and ask for their name and favorite drink': 'meet_guest', 'look at me and say hello': 'look_at_person',
        'patrol between the left room and the right room': 'patrol', 'if somebody waves, wave back and greet them': 'wave_back'}
TASKS = list(GOLD)
_UNUSED = ['go to the door and say hello', 'find a person in the left room and tell them hello', 'bring me a cup',
         'count the people in the right room', 'ask the person at the door for their name', 'go home',
         'greet the new guest and ask for their name and favorite drink', 'tell me what you see']
SETTINGS = [{}, {'repeat_penalty': 1.1}, {'repeat_penalty': 1.1, 'repeat_last_n': 8}, {'dry_multiplier': 0.8},
            {'dry_multiplier': 0.8, 'dry_allowed_length': 3}, {'temperature': 0.0}, {'temperature': 0.0, 'dry_multiplier': 0.8}]


def system_prompt():
    cat = discover_catalog()
    for ext in cat.extensions.values():       # deployment without an arm (extensions.launch.py enable_arm:=false)
        for n in ('MoveArmToPose', 'OpenGripper', 'CloseGripper'):
            ext.pop(n, None)
    lib = callable_macros(load_macros(), load_whitelist())
    src = open(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'llm_bridge', 'src', 'llm_bridge_node.cpp')).read()
    a = src.index('kDefaultSystem =') + len('kDefaultSystem =')
    base = ''.join(p.strip()[1:-1].encode().decode('unicode_escape') for p in src[a:src.index(';', a)].split('\n') if p.strip())
    listing = format_listing(cat.all().values(), with_desc=True) + (
        '\n\nMacros: a step {"macro": "<name>", "args": {...}} expands to several nodes - prefer a macro when one fits '
        '(shorter, faster):\n' + describe(lib))
    know = open(os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'framework_bringup', 'param', 'llm_knowledge.md')).read()
    return base + 'Nodes (id[category](ports, ? = optional) - description):\n' + listing + '\nRobot knowledge:\n' + know + '\n', \
        spec_schema(list(cat.all().values()), lib)


def loops(steps):
    s = [json.dumps(x, sort_keys=True) for x in steps]
    return any(s[i] == s[i + 1] == s[i + 2] for i in range(len(s) - 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--body', help='a request body captured from the live llm_bridge (use its exact prompt + schema)')
    ap.add_argument('--url', default='http://127.0.0.1:8081')
    a = ap.parse_args()
    if a.body:
        live = json.load(open(a.body))
        sysmsg, schema = live['messages'][0]['content'], live['response_format']['json_schema']['schema']
    else:
        sysmsg, schema = system_prompt()
    for extra in SETTINGS:
        ok = bad = hit = 0
        lat = []
        for t in TASKS:
            body = {'messages': [{'role': 'system', 'content': sysmsg}, {'role': 'user', 'content': f'Task: {t}\nScene: {{}}'}],
                    'max_tokens': 768, 'temperature': 0.1, 'response_format': {'type': 'json_schema', 'json_schema': {'name': 'task_spec', 'schema': schema}}}
            body.update(extra)
            t0 = time.monotonic()
            r = json.load(urllib.request.urlopen(urllib.request.Request(a.url + '/v1/chat/completions', json.dumps(body).encode(), {'Content-Type': 'application/json'}), timeout=180))
            lat.append(time.monotonic() - t0)
            try:
                steps = json.loads(r['choices'][0]['message']['content'])['steps']
                ok += 1
                bad += loops(steps)
                hit += GOLD[t] in json.dumps(steps) and len(steps) <= 6
            except (ValueError, KeyError):
                pass
        print(f'{str(extra):52} right_macro {hit}/{len(TASKS)}  json_ok {ok}  looping {bad}  median {statistics.median(lat):4.1f}s')


if __name__ == '__main__':
    main()
