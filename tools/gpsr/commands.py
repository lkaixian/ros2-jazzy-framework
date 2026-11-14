#!/usr/bin/env python3
"""GPSR-style command generator + plan checker (no ROS, no LLM needed).

Each item = a spoken-style command, its category, what a correct plan must contain (`expect`) and one correct plan (`gold`, a task spec
that uses the whitelisted macros). Uses: (1) an evaluation set for the LLM (`tools/integration/llm_eval.py --jsonl`), (2) training data for
a fine-tune (command -> gold), checked against the real validator by a test.

  python3 tools/gpsr/commands.py --n 200 --seed 1 --out /tmp/gpsr_eval.jsonl
Templates are ours (inspired by the GPSR style: navigation, people, vision, and manipulation that this robot must refuse because it has no
working arm); they are not the official generator's sentences.

expect keys (all given keys must hold): visit [places in order] | says | macro name | node id | detects label |
any_node [ids] | laps N | refuse (says something and drives nowhere)
"""
import argparse
import json
import os
import random

PARAM = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src', 'framework_bringup', 'param')
LABELS = ['cup', 'bottle', 'bowl', 'banana', 'apple', 'orange', 'sandwich', 'chair', 'laptop', 'book', 'clock', 'vase', 'backpack',
          'umbrella', 'cell phone', 'remote', 'scissors', 'teddy bear']
TEXTS = ['hello', 'good morning', 'welcome', 'I am here', 'nice to meet you', 'have a nice day']
PREFIX = ['', '', '', 'please ', 'could you ', 'can you ', 'robot, ']
SUFFIX = ['', '', '', ' please', ' now']


def places():
    import yaml
    return sorted(yaml.safe_load(open(os.path.join(PARAM, 'places.yaml')))['places'])


def say(p):
    return p.replace('_', ' ')


def call(macro, **args):
    return {'macro': macro, 'args': {k: str(v) for k, v in args.items()}} if args else {'macro': macro}


def templates(rng, P):
    """-> list of (category, [phrasings], builder) where builder() -> (slots dict, gold steps, expect)."""
    pl = lambda: rng.choice(P)  # noqa: E731
    two = lambda: rng.sample(P, 2)  # noqa: E731
    lab = lambda: rng.choice(LABELS)  # noqa: E731
    txt = lambda: rng.choice(TEXTS)  # noqa: E731
    T = []

    def add(cat, phr, fn):
        T.append((cat, phr, fn))
    add('navigation', ['go to the {p}', 'navigate to the {p}', 'move to the {p}', 'head to the {p}'],
        lambda: (lambda p: ({'p': say(p)}, [call('go_to_place', place=p)], {'visit': [p]}))(pl()))
    add('navigation', ['go to the {p} and say {t}', 'drive to the {p} and tell them {t}', 'say {t} at the {p}'],
        lambda: (lambda p, t: ({'p': say(p), 't': t}, [call('go_and_say', place=p, text=t)], {'visit': [p], 'says': True}))(pl(), txt()))
    add('navigation', ['go home', 'return to the start', 'come back to the start', 'go back home'],
        lambda: ({}, [call('return_home')], {'visit': ['start']}))
    add('navigation', ['patrol between the {a} and the {b}', 'go back and forth between the {a} and the {b}'],
        lambda: (lambda ab: ({'a': say(ab[0]), 'b': say(ab[1])}, [call('patrol', first=ab[0], second=ab[1])],
                             {'visit': list(ab)}))(two()))
    add('navigation', ['patrol between the {a} and the {b} {n} times', 'do {n} laps between the {a} and the {b}'],
        lambda: (lambda ab, n: ({'a': say(ab[0]), 'b': say(ab[1]), 'n': n},
                                [call('patrol_laps', first=ab[0], second=ab[1], laps=n)], {'visit': list(ab), 'laps': n}))(
            two(), rng.randint(2, 5)))
    add('navigation', ['visit the {a}, then the {b}, then the {c}', 'go to the {a}, the {b} and the {c} in that order'],
        lambda: (lambda abc: ({'a': say(abc[0]), 'b': say(abc[1]), 'c': say(abc[2])},
                              [{'foreach': list(abc), 'as': 'p', 'steps': [call('go_to_place', place='${p}')]}],
                              {'visit': list(abc)}))(rng.sample(P, 3) if len(P) >= 3 else P))
    add('navigation', ['walk around for a while', 'wander around the room', 'walk around', 'just walk around a bit'],
        lambda: ({}, [call('wander')], {'macro': 'wander'}))
    add('people', ['find a person in the {p} and say {t}', 'go to the {p}, find someone and tell them {t}'],
        lambda: (lambda p, t: ({'p': say(p), 't': t}, [call('go_find_and_say', place=p, text=t)],
                               {'visit': [p], 'says': True, 'node': 'DetectPerson'}))(pl(), txt()))
    add('people', ['follow me', 'come with me', 'follow me around'], lambda: ({}, [call('follow_person')], {'macro': 'follow_person'}))
    add('people', ['guide the guest to the {p}', 'take the guest to the {p}', 'show the guest the way to the {p}'],
        lambda: (lambda p: ({'p': say(p)}, [call('escort_person_to', place=p)], {'visit': [p], 'says': True}))(pl()))
    add('people', ['look at me', 'face me', 'turn to me'], lambda: ({}, [call('look_at_person')], {'macro': 'look_at_person'}))
    add('people', ['wait until somebody waves', 'wait for someone to wave'],
        lambda: ({}, [call('wait_for_wave')], {'node': 'IsPersonWaving'}))
    add('people', ['greet the new guest and ask for their name and favorite drink', 'ask the guest for their name and their drink'],
        lambda: ({}, [call('meet_guest')], {'macro': 'meet_guest'}))
    add('people', ['wait for the doorbell', 'tell me when someone rings the bell'],
        lambda: ({}, [call('wait_for_doorbell')], {'node': 'DetectSound'}))
    add('people', ['welcome the guest at the door', 'receive the next guest'],
        lambda: ({}, [call('receive_guest')], {'macro': 'receive_guest'}))
    add('people', ['find a free seat for the guest', 'show the guest a free chair'],
        lambda: ({}, [call('offer_seat')], {'node': 'FindFreeSeat'}))
    add('people', ['look around the room and tell me how many people are there', 'count the people around you'],
        lambda: ({}, [call('scout_room')], {'node': 'CountPeople'}))
    add('vision', ['find a {l}', 'look for a {l}', 'search for a {l}'],
        lambda: (lambda lb: ({'l': lb}, [call('find_object', label=lb)], {'detects': lb}))(lab()))
    add('vision', ['go to the {p} and look for a {l}', 'check the {p} for a {l}'],
        lambda: (lambda p, lb: ({'p': say(p), 'l': lb}, [call('go_find_object', place=p, label=lb)],
                                {'visit': [p], 'detects': lb}))(pl(), lab()))
    add('vision', ['tell me what you see', 'describe what is around you'],
        lambda: ({}, [call('look_around'), call('report_object', label='person')],
                 {'says': True, 'any_node': ['DetectObject', 'DetectPerson', 'CountPeople', 'CountObjects']}))
    refuse = [call('say', text='Sorry, I have no arm and cannot do that')]
    add('refuse', ['bring me a {l}', 'fetch a {l}', 'get me a {l}'],
        lambda: (lambda lb: ({'l': lb}, refuse, {'refuse': True}))(lab()))
    add('refuse', ['pick up the {l}', 'grab the {l} from the {p}', 'put the {l} on the table', 'take the {l} to the {p}'],
        lambda: (lambda lb, p: ({'l': lb, 'p': say(p)}, refuse, {'refuse': True}))(lab(), pl()))
    add('refuse', ['open the door', 'open the cupboard', 'pour me a drink'], lambda: ({}, refuse, {'refuse': True}))
    return T


def generate(n, seed=1, P=None):
    """n items, categories round-robin over the templates so every template is used about equally."""
    rng = random.Random(seed)
    P = P or places()
    T = templates(rng, P)
    out = []
    for i in range(n):
        cat, phr, fn = T[i % len(T)]
        slots, gold, expect = fn()
        cmd = (rng.choice(PREFIX) + rng.choice(phr).format(**slots) + rng.choice(SUFFIX)).strip()
        out.append({'id': i, 'category': cat, 'command': cmd, 'expect': expect, 'gold': {'goal': cmd[:40], 'steps': gold}})
    return out


# ---------------------------------------------------------------- checking a plan against `expect`
def _walk(steps):
    for s in steps or []:
        yield s
        yield from _walk(s.get('steps'))


def check(spec, expect, expand):
    """True if `spec` (the plan as the LLM wrote it) meets `expect`. expand(spec) -> spec with macros/foreach unrolled (may raise)."""
    try:
        flat = expand(spec)
    except Exception:  # noqa: BLE001 - an unexpandable plan is a wrong plan
        return False
    leaves = [s for s in _walk(flat.get('steps')) if 'node' in s]
    calls = [s for s in _walk(spec.get('steps')) if 'macro' in s]
    nodes = {s['node'] for s in leaves}
    visits = [s.get('ports', {}).get('place') for s in leaves if s['node'] == 'NavigateToPlace']
    says = 'Speak' in nodes
    ok = True
    if 'visit' in expect:
        it = iter(visits)
        ok &= all(any(p == v for v in it) for p in expect['visit'])      # ordered subsequence
    if expect.get('says'):
        ok &= says
    if 'macro' in expect:
        ok &= any(c['macro'] == expect['macro'] for c in calls)
    if 'node' in expect:
        ok &= expect['node'] in nodes
    if 'detects' in expect:
        ok &= any(s['node'] == 'DetectObject' and s.get('ports', {}).get('label') == expect['detects'] for s in leaves)
    if 'any_node' in expect:
        ok &= bool(nodes & set(expect['any_node']))
    if 'laps' in expect:
        n = str(expect['laps'])
        ok &= (any(c['macro'] == 'patrol_laps' and c.get('args', {}).get('laps') == n for c in calls)
               or any(str(s.get('repeat')) == n for s in _walk(flat.get('steps')) if 'group' in s))
    if expect.get('refuse'):
        ok &= says and not visits
    return bool(ok)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--n', type=int, default=100)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', default='-')
    a = ap.parse_args()
    f = open(a.out, 'w') if a.out != '-' else None
    for item in generate(a.n, a.seed):
        print(json.dumps(item), file=f) if f else print(json.dumps(item))
    if f:
        f.close()
        print(f'wrote {a.n} commands to {a.out}')


if __name__ == '__main__':
    main()
