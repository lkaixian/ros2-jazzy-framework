#!/usr/bin/env python3
"""Sentences in the shape of the official GPSR/EGPSR command generator (github.com/johaq/CommandGenerator @ b2c431b), with the word lists
of the RoboCup@Home 2025 arena (github.com/robocupmalaysia/at-home-2025: maps/room_names.md, location_names.md, objects/objects.md,
names/names.md). Only for the GPSR and EGPSR tasks. Gestures / poses / colours / talk topics are the generator's hard-coded kinds
(exact wording is ours). Every command carries an `expect` for `llm_eval.py --jsonl`, so the plan is scored on what it asked.

  python3 tools/gpsr/official.py --n 100 --seed 1 --out /tmp/gpsr_official.jsonl
Needs the detector labels in the validator (bt_validator_node.labels = OBJECTS below) and the arena places in places.yaml.
"""
import argparse
import json
import random

ROOMS = ['bedroom', 'kitchen', 'office', 'living room']
PLACEMENT = {'shelves': 'food', 'dining table': 'tableware', 'coffee table': 'drinks', 'office cabinet': 'snacks'}
OTHER = ['bed', 'dishwasher', 'office desk', 'entrance', 'exit']
OBJECTS = {'drinks': ['cola', 'coffee', 'water', 'milk'], 'food': ['cereal', 'tom-yam cup noodle', 'yen-ta-four cup noodle'],
           'snacks': ['biscuit roll', 'paprika', 'potae', 'cornae', 'pringles', 'lay stax', 'pocky'],
           'tableware': ['spoon', 'plate', 'cup', 'mug', 'fork', 'bowl']}
NAMES = ['Liam', 'Noah', 'Oliver', 'Laura', 'James', 'William', 'Benjamin', 'Lucas', 'Henry', 'Alexander', 'Emma', 'Olivia', 'Ava', 'Sophia',
         'Isabella']
GESTURES = ['waving', 'raising their left arm', 'raising their right arm', 'pointing to the left', 'pointing to the right']
POSES = ['sitting', 'standing', 'lying down']
COLOURS = ['blue', 'yellow', 'black', 'white', 'red', 'orange', 'gray']
CLOTHES = ['t shirt', 'shirt', 'blouse', 'sweater', 'coat', 'jacket']
TALK = ['something about yourself', 'the time', 'what day is today', 'the day of the month', 'your teams name', 'a joke']
COMPARE = ['biggest', 'largest', 'smallest', 'heaviest', 'lightest', 'thinnest']
INFO = ['name', 'pose', 'gesture']
LABELS = [o for v in OBJECTS.values() for o in v]
pid = lambda s: s.replace(' ', '_')          # noqa: E731 - "dining table" -> the place id in places.yaml
art = lambda w: ('an ' if w[0] in 'aeiou' else 'a ') + w          # noqa: E731


def generate(n, seed=1):
    r = random.Random(seed)
    room = lambda: r.choice(ROOMS)          # noqa: E731
    plc = lambda: r.choice(list(PLACEMENT))          # noqa: E731
    anyloc = lambda: r.choice(ROOMS + list(PLACEMENT) + OTHER)          # noqa: E731
    obj_at = lambda p: r.choice(OBJECTS[PLACEMENT[p]])          # noqa: E731
    name = lambda: r.choice(NAMES)          # noqa: E731
    cloth = lambda: f'{r.choice(COLOURS)} {r.choice(CLOTHES)}'          # noqa: E731
    T = []           # (category, lambda -> (command, expect))

    def add(cat, fn):
        T.append((cat, fn))

    def two(pool):
        a_, b_ = r.sample(pool, 2)          # a start and a different destination
        return a_, b_

    # --- people commands
    add('gpsr', lambda: (lambda x: (f'go to the {x}', {'visit': [pid(x)]}))(anyloc()))
    add('gpsr', lambda: (lambda x: (f'find a person in the {x} and {r.choice(["say " + r.choice(TALK), "follow them"])}',
                                    {'visit': [pid(x)]}))(room()))
    add('gpsr', lambda: (lambda x, g: (f'find a person in the {x} and guide them to the {g}', {'visit': [pid(x), pid(g)]}))(room(), anyloc()))
    add('gpsr', lambda: (lambda x, nm: (f'meet {nm} at the {x} and follow them', {'visit': [pid(x)], 'macro': 'meet_and_remember'}))(anyloc(), name()))
    add('gpsr', lambda: (lambda x, c: (f'tell me how many people in the {x} are wearing a {c}', {'visit': [pid(x)], 'node': 'CountPeople'}))(room(), cloth()))
    add('gpsr', lambda: (lambda x: (f'tell me how many people in the {x} are {r.choice(POSES)}', {'visit': [pid(x)], 'node': 'CountPeople'}))(room()))
    add('gpsr', lambda: (lambda x: (f'tell me the {r.choice(INFO)} of the person at the {x}', {'visit': [pid(x)]}))(anyloc()))
    add('gpsr', lambda: (lambda a, b, nm: (f'follow {nm} from the {a} to the {b}', {'visit': [pid(a)]}))(*two(ROOMS + list(PLACEMENT) + OTHER), name()))
    add('gpsr', lambda: (lambda a, b, nm: (f'guide {nm} from the {a} to the {b}', {'visit': [pid(a), pid(b)]}))(*two(ROOMS + list(PLACEMENT) + OTHER), name()))
    add('gpsr', lambda: (lambda x, nm: (f'greet {nm} in the {x}', {'visit': [pid(x)], 'says': True}))(room(), name()))
    add('gpsr', lambda: (lambda x: (f'greet the person wearing a {cloth()} in the {x}', {'visit': [pid(x)], 'says': True}))(room()))
    # --- object commands
    add('gpsr', lambda: (lambda p: (lambda o: (f'take {art(o)} from the {p} and {r.choice(["give it to me", "bring it to me"])}',
                                               {'macro': 'move_object'}))(obj_at(p)))(plc()))
    add('gpsr', lambda: (lambda p, q: (f'take {art(obj_at(p))} from the {p} and place it on the {q}',
                                       {'macro': 'move_object', 'visit': [pid(p), pid(q)]}))(*two(list(PLACEMENT))))
    add('gpsr', lambda: (lambda x, o: (f'find {art(o)} in the {x}', {'visit': [pid(x)], 'detects': o}))(room(), r.choice(LABELS)))
    add('gpsr', lambda: (lambda p: (lambda o: (f'tell me how many {o} are there on the {p}', {'visit': [pid(p)], 'node': 'CountObjects'}))(obj_at(p)))(plc()))
    add('gpsr', lambda: (lambda p: (f'tell me what is the {r.choice(COMPARE)} object on the {p}', {'visit': [pid(p)]}))(plc()))
    add('gpsr', lambda: (lambda p: (lambda o: (f'bring me {art(o)} from the {p}', {'macro': 'move_object'}))(obj_at(p)))(plc()))
    # --- EGPSR
    add('egpsr', lambda: (lambda p, q: (f'the {r.choice(LABELS)} is at the {p} instead of the {q}; put it back',
                                       {'macro': 'move_object', 'visit': [pid(p), pid(q)]}))(*two(list(PLACEMENT))))
    out = []
    for i in range(n):
        cat, fn = T[i % len(T)] if i < len(T) else r.choice(T)
        cmd, expect = fn()
        out.append({'id': i, 'category': cat, 'command': cmd, 'expect': expect})
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=30)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out')
    ap.add_argument('--labels', action='store_true', help='print the object labels for bt_validator_node.labels and exit')
    a = ap.parse_args()
    if a.labels:
        print(','.join(LABELS))
        raise SystemExit
    rows = generate(a.n, a.seed)
    if a.out:
        open(a.out, 'w').write(''.join(json.dumps(x) + '\n' for x in rows))
    else:
        print('\n'.join(f"{x['category']:6} {x['command']}   {x['expect']}" for x in rows))
