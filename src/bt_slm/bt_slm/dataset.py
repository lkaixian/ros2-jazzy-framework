"""Synthetic (task_spec -> BT XML) training data, labelled by the real validator.

Two kinds of rows, both in the exact prompt format used at inference:
  * generate: NODES + TASK_SPEC            -> valid XML
  * repair:   ... + PREVIOUS_XML + ERRORS  -> valid XML   (what the retry loop sends)
"""
import argparse
import json
import os
import random
import re

from bt_common.catalog import Catalog, discover_catalog, format_listing  # noqa: F401
from bt_validator.validate import validate_xml

from bt_slm.fallback import build_xml
from bt_slm.prompt import build_prompt, normalize_spec

LABELS = ['cup', 'bottle', 'person', 'chair', 'laptop', 'book', 'phone', 'backpack', 'apple']
NAMES = ['alice', 'bob', 'carol', 'dave']
WORDS = ['fetch', 'find', 'check', 'go', 'look', 'greet', 'inspect', 'follow', 'wait', 'turn']
EXTENSION_PACKAGES = ['arm_extension', 'pose_extension', 'odom_extension', 'pid_extension',
                      'social_extension']


def value_for(node, port, rng, labels=None):
    if port.startswith('out_'):
        return '{' + port[4:] + '}'
    if port in ('label',):
        return rng.choice(labels or LABELS)
    if port == 'text':
        return rng.choice(['Hello {person}', 'Please look at the camera', 'I can see you', 'Done'])
    if port == 'person':
        return rng.choice(NAMES)
    if port == 'wait':
        return rng.choice(['true', 'false'])
    if port in ('identity',):
        return rng.choice(NAMES)
    if port == 'direction':
        return rng.choice(['left', 'right'])
    if port == 'controller':
        return rng.choice(['heading', 'follow_turn', 'follow_range'])
    if port == 'approach':
        return rng.choice(['true', 'false'])
    if port == 'frame':
        return rng.choice(['odom', 'base_link'])
    if port in ('x', 'y', 'z'):
        return f'{rng.uniform(-1.0, 1.0):.2f}'
    if port in ('yaw', 'roll', 'pitch', 'radians'):
        return f'{rng.uniform(-3.1, 3.1):.2f}'
    if port in ('meters',):
        return f'{rng.uniform(-2.0, 2.0):.1f}'
    if port in ('msec', 'timeout_ms'):
        return str(rng.choice([500, 1000, 2000, 5000, 10000]))
    if port in ('min_conf', 'min_similarity', 'target_height'):
        return f'{rng.uniform(0.3, 0.9):.1f}'
    if port in ('speed', 'tolerance', 'width', 'force', 'kp', 'ki', 'kd'):
        return f'{rng.uniform(0.05, 1.0):.2f}'
    if port in ('track_id', 'num_attempts', 'success_count', 'failure_count'):
        return str(rng.randint(1, 5))
    return str(rng.randint(1, 5))


# profile -> (top-level steps, steps per group, group depth limit, chance of a group)
# `heavy` = long plans, `max` = close to the validator's 64-node limit (most samples are rejected, so it is slow)
PROFILES = {'normal': ((1, 5), (1, 3), 2, 0.2), 'heavy': ((5, 9), (2, 5), 3, 0.35), 'max': ((9, 14), (3, 6), 4, 0.4)}


def sample_steps(leaves, rng, depth=0, labels=None, profile='normal'):
    top, inner, max_depth, p_group = PROFILES[profile]
    steps = []
    for _ in range(rng.randint(*(top if depth == 0 else inner))):
        if depth < max_depth and rng.random() < p_group:
            g = {'group': rng.choice(['Sequence', 'Fallback', 'ReactiveSequence',
                                      'ReactiveFallback', 'Parallel']),
                 'steps': sample_steps(leaves, rng, depth + 1, labels, profile)}
            if rng.random() < 0.3:
                g['retry'] = rng.randint(2, 4)
            if rng.random() < 0.15:
                g['repeat'] = rng.randint(2, 4)
            if g['group'] == 'Parallel' and rng.random() < 0.5:
                g['success_count'] = rng.randint(1, 2)
            steps.append(g)
            continue
        n = rng.choice(leaves)
        ports = {p: value_for(n.id, p, rng, labels) for p in n.required}
        for p in n.optional:
            if rng.random() < 0.4:
                ports[p] = value_for(n.id, p, rng, labels)
        steps.append({'node': n.id, 'ports': ports})
    return steps


def sample_spec(leaves, rng, labels=None, profile='normal'):
    spec = {'goal': ' '.join(rng.sample(WORDS, rng.randint(1, 2)) + [rng.choice(labels or LABELS)]),
            'steps': sample_steps(leaves, rng, 0, labels, profile)}
    if rng.random() < 0.25:
        spec['retry'] = rng.randint(2, 5)
    return spec


def corrupt(xml, rng):
    """Break a valid tree in a way the validator reports. Returns None if it can't."""
    opts = []
    if re.search(r'<[A-Z]\w+ ', xml):
        opts.append('unknown')
    if re.search(r' (label|x|meters|radians|msec|controller|kp)="', xml):
        opts.append('drop_port')
    opts.append('bogus_port')
    kind = rng.choice(opts)
    leaves = list(re.finditer(r'<((?!root|BehaviorTree|Sequence|RetryUntilSuccessful)[A-Z]\w*)([^>]*)/>', xml))
    if not leaves:
        return None
    m = rng.choice(leaves)
    if kind == 'unknown':
        return xml[:m.start(1)] + rng.choice(['Teleport', 'Fly', 'Grab', 'MoveTo']) + xml[m.end(1):]
    if kind == 'bogus_port':
        return xml[:m.end(1)] + ' bogus="1"' + xml[m.end(1):]
    attrs = list(re.finditer(r' (label|x|meters|radians|msec|controller|kp)="[^"]*"', m.group(0)))
    if not attrs:
        return None
    a = rng.choice(attrs)
    s0 = m.start() + a.start()
    return xml[:s0] + xml[m.start() + a.end():]


def generate(cat, n, seed=0, repair_frac=0.25, subset_frac=0.3, labels=None, profile='normal'):
    """Yield dict(prompt, completion, kind)."""
    rng = random.Random(seed)
    specs = list(cat.all().values())
    leaves = [s for s in specs if s.category in ('Action', 'Condition')]
    produced = 0
    while produced < n:
        spec = sample_spec(leaves, rng, labels, profile)
        spec_json = normalize_spec(json.dumps(spec))
        xml = build_xml(spec_json)
        if validate_xml(xml, cat):          # labels must come from the validator
            continue
        used = set(re.findall(r'"node":"(\w+)"', spec_json))
        shown = specs
        if rng.random() < subset_frac:
            keep = used | {s.id for s in rng.sample(specs, rng.randint(2, len(specs)))}
            shown = [s for s in specs if s.id in keep]
        listing = format_listing(shown)
        if rng.random() < repair_frac:
            bad = corrupt(xml, rng)
            errors = validate_xml(bad, cat) if bad else []
            if not errors:
                continue
            prompt = build_prompt(spec_json, listing, bad, errors)
            kind = 'repair'
        else:
            prompt, kind = build_prompt(spec_json, listing), 'generate'
        produced += 1
        yield {'prompt': prompt, 'completion': xml, 'kind': kind}


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out-dir', default='bt_dataset')
    ap.add_argument('--n', type=int, default=20000)
    ap.add_argument('--val-frac', type=float, default=0.02)
    ap.add_argument('--repair-frac', type=float, default=0.25)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--profile', choices=sorted(PROFILES), default='normal', help='heavy / max = long plans, up to the 64-node limit')
    ap.add_argument('--labels', default='',
                    help='comma-separated object labels = your detector classes (or @file, one per line)')
    ap.add_argument('--extension-config', action='append', default=[],
                    help='extra extension manifest YAML (repeatable)')
    a = ap.parse_args(args)
    cat = discover_catalog(a.extension_config)
    if a.labels.startswith('@'):
        with open(a.labels[1:]) as f:
            labels = [ln.strip() for ln in f if ln.strip()]
    else:
        labels = [s.strip() for s in a.labels.split(',') if s.strip()]
    os.makedirs(a.out_dir, exist_ok=True)
    n_val = max(1, int(a.n * a.val_frac))
    counts = {'train': 0, 'val': 0}
    with open(os.path.join(a.out_dir, 'train.jsonl'), 'w') as tr, \
            open(os.path.join(a.out_dir, 'val.jsonl'), 'w') as va:
        for i, row in enumerate(generate(cat, a.n, a.seed, a.repair_frac, labels=labels, profile=a.profile)):
            split = 'val' if i < n_val else 'train'
            (va if split == 'val' else tr).write(json.dumps(row) + '\n')
            counts[split] += 1
    print(f'{len(cat.all())} nodes in catalog; wrote {counts} to {a.out_dir}')


if __name__ == '__main__':
    main()
