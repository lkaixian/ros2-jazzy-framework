#!/usr/bin/env python3
"""Does the online target classifier (pid_extension.target.TargetModel) beat plain cosine matching on real people?

  source env.sh; timeout 60 python3 tools/reid/target_check.py [--seconds 25]

Needs person_detection_node running with reid_model, and exactly two people in the picture for the whole run, in separate seats
(the left one is the target). The first half of the looks is used to train the classifier the way FollowTarget does (target against the
other person, frame by frame), the second half to score it. Prints how well each method separates the two people, and again with the
target's looks corrupted by a random direction (a blocker in front).
"""
import argparse
import time

import numpy as np
import rclpy
from person_detection_msgr.msg import PersonArray

from pid_extension.target import TargetModel

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=25.0)
ap.add_argument('--topic', default='persons')
a = ap.parse_args()
rclpy.init()
node = rclpy.create_node('target_check')
frames = []          # one entry per message: (left vector, right vector)


def on(m):
    if len(m.persons) == 2 and all(len(p.embedding) for p in m.persons):
        l, r = sorted(m.persons, key=lambda p: p.x_min + p.x_max)
        frames.append((np.asarray(l.embedding), np.asarray(r.embedding)))


node.create_subscription(PersonArray, a.topic, on, 10)
end = time.monotonic() + a.seconds
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
if len(frames) < 60:
    raise SystemExit(f'only {len(frames)} frames with exactly two people: keep both in the picture')
half = len(frames) // 2
model = TargetModel()
for t, o in frames[:half]:
    model.update(t, [o])
if not model.ready:
    raise SystemExit('the classifier never became ready (too few frames it trusted)')
T = np.stack([f[0] for f in frames[half:]])
O = np.stack([f[1] for f in frames[half:]])
ref = np.mean([f[0] for f in frames[:half]], axis=0)
ref /= np.linalg.norm(ref)
rng = np.random.default_rng(0)


def report(name, tgt, oth, cos):
    s_t = np.array([model.score(v) for v in tgt])
    s_o = np.array([model.score(v) for v in oth])
    c_t, c_o = tgt @ ref, oth @ ref
    print(f'{name:28s} classifier: target {s_t.mean():.2f} (min {s_t.min():.2f})  other {s_o.mean():.2f} (max {s_o.max():.2f})'
          f'   | cosine: target {c_t.mean():.2f} (min {c_t.min():.2f})  other {c_o.mean():.2f} (max {c_o.max():.2f})')


report('held-out second half', T, O, None)
for keep in (0.8, 0.6):
    z = rng.standard_normal(T.shape)
    z /= np.linalg.norm(z, axis=1, keepdims=True)
    c = keep * T + (1 - keep) * z
    report(f'target {keep:.0%} signal + blocker', c / np.linalg.norm(c, axis=1, keepdims=True), O, None)
print(f'{half} training frames, {len(frames) - half} test frames. Good = the target\'s MIN is above the other person\'s MAX.')
