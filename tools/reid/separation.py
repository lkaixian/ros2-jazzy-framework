#!/usr/bin/env python3
"""How well does the appearance vector tell two people apart? Needs person_detection_node running with reid_model.

  source env.sh; timeout 40 python3 tools/reid/separation.py [--seconds 20] [--topic persons]

Two people sit still in different seats, side by side in the picture, for the whole run (no blocker). Detections are split into
a left and a right person by their x position; it prints the cosine similarity of each person to themselves (their own looks, a
second apart) and to the other person. A gap between "same person" and "other person" is what a threshold can use.
"""
import argparse
import time

import numpy as np
import rclpy
from person_detection_msgr.msg import PersonArray

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=20.0)
ap.add_argument('--topic', default='persons')
a = ap.parse_args()
rclpy.init()
node = rclpy.create_node('reid_separation')
rows = []   # (time, x centre, vector)


def on(m):
    if len(m.persons) == 2 and all(len(p.embedding) for p in m.persons):
        for p in m.persons:
            rows.append((time.monotonic(), (p.x_min + p.x_max) / 2, np.asarray(p.embedding)))


node.create_subscription(PersonArray, a.topic, on, 10)
end = time.monotonic() + a.seconds
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
if len(rows) < 20:
    raise SystemExit(f'only {len(rows)} looks with exactly two people in view: seat both and keep them in the picture')
cut = np.median([r[1] for r in rows])
side = [[r for r in rows if r[1] < cut], [r for r in rows if r[1] >= cut]]
E = [np.stack([r[2] for r in s]) for s in side]
rng = np.random.default_rng(0)


def pairs(x, y, same):
    idx = rng.integers(0, [len(x), len(y)], (400, 2))
    s = np.array([float(x[i] @ y[j]) for i, j in idx if not (same and i == j)])
    return s.min(), np.percentile(s, 5), s.mean(), np.percentile(s, 95), s.max()


for name, (x, y, same) in {'left vs itself': (E[0], E[0], True), 'right vs itself': (E[1], E[1], True),
                           'left vs RIGHT person': (E[0], E[1], False)}.items():
    lo, p5, mean, p95, hi = pairs(x, y, same)
    print(f'{name:22s} mean {mean:.2f}  5th percentile {p5:.2f}  95th {p95:.2f}  lowest {lo:.2f}  highest {hi:.2f}')
print(f'{len(side[0])} + {len(side[1])} looks. A usable threshold sits above the "RIGHT person" 95th percentile and below the "itself" 5th percentile.')
