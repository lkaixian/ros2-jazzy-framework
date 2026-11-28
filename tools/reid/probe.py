#!/usr/bin/env python3
"""See how the appearance vector of the largest person changes as they move: needs person_detection_node running with reid_model.

  source env.sh; timeout 90 python3 tools/reid/probe.py [--topic persons]

The first 4 s become the reference look. Then, twice a second: similarity to the reference (1.00 = identical), box size, distance, colour.
Walk through the cases that fail (closer, farther, side, back, camera moved) and read what the similarity does.
"""
import argparse
import time

import numpy as np
import rclpy
from person_detection_msgr.msg import PersonArray

ap = argparse.ArgumentParser()
ap.add_argument('--topic', default='persons')
a = ap.parse_args()
rclpy.init()
node = rclpy.create_node('reid_probe')
ref, t0, last = [], time.monotonic(), 0.0


def on(m):
    global ref, last
    ps = [p for p in m.persons if len(p.embedding)]
    if not ps:
        return
    p = max(ps, key=lambda q: (q.x_max - q.x_min) * (q.y_max - q.y_min))
    e = np.asarray(p.embedding)
    now = time.monotonic()
    if now - t0 < 4.0:
        ref.append(e)
        return
    if len(ref) and not isinstance(ref, np.ndarray):
        ref = np.mean(ref, axis=0)
        ref /= np.linalg.norm(ref)
        print(f'reference set from the first 4 s; now move ({time.strftime("%H:%M:%S")})')
    if now - last >= 0.5:
        last = now
        print(f'sim {float(ref @ e):5.2f}  box {p.x_max - p.x_min:4.0f}x{p.y_max - p.y_min:4.0f}  '
              f'dist {p.distance_m:4.1f} m  {p.clothes_colour or "-":8s} id {p.track_id}')


node.create_subscription(PersonArray, a.topic, on, 10)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
