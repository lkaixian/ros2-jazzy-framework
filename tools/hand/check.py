#!/usr/bin/env python3
"""What does the hand model see? Live check of the five-finger stop sign on the robot's camera.

  source env.sh; python3 tools/hand/check.py [--topic /camera/camera/color/image_raw] [--show]

Hold your hand up at the distance the operator will be (1-2 m) and open it, make a fist, hold up 3 fingers. Each second it prints, for every hand it finds on the WHOLE frame
(palm score, which of thumb / index / middle / ring / little are extended); the last frame with the hand skeleton drawn is saved to /tmp/hand_check.jpg (--show also opens a
window). The robot itself only looks near the operator's raised wrist (hands.py five_fingers), which is stricter. Needs models/hand/*.onnx (docs/perception.md).
"""
import argparse
import time

import cv2
import rclpy
from pose_extension.hands import HandModel
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from yolo11_common.base_node import image_to_bgr

EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18),
         (18, 19), (19, 20), (0, 17)]
ap = argparse.ArgumentParser()
ap.add_argument('--topic', default='/camera/camera/color/image_raw')
ap.add_argument('--show', action='store_true')
a = ap.parse_args()
model = HandModel()
if not model.available:
    raise SystemExit('no hand models in models/hand/ (docs/perception.md)')
rclpy.init()
node = rclpy.create_node('hand_check')
last = {'img': None}
node.create_subscription(Image, a.topic, lambda m: last.__setitem__('img', image_to_bgr(m)), qos_profile_sensor_data)
t0 = 0.0
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
    if last['img'] is None or time.monotonic() - t0 < 1.0:
        continue
    t0, img = time.monotonic(), last['img'].copy()
    hands = model.read(img)
    for lm, ext, score in hands:
        for p, q in EDGES:
            cv2.line(img, tuple(int(v) for v in lm[p][:2]), tuple(int(v) for v in lm[q][:2]), (0, 255, 0), 2)
        cv2.putText(img, ''.join('1' if e else '0' for e in ext), (int(lm[0][0]), int(lm[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    print(f'{len(hands)} hand(s): ' + ' | '.join(f'palm {s:.2f} fingers {sum(e)}/5 {["thumb", "index", "middle", "ring", "little"] if all(e) else [n for n, x in zip(["thumb", "index", "middle", "ring", "little"], e) if x]}' for _, e, s in hands))
    cv2.imwrite('/tmp/hand_check.jpg', img)
    if a.show:
        cv2.imshow('hand check', img)
        cv2.waitKey(1)
