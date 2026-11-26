#!/usr/bin/env python3
"""Read-only: segment the bottle the arm eye sees and report what a mask would tell the pick (tilt, width, how much of it has depth).

Moves nothing. Needs tools/arm/arm_eye.sh running and models/seg/yolo26s-seg.pt.
    python3 tools/arm/seg_probe.py [label] [out.png]
The overlay (mask outline, long axis, detector box) is saved so you can see whether the mask follows the object.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from object_detection_msgr.msg import ObjectArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import Image
from ultralytics import YOLO
from yolo11_common.base_node import image_to_bgr

HERE = os.path.dirname(os.path.realpath(__file__))
label = sys.argv[1] if len(sys.argv) > 1 else 'bottle'
out = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/seg_probe.png')
rclpy.init()
n, S = Node('seg_probe'), {}
n.create_subscription(Image, '/arm_cam/rgb/image_raw', lambda m: S.__setitem__('rgb', m), Q)
n.create_subscription(Image, '/arm_cam/stereo/image_raw', lambda m: S.__setitem__('d', m), Q)
n.create_subscription(ObjectArray, '/arm_eye/objects', lambda m: S.__setitem__('o', m), Q)
t = time.time()
while time.time() - t < 8 and len(S) < 3:
    rclpy.spin_once(n, timeout_sec=0.2)
if len(S) < 3:
    sys.exit('missing a topic (is tools/arm/arm_eye.sh running?): got %s' % sorted(S))
det = [o for o in S['o'].objects if o.label == label]
if not det:
    sys.exit(f'no {label} in view of the detector')
b = max(det, key=lambda o: o.confidence)
frame = cv2.rotate(image_to_bgr(S['rgb']), cv2.ROTATE_180)               # upright, the frame the detector boxes are in
model = YOLO(os.path.join(HERE, '..', '..', 'models', 'seg', 'yolo26s-seg.pt'))
cls = [i for i, nme in model.names.items() if nme == label]
res = model(frame, classes=cls, verbose=False)[0]
if res.masks is None or not len(res.masks.xy):
    sys.exit('the seg model found no %s (detector box %s)' % (label, [round(float(v)) for v in (b.x_min, b.y_min, b.x_max, b.y_max)]))


def iou(a, c):
    iw, ih = max(0, min(a[2], c[2]) - max(a[0], c[0])), max(0, min(a[3], c[3]) - max(a[1], c[1]))
    return iw * ih / ((a[2] - a[0]) * (a[3] - a[1]) + (c[2] - c[0]) * (c[3] - c[1]) - iw * ih)


box = (b.x_min, b.y_min, b.x_max, b.y_max)
best = max(range(len(res.masks.xy)), key=lambda i: iou(box, res.boxes.xyxy[i].tolist()))
poly = res.masks.xy[best].astype(np.float32)
mask = np.zeros(frame.shape[:2], np.uint8)
cv2.fillPoly(mask, [poly.astype(np.int32)], 1)
ys, xs = np.nonzero(mask)
pts = np.stack([xs, ys], 1).astype(float)
c = pts.mean(0)
w, v = np.linalg.eigh(np.cov((pts - c).T))
ax = v[:, 1]                                                              # long axis
tilt = abs(np.degrees(np.arctan2(ax[0], ax[1])))                          # 0 = vertical in the upright image
length, width = 4 * np.sqrt(w[1]), 4 * np.sqrt(w[0])                      # ~ full extent of a filled rectangle
d = S['d']
raw = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width)
rawmask = cv2.resize(cv2.rotate(mask, cv2.ROTATE_180), (d.width, d.height), interpolation=cv2.INTER_NEAREST).astype(bool)
dv = raw[rawmask]
ok = dv[(dv > 0) & (dv < 20000)]
print(f'detector box {[round(float(x)) for x in box]}  seg box {[round(float(x)) for x in res.boxes.xyxy[best]]}  seg conf {float(res.boxes.conf[best]):.2f}  IoU {iou(box, res.boxes.xyxy[best].tolist()):.2f}')
print(f'mask {int(mask.sum())} px = {100 * mask.sum() / ((box[2] - box[0]) * (box[3] - box[1])):.0f}% of the detector box;  long axis {length:.0f} px, width {width:.0f} px, tilt from vertical {tilt:.0f} deg')
print('orientation:', 'UPRIGHT' if tilt < 25 else 'LYING' if tilt > 65 else 'SLANTED', f'(tilt {tilt:.0f} deg)')
print(f'depth on the mask: {ok.size}/{dv.size} valid ({100 * ok.size / max(1, dv.size):.0f}%)',
      ('percentiles 5/25/50/75 mm: ' + str(np.percentile(ok, [5, 25, 50, 75]).astype(int).tolist())) if ok.size else '')
over = frame.copy()
cv2.polylines(over, [poly.astype(np.int32)], True, (0, 255, 0), 3)
cv2.rectangle(over, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
p1, p2 = (c - ax * length / 2).astype(int), (c + ax * length / 2).astype(int)
cv2.line(over, tuple(map(int, p1)), tuple(map(int, p2)), (0, 0, 255), 3)
cv2.imwrite(out, over)
print('overlay saved to', out, '(green = mask outline, blue = detector box, red = long axis)')
rclpy.shutdown()
