#!/usr/bin/env python3
"""Read-only: where do the OAK (on the gripper, via joint angles + hand-eye) and the RealSense (fixed beside the arm base) put the same bottle?
Hold one bottle at 0.45-0.6 m in front of both. Moves nothing. Usage: python3 tools/arm/compare_cameras.py [label] [x y z] [--fit N]
--fit N: average N looks (bottle held STILL) and print the mean, spread and the fixed_cam_xyz that would make the RealSense agree with the OAK
(the OAK is itself hand-measured: this shows the two agree, not which is right)."""
import sys
import time

import numpy as np
import rclpy
from arm_control.hand_eye import box_depth_m, fixed_cam_point, load, upright_pixel_to_arm
from arm_control.omx_fk import Arm
from object_detection_msgr.msg import ObjectArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import CameraInfo, Image, JointState

argv = sys.argv[1:]
fit = int(argv.pop(argv.index('--fit') + 1)) if '--fit' in argv else 1
if '--fit' in argv:
    argv.remove('--fit')
label = argv[0] if argv else 'bottle'
xyz = [float(v) for v in argv[1:4]] if len(argv) >= 4 else [0.0, -0.08, -0.02]
rclpy.init()
n, S = Node('compare_cameras'), {}
for key, typ, topic in (('oo', ObjectArray, '/arm_eye/objects'), ('od', Image, '/arm_cam/stereo/image_raw'), ('ro', ObjectArray, '/objects'),
                        ('rd', Image, '/camera/camera/aligned_depth_to_color/image_raw'), ('ri', CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'),
                        ('js', JointState, '/joint_states')):
    n.create_subscription(typ, topic, lambda m, k=key: S.__setitem__(k, m), Q if key != 'js' else 10)
t = time.time()
while time.time() - t < 8 and len(S) < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
print('got:', sorted(S), '(need oo od ro rd ri js)')


def best(msg):
    hits = [o for o in msg.objects if o.label == label]
    return max(hits, key=lambda o: o.confidence) if hits else None


def depth(m):
    return np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width)


diffs = []
for i in range(fit):
    if i:
        t = time.time()
        while time.time() - t < 0.5:
            rclpy.spin_once(n, timeout_sec=0.1)
    if len(S) < 6 or not best(S['oo']) or not best(S['ro']):
        print(f'no {label} in view of both cameras (or a topic missing): hold one at 0.45-0.6 m in front of both')
        continue
    q = {j: p for j, p in zip(S['js'].name, S['js'].position)}
    o, r = best(S['oo']), best(S['ro'])
    do = box_depth_m(depth(S['od']), (o.x_min, o.y_min, o.x_max, o.y_max), S['oo'].image_width, S['oo'].image_height, load()[2])
    dr = box_depth_m(depth(S['rd']), (r.x_min, r.y_min, r.x_max, r.y_max), S['ro'].image_width, S['ro'].image_height, 0)
    if not (do and dr):
        print('no valid depth on both (OAK %s, RealSense %s)' % (do, dr))
        continue
    k = S['ri'].k
    po = upright_pixel_to_arm(Arm(), q, (o.x_min + o.x_max) / 2, (o.y_min + o.y_max) / 2, do, S['oo'].image_width, S['oo'].image_height)
    pr = fixed_cam_point((r.x_min + r.x_max) / 2, (r.y_min + r.y_max) / 2, dr, (k[0], k[4], k[2], k[5]), xyz)
    diffs.append(pr - po)
    print('OAK', np.round(po, 3).tolist(), ' RealSense', np.round(pr, 3).tolist(), ' diff', np.round(pr - po, 3).tolist())
if diffs:
    d = np.array(diffs)
    print(f'{len(d)} looks: mean diff (RealSense - OAK) {np.round(d.mean(0), 4).tolist()}, spread {np.round(d.std(0), 4).tolist()}')
    print('fixed_cam_xyz that matches the OAK:', np.round(np.array(xyz) - d.mean(0), 3).tolist(),
          '(a small spread with a clear mean = a real offset; x noise = the bottle moved)')
rclpy.shutdown()
