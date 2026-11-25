#!/usr/bin/env python3
"""Touch test: checks the hand-eye pose, the depth and the camera intrinsics in one go. The arm is moved BY HAND (torque off, driver stopped).

    python3 tools/arm/touch_test.py --make-marker      # writes ~/arm_marker_4x4_id0.png: print it ~50 mm wide on stiff paper
    python3 tools/arm/touch_test.py                    # the test (start the OAK driver first: tools/arm/arm_eye.sh or sensors.launch enable_oak)

Setup: tape the printed marker card by its edge to something rigid (a box edge) so the marker hangs free and cannot move.
  t  TOUCH: pinch the marker's centre between the closed finger pads (the pad-centre point). The true position of the target in the arm frame is read from the servos.
  v  VIEW:  open the gripper, move the arm by hand so the camera sees the marker from 40-60 cm, hold still, press v. Repeat from 4+ different poses/distances/angles.
  s  show the summary (and save it), q quit.
The camera cannot see the pad-centre point at touch distance (it is behind and above the pads, and depth needs > ~35 cm), hence two steps.
Each view's estimate is compared with the touched position. A constant error in the GRIPPER frame means the camera offset is off by that much; an error that grows
with distance along the optical axis means a depth scale error. IDEAL CASE: +-5 mm hand measurement, marker centre +-1 mm, pad-centre touching +-3 mm.
"""
import argparse
import json
import os
import threading
import time

import numpy as np
from free_map import JOINTS
from hand_eye import R_G_C, load, to_arm

MARKER_PX_DICT = 'DICT_4X4_50'


def detect_marker(img_bgr):
    """(u, v) centre of the first ArUco 4x4 marker in a raw BGR image, or None."""
    import cv2
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_PX_DICT)), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY))
    if ids is None or not len(ids):
        return None
    c = corners[0].reshape(4, 2).mean(axis=0)
    return float(c[0]), float(c[1])


def depth_at(depth_mm, u, v, half=3):
    """Median of the valid depth (m) in a small window around the raw pixel, or None."""
    h, w = depth_mm.shape
    win = depth_mm[max(0, int(v) - half):int(v) + half + 1, max(0, int(u) - half):int(u) + half + 1]
    win = win[(win > 0) & (win < 20000)]
    return float(np.median(win)) / 1000.0 if win.size >= 10 else None


def analyse(arm, touch_q, views, position):
    """Compare each view's camera estimate with the touched position.

    views: [{'q': {joint: rad}, 'p_opt': raw-optical-frame point of the marker centre (3,)}]. Returns the per-view errors, the mean error in the GRIPPER frame
    (= assumed camera offset - true camera offset, so the corrected offset is position - mean), and the slope of the error along the optical axis against range.
    """
    target = arm.xyz('end_effector_link', touch_q)
    rows = []
    for v in views:
        est = to_arm(arm, v['q'], v['p_opt'], position)
        m = arm.pose('end_effector_link', v['q'])
        e_arm = est - target
        e_grip = m[:3, :3].T @ e_arm                       # error expressed in the gripper frame at the view pose
        rows.append({'range_m': float(v['p_opt'][2]), 'err_arm_mm': (e_arm * 1000).tolist(), 'err_grip_mm': (e_grip * 1000).tolist(),
                     'err_optical_z_mm': float((R_G_C.T @ e_grip)[2] * 1000)})
    eg = np.array([r['err_grip_mm'] for r in rows])
    rng, ez = np.array([r['range_m'] for r in rows]), np.array([r['err_optical_z_mm'] for r in rows])
    slope = float(np.polyfit(rng, ez, 1)[0]) if len(rows) >= 3 and np.ptp(rng) > 0.05 else None    # mm of error per metre of range
    return {'target_arm_m': target.tolist(), 'views': rows, 'mean_err_grip_mm': eg.mean(axis=0).tolist(),
            'std_err_grip_mm': eg.std(axis=0).tolist(), 'depth_slope_mm_per_m': slope,
            'suggested_position_m': (np.asarray(position) - eg.mean(axis=0) / 1000.0).tolist()}


class Reader(threading.Thread):
    """Reads the servos continuously (read-only), streams them to the GUI, keeps the latest joint angles."""

    def __init__(self):
        super().__init__(daemon=True)
        from load_test import RealBus
        self.bus, self.q, self.stop = RealBus(None), None, threading.Event()

    def run(self):
        from load_test import send_arm
        while not self.stop.is_set():
            try:
                st = self.bus.read()
                self.q = dict(zip(JOINTS, (float(v) for v in st['pos'])))
                send_arm(st)
            except Exception:  # noqa: BLE001 - a missed read: try again
                pass
            time.sleep(0.1)


def make_marker():
    import cv2
    img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_PX_DICT)), 0, 400)
    img = cv2.copyMakeBorder(img, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
    path = os.path.expanduser('~/arm_marker_4x4_id0.png')
    cv2.imwrite(path, img)
    print(f'wrote {path}: print it so the black square is about 50 mm wide (check with a ruler), stick it on stiff card')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--make-marker', action='store_true')
    ap.add_argument('--topic', default='/arm_cam/rgb/image_raw')
    ap.add_argument('--depth-topic', default='/arm_cam/stereo/image_raw')
    a = ap.parse_args()
    if a.make_marker:
        return make_marker()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image
    from omx_fk import Arm
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if uri.startswith('file://') and not os.path.isfile(uri[7:]):
        del os.environ['CYCLONEDDS_URI']
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node, last = Node('touch_test'), {}
    node.create_subscription(Image, a.topic, lambda m: last.__setitem__('rgb', m), qos_profile_sensor_data)
    node.create_subscription(Image, a.depth_topic, lambda m: last.__setitem__('depth', m), qos_profile_sensor_data)
    position, k, _ = load()
    arm, touch_q, views = Arm(), None, []
    reader = Reader()
    reader.start()
    print(__doc__.split('\n\n')[1])
    try:
        while True:
            cmd = input('\n[t]ouch  [v]iew  [s]ummary  [q]uit > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 't':
                time.sleep(0.5)
                touch_q = dict(reader.q or {})
                if touch_q:
                    print('touch recorded: gripper pad-centre at', np.round(arm.xyz('end_effector_link', touch_q) * 1000).astype(int).tolist(), 'mm (arm frame)')
                else:
                    print('no servo data (is the arm driver stopped?)')
            elif cmd == 'v':
                if touch_q is None:
                    print('record the touch first (t)')
                    continue
                us, vs, zs, qs = [], [], [], []
                end = time.time() + 2.0
                while time.time() < end:                              # ~2 s of frames while the arm is held still
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if 'rgb' in last and 'depth' in last and reader.q:
                        m, d = last['rgb'], last['depth']
                        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                        img = np.ascontiguousarray(img[:, :, ::-1] if m.encoding == 'rgb8' else img)
                        c = detect_marker(img)
                        dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step // 2)[:, :d.width]
                        z = depth_at(dep, *c) if c else None
                        if c and z:
                            us.append(c[0]); vs.append(c[1]); zs.append(z); qs.append(dict(reader.q))
                if len(zs) < 8:
                    print(f'marker/depth seen in only {len(zs)} frames: move to 40-60 cm, keep the marker flat to the camera and well lit')
                    continue
                u, v, z = float(np.median(us)), float(np.median(vs)), float(np.median(zs))
                q = {j: float(np.median([x[j] for x in qs])) for j in JOINTS}
                p_opt = np.array([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z])
                views.append({'q': q, 'p_opt': p_opt})
                r = analyse(arm, touch_q, views[-1:], position)['views'][0]
                print(f"view {len(views)}: range {z:.2f} m, error vs the touched point {np.round(r['err_arm_mm']).astype(int).tolist()} mm (arm frame), "
                      f"{np.round(r['err_grip_mm']).astype(int).tolist()} mm (gripper frame)")
            elif cmd == 's':
                if touch_q is None or not views:
                    print('need a touch and at least one view')
                    continue
                res = analyse(arm, touch_q, views, position)
                print(f"\n{len(views)} views. mean error in the gripper frame (mm): {np.round(res['mean_err_grip_mm'], 1).tolist()}  std {np.round(res['std_err_grip_mm'], 1).tolist()}")
                if res['depth_slope_mm_per_m'] is not None:
                    print(f"error along the optical axis grows {res['depth_slope_mm_per_m']:.1f} mm per metre of range (a depth scale error if large)")
                print('suggested camera position in the gripper frame (m):', np.round(res['suggested_position_m'], 4).tolist(), '(currently', position.tolist(), ')')
                path = os.path.expanduser(time.strftime('~/arm_touch_test_%H%M%S.json'))
                json.dump(res, open(path, 'w'), indent=1)
                print('saved', path)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        reader.stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
