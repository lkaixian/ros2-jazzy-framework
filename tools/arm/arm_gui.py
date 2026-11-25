#!/usr/bin/env python3
"""Arm monitor: a READ-ONLY browser page (no button moves anything) for the arm, the gripper and the arm eye.

    python3 tools/arm/arm_gui.py                      # then open http://127.0.0.1:8088
    python3 tools/arm/arm_gui.py --yolo yolo26s.pt    # draw YOLO boxes on the arm-eye view
    python3 tools/arm/arm_gui.py --no-camera          # arm and gripper only

Shows: the arm skeleton over the hand-taught free-space map (top and side), joint angles against the ranges seen, the gripper gap and
current over the last minute, servo health (temperature, voltage, error flag, current), the gravity-load estimate, and the OAK image
(rotated in software) with detections and the distance ahead. Data: the ROS /joint_states when the arm driver runs, otherwise the servos
read directly (read-only, with the arm_serial lock). While a tool (gripper_tool, load_test, teach) runs, it streams its own readings here.
"""
import argparse
import collections
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

S = {'lock': threading.Lock(), 'base': {}, 'tool': {}, 'tool_t': {}}
HIST = collections.deque(maxlen=900)
FR = {'jpg': None, 'depth_m': None, 'dets': [], 'err': '', 't': 0.0}
CTX = {}
ROT = {0: None, 90: 'ROTATE_90_CLOCKWISE', 180: 'ROTATE_180', 270: 'ROTATE_90_COUNTERCLOCKWISE'}


# ----------------------------------------------------------------------------------------------------- state
def snapshot(record=False):
    """The JSON the page polls: merged readings, skeleton, load, history. Only the 10 Hz sampler (record=True) extends the history."""
    from free_map import JOINTS, LINKS
    now = time.time()
    with S['lock']:
        base, tool, tool_t = dict(S['base']), dict(S['tool']), dict(S['tool_t'])
    arm, grip, src = base.get('arm'), base.get('gripper'), base.get('src', 'none')
    if now - tool_t.get('arm', 0) < 0.7:
        arm, src = tool['arm'], 'tool'
    if now - tool_t.get('gripper', 0) < 0.7:
        grip, src = tool['gripper'], 'tool'
    d = {'t': now, 'src': src, 'msg': base.get('msg', ''), 'range': CTX.get('range', {}), 'frame': {'depth_m': FR['depth_m'], 'dets': FR['dets'], 'err': FR['err']}}
    if arm:
        q = dict(zip(JOINTS, arm['pos']))
        arm_model = CTX['arm']
        pts = [np.zeros(3)] + [arm_model.xyz(link, q) for link in LINKS]
        tau = CTX['model'].torques(q)
        d['joints'], d['sk'] = list(map(float, arm['pos'])), [[float(v) for v in p] for p in pts]
        d['load'] = max(abs(t) for k, t in tau.items() if k != 'joint1') / 4.1
        d['servo'] = {'cur_ma': [abs(c) * 2.69 for c in arm.get('cur', [0] * 4)], 'temp': arm.get('temp'), 'volt': arm.get('volt'), 'err': arm.get('err'),
                      'torque': arm.get('torque')}
    if grip:
        d['grip'] = {'raw': grip.get('raw'), 'gap': grip.get('gap'), 'cur_ma': abs(grip.get('cur', 0)) * 2.69, 'temp': grip.get('temp'), 'volt': grip.get('volt'),
                     'err': grip.get('err'), 'torque': grip.get('torque')}
        if record:
            HIST.append((now, grip.get('gap') or 0, abs(grip.get('cur', 0)) * 2.69, d.get('load', 0) * 100))
    d['hist'] = [[round(t - now, 1), g, round(c), round(ld, 1)] for t, g, c, ld in list(HIST)[-450:]]
    return d


def sample_history(stop):
    while not stop.is_set():
        try:
            snapshot(record=True)
        except Exception:  # noqa: BLE001 - no data yet: try again
            pass
        time.sleep(0.1)


def listen_udp(stop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', 8089))
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            m = json.loads(sock.recvfrom(65536)[0])
        except (socket.timeout, ValueError):
            continue
        with S['lock']:
            S['tool'][m['k']], S['tool_t'][m['k']] = m, time.time()


def poll_sdk(stop):
    """Read the servos directly, read-only, only when no other tool holds the port."""
    import bus_lock
    import gripper_tool
    from load_test import BusError, RealBus
    bus = None
    while not stop.is_set():
        lock = bus_lock.acquire(0.05)
        if lock is None:
            time.sleep(0.05)
            continue
        msg = ''
        try:
            if bus is None:
                bus = RealBus(None, lock=False)
            st = bus.read()
            g = gripper_tool.read_state(bus.port, bus.pk)
            arm = {'pos': st['pos'].tolist(), 'cur': st['cur'].tolist(), 'temp': st['temp'].tolist(), 'volt': st['volt'].tolist(), 'err': st['err'].tolist()}
            grip = {'raw': g['raw'], 'cur': g['current_units'], 'gap': round(gripper_tool.gap_mm(g['raw']), 1), 'temp': g['temp'], 'volt': g['volt'],
                    'err': g['error'], 'torque': g['torque']}
            with S['lock']:
                S['base'] = {'src': 'servos (direct)', 'arm': arm, 'gripper': grip, 'msg': ''}
        except (BusError, SystemExit, OSError) as e:
            msg = f'servo read failed: {e}'
            bus = None
            with S['lock']:
                S['base'] = dict(S['base'], msg=msg)
        finally:
            lock.close()
        time.sleep(0.1)


# ----------------------------------------------------------------------------------------------- ROS (camera, joints)
def ros_thread(args, stop, ros_joints):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image, JointState
    import cv2
    import gripper_tool
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('arm_gui')
    rot = getattr(cv2, ROT[args.rotate]) if ROT[args.rotate] else None
    model = None
    if args.yolo:
        import torch
        from ultralytics import YOLO
        os.makedirs(os.path.expanduser('~/.cache/arm_gui'), exist_ok=True)
        os.chdir(os.path.expanduser('~/.cache/arm_gui'))                   # weights download here, not into the repo
        model, dev = YOLO(args.yolo), (0 if torch.cuda.is_available() else 'cpu')

    def to_img(m, channels):
        """Image msg -> (h, w, 3) uint8 BGR or (h, w) uint16, indexing rows by `step` (it may include padding)."""
        if channels:
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            return a[:, :, ::-1] if m.encoding == 'rgb8' else a
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]

    def on_rgb(m):
        try:
            img = np.ascontiguousarray(to_img(m, 3))
            img = cv2.rotate(img, rot) if rot is not None else img
            dets = []
            if model is not None:
                r = model(img, device=dev, verbose=False, conf=0.35)[0]
                img = r.plot()
                dets = [{'label': model.names[int(c)], 'conf': round(float(p), 2)} for c, p in zip(r.boxes.cls, r.boxes.conf)][:8]
            img = cv2.resize(img, (640, int(640 * img.shape[0] / img.shape[1])))
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                FR.update(jpg=jpg.tobytes(), dets=dets, t=time.time(), err='')
        except Exception as e:  # noqa: BLE001 - a bad frame must not kill the viewer
            FR['err'] = f'{type(e).__name__}: {e}'

    def on_depth(m):
        try:
            d = to_img(m, 0)
            d = cv2.rotate(d, rot) if rot is not None else d
            h, w = d.shape[:2]
            c = d[h // 2 - 30:h // 2 + 30, w // 2 - 30:w // 2 + 30]
            c = c[c > 0]
            FR['depth_m'] = round(float(np.median(c)) / 1000, 2) if c.size > 200 else None
        except Exception:  # noqa: BLE001
            FR['depth_m'] = None

    def on_joints(m):
        p = dict(zip(m.name, m.position))
        if not all(f'joint{i}' in p for i in range(1, 5)):
            return
        gm = p.get('gripper_left_joint')
        raw = None if gm is None else round((gm + gripper_tool.M_OFFSET * -1) / gripper_tool.M_PER_TICK)
        with S['lock']:
            S['base'] = {'src': 'ROS /joint_states', 'arm': {'pos': [p[f'joint{i}'] for i in range(1, 5)], 'cur': [0] * 4},
                         'gripper': None if raw is None else {'raw': raw, 'gap': round(gripper_tool.gap_mm(raw), 1), 'cur': 0}, 'msg': 'driver running: health shown by the driver, not here'}

    if not args.no_camera:
        node.create_subscription(Image, args.topic, on_rgb, qos_profile_sensor_data)
        node.create_subscription(Image, args.depth_topic, on_depth, qos_profile_sensor_data)
    if ros_joints:
        node.create_subscription(JointState, '/joint_states', on_joints, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    while not stop.is_set():
        ex.spin_once(timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


# ------------------------------------------------------------------------------------------------------ HTTP
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/':
            self._send(page().encode(), 'text/html; charset=utf-8')
        elif self.path == '/state':
            self._send(json.dumps(snapshot()).encode(), 'application/json')
        elif self.path == '/map':
            self._send(json.dumps(CTX.get('map', {})).encode(), 'application/json')
        elif self.path == '/video':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last = 0.0
            try:
                while True:
                    if FR['jpg'] is not None and FR['t'] != last:
                        last = FR['t']
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + FR['jpg'] + b'\r\n')
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(404)


def load_context():
    from free_map import FreeMap
    from omx_fk import Arm
    from path_check import default_map
    from torque import TorqueModel
    CTX['arm'] = Arm()
    CTX['model'] = TorqueModel(arm=CTX['arm'])
    path = default_map()
    if path:
        fm = FreeMap.load(path)
        c = (np.array(sorted(fm.cells)) + 0.5) * fm.v
        pick = lambda cols: np.unique(np.round(c[:, cols], 2), axis=0)  # noqa: E731 - one point per 1 cm cell is plenty to draw
        CTX['map'] = {'top': pick([1, 0]).tolist(), 'side': pick([0, 2]).tolist()}
        CTX['range'] = {k: [fm.lo[k], fm.hi[k]] for k in fm.lo}


HERE = os.path.dirname(os.path.realpath(__file__))


def page():
    with open(os.path.join(HERE, 'arm_gui.html')) as f:
        return f.read()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', type=int, default=8088)
    ap.add_argument('--bind', default='127.0.0.1')
    ap.add_argument('--rotate', type=int, default=180, choices=sorted(ROT), help='rotate the OAK image in software (it is mounted upside down)')
    ap.add_argument('--yolo', default='', help='weights name, e.g. yolo26s.pt: draw detections on the arm-eye view')
    ap.add_argument('--topic', default='/arm_cam/rgb/image_raw')
    ap.add_argument('--depth-topic', default='/arm_cam/stereo/image_raw')
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--source', choices=('auto', 'ros', 'sdk'), default='auto')
    a = ap.parse_args()
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if uri.startswith('file://') and not os.path.isfile(uri[7:]):
        print(f'warn: ignoring CYCLONEDDS_URI, file missing: {uri}')
        del os.environ['CYCLONEDDS_URI']
    driver = subprocess.run(['pgrep', '-f', 'lib/controller_manager/[r]os2_control_node'], capture_output=True).returncode == 0
    use_ros_joints = a.source == 'ros' or (a.source == 'auto' and driver)
    load_context()
    stop = threading.Event()
    threads = [threading.Thread(target=listen_udp, args=(stop,), daemon=True), threading.Thread(target=sample_history, args=(stop,), daemon=True)]
    if not use_ros_joints:
        threads.append(threading.Thread(target=poll_sdk, args=(stop,), daemon=True))
    if use_ros_joints or not a.no_camera:
        threads.append(threading.Thread(target=ros_thread, args=(a, stop, use_ros_joints), daemon=True))
    for t in threads:
        t.start()
    srv = ThreadingHTTPServer((a.bind, a.port), Handler)
    print(f"arm monitor on http://{a.bind}:{a.port}  (source: {'ROS /joint_states' if use_ros_joints else 'servos, read-only'}; camera: {'off' if a.no_camera else a.topic}; "
          f"yolo: {a.yolo or 'off'})  Ctrl+C to stop")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    stop.set()
    srv.server_close()
    sys.exit(0)


if __name__ == '__main__':
    main()
