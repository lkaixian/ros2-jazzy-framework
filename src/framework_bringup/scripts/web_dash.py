#!/usr/bin/python3
"""web_dash: the `dash` view as a web page, with buttons for the same controls.

  ros2 run framework_bringup web_dash                  # http://127.0.0.1:8765/?t=<token> (printed)
  ros2 run framework_bringup web_dash --host 0.0.0.0   # reachable from the LAN (phone, other PC)
  ros2 run framework_bringup web_dash --arm-source ros # Arm page: follow the arm driver only (default auto: servos read directly while no driver runs)

Shows what `dash` shows (LLM, battery, task, devices, plan tree, cluster, nodes) plus live views: the camera
image (MJPEG, ~5 fps), the lidar scan and the SLAM map (/map, once navigation runs). Offers: restart a node,
halt the task, stop / restart the whole system. The Arm page also shows the arm monitor (tools/arm/arm_gui.py, reused as a
module): pose over the taught free space, servo temperature / voltage / current, gripper gap and current. Read-only. Every request needs the token printed at startup (random per
run); the default bind is loopback only, because the buttons can stop the robot's software. Plain HTTP:
do not expose it beyond a network you trust.
"""
import argparse
import collections
import importlib.machinery
import json
import math
import os
import queue
import secrets
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from audio_msgr.msg import AudioChunk, Transcript, TtsStatus
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from bt_pipeline_msgs.msg import BtPlan
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from yolo11_common.base_node import image_to_bgr

VIDEO = {'objects': '/detections/image', 'persons': '/detections/person_image',
         'faces': '/detections/face_image', 'raw': '/camera/image_raw', 'arm': '/arm_cam/rgb/image_raw'}
HERE = os.path.dirname(os.path.realpath(__file__))
PAGE = os.path.join(HERE, 'web_dash.html')


def load_dash():
    for name in ('dash', 'dash.py'):
        path = os.path.join(HERE, name)
        if os.path.exists(path):
            return importlib.machinery.SourceFileLoader('dash_mod', path).load_module()
    raise FileNotFoundError('dash next to web_dash not found')


def load_arm_tools():
    """tools/arm/arm_gui.py as a module (the arm monitor's logic), found by walking up from this script; (module, note)."""
    d = HERE
    for _ in range(8):
        cand = os.path.join(d, 'tools', 'arm')
        if os.path.exists(os.path.join(cand, 'arm_gui.py')):
            sys.path.insert(0, cand)
            try:
                import arm_gui
                arm_gui.load_context()
            except Exception as e:  # noqa: BLE001 - the dashboard must start without the arm tools
                return None, f'arm tools failed to load: {type(e).__name__}: {e}'
            return arm_gui, ''
        d = os.path.dirname(d)
    return None, 'tools/arm not found next to this install: arm detail is off'


class Live:
    """Latest camera JPEG, lidar scan JSON and map PNG, filled by subscriptions (rate-limited)."""

    def __init__(self, node):
        self.scan, self.map_png, self.map_meta = '{}', None, None
        self.voice, self.voice_seq, self.mic = collections.deque(maxlen=500), 0, 0.0      # speech events, mic RMS
        node.create_subscription(AudioChunk, 'audio/raw', self._mic, qos_profile_sensor_data)
        node.create_subscription(AudioChunk, 'stt/utterance', self._utterance, 10)
        node.create_subscription(Transcript, 'stt/transcript', self._transcript, 10)
        node.create_subscription(TtsStatus, 'tts/status', self._tts, 10)
        self.logs, self.log_seq = collections.deque(maxlen=3000), 0      # /rosout ring: [seq, time, level, node, text]
        node.create_subscription(Log, '/rosout', self._log, 200)
        self.node = node
        self.plan, self.tasks, self.errors = None, collections.deque(maxlen=20), collections.deque(maxlen=20)   # prompt page
        self.status, self.stop_state, self.mode = '', '', ''
        self.joints, self.joints_t = {}, 0.0                                                                    # arm page
        self.armmod, self.arm_note, self.arm_source, self.health, self.gripper_tool = None, 'arm detail off', 'off', None, None
        self.arm_rotate = None      # cv2 rotate code for the arm eye (the OAK is mounted upside down); None = as published
        self.task_pub = node.create_publisher(String, 'bt/task', 10)
        self.run_cli = node.create_client(Trigger, 'bt_executor/run')
        node.create_subscription(BtPlan, 'bt/plan', self._plan, 10)
        node.create_subscription(String, 'bt/task', lambda m: self.tasks.append([time.strftime('%H:%M:%S'), m.data]), 10)
        node.create_subscription(String, 'llm/error', lambda m: self.errors.append([time.strftime('%H:%M:%S'), m.data]), 10)
        node.create_subscription(String, 'bt/status', lambda m: setattr(self, 'status', m.data), 10)
        node.create_subscription(String, 'safety/stop_state', lambda m: setattr(self, 'stop_state', m.data), 10)
        node.create_subscription(String, 'robot/mode', lambda m: setattr(self, 'mode', m.data), 10)
        node.create_subscription(JointState, '/joint_states', self._joints, 10)
        node.create_subscription(String, 'arm/health', self._health, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._t_img, self._t_scan = {}, 0.0
        self.watched = {}   # source -> last time a browser asked for it: encode JPEGs only while someone watches
        self.jpeg = {}      # source -> latest JPEG
        for src, topic in VIDEO.items():
            node.create_subscription(Image, topic, lambda m, src=src: self._image(src, m), qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, '/map', self._map, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _plan(self, m):
        self.plan = {'time': time.strftime('%H:%M:%S'), 'task': m.task, 'spec': m.task_spec, 'xml': m.xml, 'valid': m.valid,
                     'errors': list(m.errors), 'attempts': m.attempts}

    def _joints(self, m):
        if not any(n.startswith('joint') for n in m.name):
            return          # the Kobuki publishes wheel_*_joint on the same topic: not the arm
        self.joints_t = time.monotonic()
        eff = list(m.effort) + [None] * len(m.name)
        self.joints = {n: [round(p, 3), round(m.velocity[i], 3) if i < len(m.velocity) else None, eff[i]]
                       for i, (n, p) in enumerate(zip(m.name, m.position)) if n.startswith(('joint', 'gripper'))}
        if self.arm_source == 'ros':
            self._arm_feed({n: p for n, p in zip(m.name, m.position)})

    def _health(self, m):
        try:
            self.health = json.loads(m.data)
        except ValueError:
            pass

    @staticmethod
    def arm_driver_running():
        return subprocess.run(['pgrep', '-f', 'lib/controller_manager/[r]os2_control_node'], capture_output=True).returncode == 0

    def _guarded(self, fn, stop):
        try:
            fn(stop)
        except OSError as e:            # e.g. UDP 8089 already taken by a running arm_gui.py
            self.arm_note = f'{fn.__name__}: {e}'

    def _sdk(self, on):
        """Start / stop the read-only direct servo poll (it releases the serial port when stopped)."""
        if on and self._sdk_stop is None:
            self._sdk_stop = threading.Event()
            threading.Thread(target=self._guarded, args=(self.armmod.poll_sdk, self._sdk_stop), daemon=True).start()
        elif not on and self._sdk_stop is not None:
            self._sdk_stop.set()
            self._sdk_stop = None

    def _arm_watch(self):
        """auto: read the servos directly while no arm driver runs; the moment one starts, let go of the port and follow the driver."""
        while True:
            driver = self.arm_driver_running()
            if driver and self.arm_source == 'sdk':
                self._sdk(False)
                self.arm_source = 'ros'
            elif not driver and self.arm_source == 'ros' and time.monotonic() - self.joints_t > 3.0:
                self.arm_source = 'sdk'
                self._sdk(True)
            time.sleep(0.5)

    def arm_start(self, source, rotate=180):
        """source: ros (the driver's /joint_states + arm/health, no bus access), sdk (servos read directly, read-only, only when no
        driver runs), auto (sdk while no driver runs, ros once one does), off."""
        self.arm_rotate = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}[rotate]
        if source == 'off':
            return
        self.armmod, self.arm_note = load_arm_tools()
        if self.armmod is None:
            return
        self._sdk_stop = None
        self.arm_source = 'ros' if source == 'ros' else 'sdk' if source == 'sdk' else ('ros' if self.arm_driver_running() else 'sdk')
        stop = threading.Event()        # the threads live as long as the process
        for fn in (self.armmod.sample_history, self.armmod.listen_udp):
            threading.Thread(target=self._guarded, args=(fn, stop), daemon=True).start()
        self._sdk(self.arm_source == 'sdk')
        if source == 'auto':
            threading.Thread(target=self._arm_watch, daemon=True).start()
        try:
            import gripper_tool
            self.gripper_tool = gripper_tool
        except Exception:  # noqa: BLE001 - no Dynamixel SDK here: the gripper gap is left out in ROS mode
            self.gripper_tool = None

    def _arm_feed(self, p):
        """/joint_states (+ temperature / voltage from arm/health) -> the arm monitor's reading."""
        if not self.armmod or not all(f'joint{i}' in p for i in range(1, 5)):
            return
        h = self.health or {}
        t, v = h.get('temps') or {}, h.get('volts') or {}
        ids = (11, 12, 13, 14)
        arm = {'pos': [p[f'joint{i}'] for i in range(1, 5)], 'cur': [0] * 4,
               'temp': [t.get(str(i)) for i in ids] if t else None, 'volt': [v.get(str(i)) for i in ids] if v else None}
        gt, gm, grip = self.gripper_tool, p.get('gripper_left_joint'), None
        if gt is not None and gm is not None:
            raw = round((gm + gt.M_OFFSET * -1) / gt.M_PER_TICK)
            grip = {'raw': raw, 'gap': round(gt.gap_mm(raw), 1), 'cur': 0, 'temp': t.get('15'), 'volt': v.get('15')}
        msg = 'driver mode: servo current is not published (no mA); temperature and voltage come from arm_health_node'
        if h and not h.get('ok', True):
            msg = 'arm health: ' + '; '.join(h.get('problems', ['unknown']))
        with self.armmod.S['lock']:
            self.armmod.S['base'] = {'src': 'ROS /joint_states', 'arm': arm, 'gripper': grip, 'msg': msg}

    def prompt(self):
        return json.dumps({'plan': self.plan, 'tasks': list(self.tasks), 'errors': list(self.errors), 'status': self.status})

    def arm(self):
        out = {'age': round(time.monotonic() - self.joints_t, 1) if self.joints_t else None, 'joints': self.joints}
        if self.armmod is None:
            out['detail_off'] = self.arm_note
        else:
            try:
                out['detail'] = self.armmod.snapshot()
            except Exception as e:  # noqa: BLE001 - no reading yet: the page shows the note
                out['detail_off'] = f'{type(e).__name__}: {e}'
            if self.arm_note:
                out['detail_note'] = self.arm_note
        return json.dumps(out)

    def arm_map(self):
        return json.dumps(self.armmod.CTX.get('map', {}) if self.armmod else {})

    @staticmethod
    def _rms(m):
        x = np.frombuffer(bytes(m.data)[:len(m.data) // 2 * 2], '<i2').astype(np.float32)
        return float(np.sqrt(np.mean(x * x))) if x.size else 0.0

    def _event(self, kind, text='', info=''):
        self.voice_seq += 1
        self.voice.append([self.voice_seq, time.strftime('%H:%M:%S'), kind, text, info])

    def _mic(self, m):
        self.mic = self._rms(m)

    def _utterance(self, m):
        self._event('utterance', '', f'{len(m.data) / 2 / m.sample_rate:.1f} s of speech sent to STT, level {self._rms(m):.0f}')

    def _transcript(self, m):
        conf = f'conf {m.confidence:.2f}' if m.confidence >= 0 else ''
        self._event('heard', m.text, ' '.join(x for x in (conf, m.language, f'{m.duration_sec:.1f} s') if x))

    def _tts(self, m):
        if m.state == TtsStatus.SPEAKING:
            self._event('said', m.text)
        elif m.state == TtsStatus.ERROR:
            self._event('tts error', m.text, m.error)

    def voice_since(self, seq):
        return json.dumps({'seq': self.voice_seq, 'mic': round(self.mic),
                           'items': [x for x in list(self.voice) if x[0] > seq][-200:]})

    def _log(self, m):
        self.log_seq += 1
        self.logs.append([self.log_seq, time.strftime('%H:%M:%S', time.localtime(m.stamp.sec)), m.level, m.name, m.msg])

    def logs_since(self, seq):
        items = [x for x in list(self.logs) if x[0] > seq][-500:]
        return json.dumps({'seq': self.log_seq, 'items': items})

    def _image(self, src, m):
        now = time.monotonic()
        if now - self.watched.get(src, -9.0) > 2.0 or now - self._t_img.get(src, 0.0) < 0.05:
            return          # nobody is watching this source / at most ~20 fps
        self._t_img[src] = time.monotonic()
        try:
            img = image_to_bgr(m)
            if src == 'arm' and self.arm_rotate is not None:
                img = cv2.rotate(img, self.arm_rotate)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        except ValueError:      # an encoding image_to_bgr does not decode (e.g. 16UC1 depth)
            return
        if ok:
            self.jpeg[src] = buf.tobytes()

    def _scan(self, m):
        if time.monotonic() - self._t_scan < 0.2:
            return
        self._t_scan = time.monotonic()
        step = max(1, len(m.ranges) // 360)
        r = [round(x, 2) if math.isfinite(x) and m.range_min <= x <= m.range_max else None for x in m.ranges[::step]]
        self.scan = json.dumps({'a0': m.angle_min, 'da': m.angle_increment * step, 'max': m.range_max, 'r': r})

    def _map(self, m):
        a = np.array(m.data, np.int8).reshape(m.info.height, m.info.width)
        img = np.full(a.shape, 205, np.uint8)       # unknown: grey
        img[a == 0], img[a > 0] = 255, 0            # free: white, occupied: black
        ok, buf = cv2.imencode('.png', cv2.flip(img, 0))      # grid row 0 is the bottom
        if ok:
            self.map_png = buf.tobytes()
            self.map_meta = {'w': m.info.width, 'h': m.info.height, 'res': m.info.resolution,
                             'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
                             'v': (self.map_meta or {}).get('v', 0) + 1}


def snapshot(dash, d, expected, live):
    n, now = d.n, time.monotonic()
    online = {x for x, _ in n.get_node_names_and_namespaces() if not x.startswith('_ros2cli')}
    names = sorted(set(expected) | online - {n.get_name()})
    d.names = names
    bat = None
    if d.battery and now - d.battery[2] < 10.0:
        lvl, txt = dash.battery_state(d.battery[0], d.battery[1])
        bat = {'level': lvl, 'text': txt}
    llm = dict(d.llm)
    llm['endpoints'] = [dict(e, link=d.links.get(e['url'])) for e in d.llm.get('endpoints', [])]
    cs = d.cluster
    return {
        'time': time.strftime('%H:%M:%S'),
        'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
        'secure': os.environ.get('ROS_SECURITY_ENABLE', '').lower() == 'true',
        'llm': llm, 'battery': bat, 'task': d.task, 'devices': d.device_rows(),
        'tree': [dict(t, s=d.nstate.get(str(t['uid']))) for t in d.tree],
        'cluster': None if cs is None else {
            'scheduler': cs.scheduler_id, 'alive': list(cs.hosts_alive), 'dead': list(cs.hosts_dead),
            'jobs': [{'id': j.job_id, 'host': j.host_id, 'state': j.job_state if j.state == 'placed' else j.state}
                     for j in cs.jobs]},
        'nodes': [{'name': a, 'state': b, 'note': c} for a, b, c in d.node_rows(names, online, now)],
        'video': sorted(live.jpeg), 'map': live.map_meta,
        'msg': d.msg, 'stop': live.stop_state, 'mode': live.mode,
    }


def make_handler(token, cache, actions, live):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _stream(self, live, src):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = None
            try:
                while True:
                    live.watched[src] = time.monotonic()
                    jpeg = live.jpeg.get(src)
                    if jpeg is not None and jpeg is not last:
                        last = jpeg
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                         str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass        # the browser closed the stream

        def _authed(self):
            q = parse_qs(urlparse(self.path).query)
            given = self.headers.get('X-Token') or (q.get('t') or [''])[0]
            return secrets.compare_digest(given, token)

        def do_GET(self):
            if not self._authed():
                return self._send(403, 'token missing or wrong: open the URL printed by web_dash', 'text/plain')
            path = urlparse(self.path).path
            if path == '/api/state':
                return self._send(200, cache['json'])
            if path == '/api/voice':
                since = (parse_qs(urlparse(self.path).query).get('since') or ['0'])[0]
                return self._send(200, live.voice_since(int(since) if since.isdigit() else 0))
            if path == '/api/logs':
                since = (parse_qs(urlparse(self.path).query).get('since') or ['0'])[0]
                return self._send(200, live.logs_since(int(since) if since.isdigit() else 0))
            if path == '/api/prompt':
                return self._send(200, live.prompt())
            if path == '/api/arm':
                return self._send(200, live.arm())
            if path == '/api/armmap':
                return self._send(200, live.arm_map())
            if path == '/api/scan':
                return self._send(200, live.scan)
            if path == '/map.png' and live.map_png:
                return self._send(200, live.map_png, 'image/png')
            if path == '/video.mjpg':
                return self._stream(live, (parse_qs(urlparse(self.path).query).get('src') or ['objects'])[0])
            if path == '/':
                with open(PAGE, 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            if not self._authed():
                return self._send(403, '{"error": "token"}')
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            except ValueError:
                return self._send(400, '{"error": "json"}')
            path, force = urlparse(self.path).path, bool(body.get('force'))
            if path == '/api/node' and body.get('name') in cache['names']:
                actions.put(('node', body['name'], force))
            elif path == '/api/task' and isinstance(body.get('text'), str) and body['text'].strip():
                actions.put(('task', body['text'].strip()[:500]))
            elif path == '/api/run':
                actions.put(('run',))
            elif path == '/api/halt':
                actions.put(('halt',))
            elif path == '/api/system' and body.get('act') in ('stop', 'restart'):
                actions.put(('system', body['act'], force))
            else:
                return self._send(400, '{"error": "unknown action"}')
            self._send(200, '{"ok": true}')
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--host', default='127.0.0.1', help='bind address (0.0.0.0 = whole LAN)')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--arm-source', choices=('auto', 'ros', 'sdk', 'off'), default='auto',
                    help='Arm page detail: auto = read the servos directly (read-only) while no arm driver runs and follow the driver '
                         '(/joint_states, arm/health) once one does; ros = driver only, no bus access; sdk = servos only; off')
    ap.add_argument('--arm-rotate', type=int, default=180, choices=(0, 90, 180, 270),
                    help='turn the arm eye upright (the OAK is mounted upside down)')
    a = ap.parse_args(argv)
    dash = load_dash()
    token = secrets.token_urlsafe(9)
    cache, actions = {'json': '{}', 'names': []}, queue.Queue()
    rclpy.init()
    node = rclpy.create_node('framework_web_dash')
    d, expected = dash.Dash(node, False, False), dash.expected_nodes()
    live = Live(node)
    live.arm_start(a.arm_source, a.arm_rotate)
    server = ThreadingHTTPServer((a.host, a.port), make_handler(token, cache, actions, live))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    shown = 'localhost' if a.host in ('127.0.0.1', '0.0.0.0') else a.host
    print(f'web_dash: http://{shown}:{a.port}/?t={token}', flush=True)
    if a.host not in ('127.0.0.1', 'localhost'):
        print('web_dash: reachable from the network; anyone with this URL can stop the robot software', flush=True)
    try:
        nxt, warm = 0.0, time.monotonic() + 2.5
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            while not actions.empty():
                act = actions.get()
                if act[0] == 'node':
                    d._node(act[1], act[2])
                elif act[0] == 'task':
                    live.task_pub.publish(String(data=act[1]))
                elif act[0] == 'run':
                    if live.run_cli.service_is_ready():
                        live.run_cli.call_async(Trigger.Request())
                    else:
                        d.say('bt_executor/run not available')
                elif act[0] == 'halt':
                    d.key('h')
                else:
                    threading.Thread(target=d._system, args=act[1:], daemon=True).start()
            if time.monotonic() >= max(nxt, warm):
                cache['json'] = json.dumps(snapshot(dash, d, expected, live), default=str)
                cache['names'] = d.names
                nxt = time.monotonic() + 1.0
    except KeyboardInterrupt:
        return 0
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
