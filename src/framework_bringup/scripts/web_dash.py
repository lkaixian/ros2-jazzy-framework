#!/usr/bin/python3
"""web_dash: the `dash` view as a web page, with buttons for the same controls.

  ros2 run framework_bringup web_dash                  # http://127.0.0.1:8765/?t=<token> (printed)
  ros2 run framework_bringup web_dash --host 0.0.0.0   # reachable from the LAN (phone, other PC)

Shows what `dash` shows (LLM, battery, task, devices, plan tree, cluster, nodes) plus live views: the camera
image (MJPEG, ~5 fps), the lidar scan and the SLAM map (/map, once navigation runs). Offers: restart a node,
halt the task, stop / restart the whole system. Every request needs the token printed at startup (random per
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
from sensor_msgs.msg import Image, LaserScan
from yolo11_common.base_node import image_to_bgr

VIDEO = {'objects': '/detections/image', 'persons': '/detections/person_image',
         'faces': '/detections/face_image', 'raw': '/camera/image_raw'}
HERE = os.path.dirname(os.path.realpath(__file__))
PAGE = os.path.join(HERE, 'web_dash.html')


def load_dash():
    for name in ('dash', 'dash.py'):
        path = os.path.join(HERE, name)
        if os.path.exists(path):
            return importlib.machinery.SourceFileLoader('dash_mod', path).load_module()
    raise FileNotFoundError('dash next to web_dash not found')


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
        self._t_img, self._t_scan = {}, 0.0
        self.watched = {}   # source -> last time a browser asked for it: encode JPEGs only while someone watches
        self.jpeg = {}      # source -> latest JPEG
        for src, topic in VIDEO.items():
            node.create_subscription(Image, topic, lambda m, src=src: self._image(src, m), qos_profile_sensor_data)
        node.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, '/map', self._map, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
            ok, buf = cv2.imencode('.jpg', image_to_bgr(m), [cv2.IMWRITE_JPEG_QUALITY, 70])
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
        'msg': d.msg,
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
    a = ap.parse_args(argv)
    dash = load_dash()
    token = secrets.token_urlsafe(9)
    cache, actions = {'json': '{}', 'names': []}, queue.Queue()
    rclpy.init()
    node = rclpy.create_node('framework_web_dash')
    d, expected = dash.Dash(node, False, False), dash.expected_nodes()
    live = Live(node)
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
