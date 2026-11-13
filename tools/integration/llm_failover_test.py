#!/usr/bin/env python3
"""Sim check of llm_bridge failover + emergency halt with two fake llama-servers (no models).

primary = stand-in for Gemma E4B on the strong PC, secondary = small model on the laptop.
Scenarios: both up -> kill primary (fallback, degraded) -> kill secondary (emergency, halt sent)
-> primary returns (recovery). Isolated DDS domain LLM_FAILOVER_DOMAIN (default 78).
Exit code 0 only if every check passed.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

os.environ['ROS_DOMAIN_ID'] = os.environ.get('LLM_FAILOVER_DOMAIN', '78')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

P1, P2 = 18281, 18282


class Fake:
    def __init__(self, port, name):
        self.port, self.name, self.srv = port, name, None
        self.start()

    def start(self):
        name = self.name

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200 if self.path == '/health' else 404)
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')

            def do_POST(self):
                self.rfile.read(int(self.headers.get('Content-Length', 0)))
                body = json.dumps({'choices': [{'message': {'role': 'assistant', 'content':
                                   json.dumps({'goal': name, 'steps': []})}}]}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(body)

        HTTPServer.allow_reuse_address = True
        self.srv = HTTPServer(('127.0.0.1', self.port), H)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def stop(self):
        self.srv.shutdown()
        self.srv.server_close()


def main():
    a, b = Fake(P1, 'primary'), Fake(P2, 'secondary')
    urls = f"['http://127.0.0.1:{P1}','http://127.0.0.1:{P2}']"
    proc = subprocess.Popen(
        ['ros2', 'run', 'llm_bridge', 'llm_bridge_node', '--ros-args',
         '-p', f'server_urls:={urls}', '-p', 'probe_period_sec:=1.0',
         '-p', 'startup_grace_sec:=0.0', '-p', 'cooldown_sec:=3.0',
         '-p', 'connect_timeout_sec:=1.0'], start_new_session=True)
    res = {}
    rclpy.init()
    n = None
    try:
        n = rclpy.create_node('llm_failover_probe')
        specs, states, halts = [], [], []
        n.create_subscription(String, 'bt/task_spec', lambda m: specs.append(json.loads(m.data)), 10)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(String, 'llm/status', lambda m: states.append(json.loads(m.data)['state']), q)

        def halt(_req, resp):
            halts.append(time.monotonic())
            resp.success, resp.message = True, 'halted (test)'
            return resp
        n.create_service(Trigger, 'bt_executor/halt', halt)
        task = n.create_publisher(String, 'bt/task', 10)

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)

        def ask(label, wait=6.0):
            before = len(specs)
            end = time.monotonic() + wait
            while time.monotonic() < end and len(specs) == before:
                task.publish(String(data='go'))
                spin(0.5)
            return specs[-1]['goal'] if len(specs) > before else None

        spin(3.0)
        res['both up: served by primary'] = ask('t1') == 'primary'
        res['state ok at start'] = 'ok' in states
        a.stop()
        res['primary dead: task still served (secondary)'] = ask('t2') == 'secondary'
        spin(2.5)
        res['state degraded'] = 'degraded' in states
        res['no halt while one LLM is alive'] = not halts
        b.stop()
        t_dead = time.monotonic()
        spin(5.0)
        res['both dead: state down'] = bool(states) and states[-1] == 'down'
        res['both dead: executor halt sent'] = bool(halts) and halts[0] > t_dead - 0.1
        res['both dead: task gets no spec'] = ask('t3', wait=3.0) is None
        a.start()
        spin(4.0)
        res['primary back: state recovers'] = bool(states) and states[-1] in ('ok', 'degraded')
        res['primary back: preferred again'] = ask('t4') == 'primary'
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(2)
        for f in (a, b):
            try:
                f.stop()
            except Exception:
                pass
        if n is not None:
            n.destroy_node()
        rclpy.try_shutdown()
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
