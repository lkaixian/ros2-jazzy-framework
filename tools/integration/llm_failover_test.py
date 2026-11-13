#!/usr/bin/env python3
"""Sim check of llm_bridge: endpoints joining/leaving via the cluster, failover, emergency halt.

Two fake llama-servers (real HTTP, no models):
  secondary = the small model on the robot laptop, a STATIC endpoint of the bridge
  primary   = Gemma E4B on the strong PC; only known through a fake cluster agent's heartbeat
Scenario: only static -> strong PC joins (preferred) -> its server dies while still heartbeating
(fallback) -> it disconnects (heartbeat stops: endpoint removed, never called) -> everything dead
(emergency halt) -> small model returns (recovery) -> strong PC rejoins (preferred again).
Isolated DDS domain LLM_FAILOVER_DOMAIN (default 78). Exit code 0 only if every check passed.
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
from cluster_msgs.msg import NodeInfo, ServiceEndpoint  # noqa: E402
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
    big, small = Fake(P1, 'primary'), Fake(P2, 'secondary')
    proc = subprocess.Popen(
        ['ros2', 'run', 'llm_bridge', 'llm_bridge_node', '--ros-args',
         '-p', f"server_urls:=['http://127.0.0.1:{P2}']", '-p', 'probe_period_sec:=1.0',
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
        n.create_subscription(String, 'llm/status', lambda m: states.append(json.loads(m.data)), q)

        def halt(_req, resp):
            halts.append(time.monotonic())
            resp.success, resp.message = True, 'halted (test)'
            return resp
        n.create_service(Trigger, 'bt_executor/halt', halt)
        task = n.create_publisher(String, 'bt/task', 10)
        info = n.create_publisher(NodeInfo, 'cluster/node_info', rclpy.qos.QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE))
        host_alive = {'pc': False}

        def beat():
            if host_alive['pc']:
                m = NodeInfo()
                m.host_id = 'strong-pc'
                m.endpoints = [ServiceEndpoint(job_id='llm_big', kind='llm', priority=10,
                                               url=f'http://127.0.0.1:{P1}')]
                info.publish(m)
        n.create_timer(0.5, beat)

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)

        def ask(wait=6.0):
            before = len(specs)
            end = time.monotonic() + wait
            while time.monotonic() < end and len(specs) == before:
                task.publish(String(data='go'))
                spin(0.5)
            return specs[-1]['goal'] if len(specs) > before else None

        def last_state():
            return states[-1]['state'] if states else None

        spin(3.0)
        res['no host yet: static small model serves'] = ask() == 'secondary'
        host_alive['pc'] = True
        spin(2.0)
        res['strong PC joined: it is preferred'] = ask() == 'primary'
        big.stop()                                    # server dies, host still heartbeating
        res['strong PC server died: falls back to small'] = ask() == 'secondary'
        spin(2.5)
        res['state degraded'] = last_state() == 'degraded'
        res['no halt while one LLM is alive'] = not halts
        big.start()
        host_alive['pc'] = False                      # strong PC disconnects (no more heartbeats)
        spin(6.0)                                     # > endpoint_ttl_sec (4 s)
        urls = [e['url'] for e in (states[-1]['endpoints'] if states else [])]
        res['strong PC left: endpoint removed from the list'] = urls == [f'http://127.0.0.1:{P2}']
        res['strong PC left: small model serves'] = ask() == 'secondary'
        small.stop()
        t_dead = time.monotonic()
        spin(5.0)
        res['all gone: state down'] = last_state() == 'down'
        res['all gone: executor halt sent'] = bool(halts) and halts[0] > t_dead - 0.1
        res['all gone: task gets no spec'] = ask(wait=3.0) is None
        small.start()
        spin(4.0)
        res['small back: state recovers'] = last_state() in ('ok', 'degraded')
        res['small back: serves again'] = ask() == 'secondary'
        host_alive['pc'] = True
        spin(2.5)
        res['strong PC rejoined: preferred again'] = ask() == 'primary'
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(2)
        for f in (big, small):
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
