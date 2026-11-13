#!/usr/bin/env python3
"""llm_bridge <-> llama-server link: TCP connection reuse and API-key authentication.

A keep-alive HTTP/1.1 fake server counts accepted TCP connections and (optionally) requires
`Authorization: Bearer <key>`. The bridge answers N llm/query calls.
  TRANSPORT_REQUIRE_KEY=1  server rejects calls without the key; the bridge gets LLM_API_KEY
  TRANSPORT_NO_BRIDGE_KEY=1  with REQUIRE_KEY: the bridge has no key and must be refused
  TRANSPORT_CALLS=N        number of calls (default 60)
Prints connections, mean ms/call and PASS/FAIL lines. Domain TRANSPORT_DOMAIN (default 82).
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

os.environ['ROS_DOMAIN_ID'] = os.environ.get('TRANSPORT_DOMAIN', '82')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_pipeline_msgs.srv import LlmQuery  # noqa: E402

PORT, KEY, CALLS = 18581, 's3cret-key', int(os.environ.get('TRANSPORT_CALLS', '60'))
REQUIRE = os.environ.get('TRANSPORT_REQUIRE_KEY') == '1'
stats = {'conns': 0, 'ok': 0, 'denied': 0, 'seen_headers': []}


class H(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'          # keep-alive: the connection lives until a side closes

    def setup(self):
        super().setup()
        stats['conns'] += 1
        if os.environ.get('TRANSPORT_SERVER_NODELAY', '1') == '1':
            # http.server writes headers and body separately; without NODELAY Nagle + the client's
            # delayed ACK stall every reply ~40 ms on a reused connection (cpp-httplib, which
            # llama-server uses, does not have this problem)
            self.request.setsockopt(__import__('socket').IPPROTO_TCP,
                                    __import__('socket').TCP_NODELAY, 1)

    def log_message(self, *a):
        pass

    def _send(self, code, body):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, b'{}')

    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length', 0)))
        auth = self.headers.get('Authorization', '')
        stats['seen_headers'].append(auth)
        if REQUIRE and auth != f'Bearer {KEY}':
            stats['denied'] += 1
            return self._send(401, b'{"error":"unauthorized"}')
        stats['ok'] += 1
        self._send(200, json.dumps({'choices': [{'message': {'content': '{"goal":"g"}'}}]}).encode())


class Srv(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    srv = Srv(('127.0.0.1', PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    env = dict(os.environ)
    NOKEY = os.environ.get('TRANSPORT_NO_BRIDGE_KEY') == '1'
    env.pop('LLM_API_KEY', None)
    if REQUIRE and not NOKEY:
        env['LLM_API_KEY'] = KEY
    bridge = subprocess.Popen(
        ['ros2', 'run', 'llm_bridge', 'llm_bridge_node', '--ros-args', '-p',
         f'server_url:=http://127.0.0.1:{PORT}', '-p', 'use_cluster:=false',
         '-p', 'probe_period_sec:=0.0'], env=env, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    res = {}
    rclpy.init()
    n = None
    try:
        n = rclpy.create_node('transport_probe')
        cli = n.create_client(LlmQuery, 'llm/query')
        if not cli.wait_for_service(timeout_sec=30):
            print('FAIL bridge service did not appear')
            return 1

        def call():
            r = LlmQuery.Request(system='s', prompt='p', json_mode=True)
            f = cli.call_async(r)
            rclpy.spin_until_future_complete(n, f, timeout_sec=10)
            return f.result()
        call()                                              # warm-up (first connection)
        t0 = time.monotonic()
        results = [call() for _ in range(CALLS)]
        ms = (time.monotonic() - t0) * 1000 / CALLS
        okc = sum(1 for r in results if r and r.ok)
        print(f'calls={CALLS} ok={okc} mean={ms:.2f} ms/call  tcp_connections={stats["conns"]} '
              f'server_ok={stats["ok"]} denied={stats["denied"]}')
        if REQUIRE and NOKEY:
            # the breaker opens after 2 refusals, so most calls fail fast without touching the server
            res['missing key is refused (HTTP 401), nothing served'] = okc == 0 and stats['ok'] == 0 \
                and stats['denied'] >= 1
        elif REQUIRE:
            res['every call authenticated'] = okc == CALLS and stats['denied'] == 0
            res['key sent as a Bearer header'] = all(h == f'Bearer {KEY}' for h in stats['seen_headers'])
        else:
            res['calls succeed'] = okc == CALLS
        res['connection is reused (<= 2 TCP connections for all calls)'] = stats['conns'] <= 2
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for sig in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(bridge.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(1)
        srv.shutdown()
        if n is not None:
            n.destroy_node()
        rclpy.try_shutdown()
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
