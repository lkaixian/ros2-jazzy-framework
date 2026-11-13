#!/usr/bin/env python3
"""What does the planning LLM actually receive? Captures the HTTP request llm_bridge sends.

Runs bt_validator (catalog) + llm_bridge against a capturing fake llama server and checks the
system prompt: node list with sync/async tags, robot knowledge, and the repair note path.
Domain PROMPT_DOMAIN (default 81). Exit code 0 only if every check passed.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

os.environ['ROS_DOMAIN_ID'] = os.environ.get('PROMPT_DOMAIN', '81')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

PORT = 18481
seen = []


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'{}')

    def do_POST(self):
        seen.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
        body = json.dumps({'choices': [{'message': {'content': '{"goal":"g","steps":[]}'}}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)


def main():
    HTTPServer.allow_reuse_address = True
    srv = HTTPServer(('127.0.0.1', PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    procs = [subprocess.Popen(c, start_new_session=True, stdout=subprocess.DEVNULL) for c in (
        ['ros2', 'run', 'bt_validator', 'bt_validator_node'],
        ['ros2', 'run', 'llm_bridge', 'llm_bridge_node', '--ros-args', '-p',
         f'server_url:=http://127.0.0.1:{PORT}', '-p', 'use_cluster:=false',
         '-p', 'probe_period_sec:=0.0'])]
    res = {}
    rclpy.init()
    n = None
    try:
        n = rclpy.create_node('prompt_probe')
        task = n.create_publisher(String, 'bt/task', 10)
        repair = n.create_publisher(String, 'bt/repair', 10)
        end = time.monotonic() + 6
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        for pub, text in ((task, 'go to the door'), (repair, 'REPAIR 1/1 failed at DetectPerson')):
            before = len(seen)
            end = time.monotonic() + 8
            while time.monotonic() < end and len(seen) == before:
                pub.publish(String(data=text))
                rclpy.spin_once(n, timeout_sec=0.3)
        system = seen[0]['messages'][0]['content'] if seen else ''
        res['request reached the LLM'] = bool(seen)
        res['prompt lists nodes with sync/async tags'] = (
            'DetectPerson[Condition,sync]' in system and 'Sleep[Action,async]' in system)
        res['prompt has the robot knowledge'] = (
            'Robot knowledge:' in system and 'no working arm' in system and 'far_corner' in system)
        res['prompt explains phases and repair notes'] = 'REPAIR note' in system and 'sync' in system
        res['bt/repair is planned like a task'] = len(seen) >= 2 and 'REPAIR 1/1' in \
            seen[1]['messages'][1]['content']
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        for p in procs:
            for sig in (signal.SIGINT, signal.SIGKILL):
                try:
                    os.killpg(p.pid, sig)
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
