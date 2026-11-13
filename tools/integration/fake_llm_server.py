#!/usr/bin/env python3
"""Stand-in for llama-server: GET /health, POST /v1/chat/completions -> {"goal": NAME, "steps": []}.

usage: fake_llm_server.py PORT NAME      (used by the integration tests; no model needed)
"""
import json
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT, NAME = int(sys.argv[1]), sys.argv[2]


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
                           json.dumps({'goal': NAME, 'steps': []})}}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)


HTTPServer.allow_reuse_address = True
HTTPServer(('127.0.0.1', PORT), H).serve_forever()
