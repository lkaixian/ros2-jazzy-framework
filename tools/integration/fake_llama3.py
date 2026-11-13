import json, os
from http.server import BaseHTTPRequestHandler, HTTPServer
class H(BaseHTTPRequestHandler):
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        open(os.environ['CAP'], 'w').write(body['messages'][0]['content'])
        out = json.dumps({"choices": [{"message": {"role": "assistant", "content": json.dumps({"goal": "find", "steps": [{"node": "DetectObject", "ports": {"label": "pedestrian"}}]})}}]}).encode()
        self.send_response(200); self.send_header("Content-Length", str(len(out))); self.end_headers(); self.wfile.write(out)
    def log_message(self, *a): pass
HTTPServer(("127.0.0.1", 8080), H).serve_forever()
