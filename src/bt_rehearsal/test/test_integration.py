"""rehearse -> store -> retrieve -> validate in the kinematic nav sim, with a mocked LLM over real HTTP."""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from bt_memory.retrieve import first_candidate, retrieve
from bt_memory.schema import MemoryStore
from bt_rehearsal.isolation import Privileged
from bt_rehearsal.llm import LlmPlanner, Reflector, http_chat
from bt_rehearsal.loop import rehearse, to_memory
from bt_rehearsal.sim import Task
from bt_rehearsal.wiring import make_env

SENTINEL = 'GT_SENTINEL_7731'


def spec(place):
    return json.dumps({'goal': 'go', 'steps': [{'node': 'NavigateToPlace', 'ports': {'place': place}}]})


@pytest.fixture
def llm():
    prompts = []

    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            text = body['messages'][1]['content']
            prompts.append(body['messages'][0]['content'] + '\n' + text)
            if body['messages'][0]['content'].startswith('Write one short sentence'):
                out = "The place 'kitchen' does not exist; use a listed place."
            else:   # first plan names a place that does not exist; it learns only after feedback
                out = spec('right_room' if 'does not exist' in text else 'kitchen')
            data = json.dumps({'choices': [{'message': {'content': out}}]}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

    srv = HTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_port}', prompts
    srv.shutdown()


def test_full_loop(llm, tmp_path):
    url, prompts = llm
    env = make_env(scene={'objects': ['cup'], 'num_persons': 0})
    real_run = env['sim'].run

    def run_with_secret(xml):          # ground truth carries a sentinel: it must never reach the planner
        obs, priv = real_run(xml)
        return obs, Privileged(priv.pose, 0, {'secret': SENTINEL})
    env['sim'].run = run_with_secret
    chat = http_chat(url, timeout=5)
    task = Task('go to the right room', 'right_room')
    res = rehearse(task, LlmPlanner(chat, env['listing']), env['build'], env['validate'], env['sim'],
                   Reflector(chat), max_attempts=3)
    assert res.success and res.attempts == 2 and len(res.lessons) == 1 and 'does not exist' in res.lessons[0]

    store = MemoryStore(str(tmp_path))
    path = store.save(to_memory(task, res, 'kobuki', '0.1.0', 'mock-llm'))

    # data isolation: nothing derived from ground truth in any prompt or in the stored memory
    blob = '\n'.join(prompts) + open(path).read()
    assert SENTINEL not in blob and 'ground_truth' not in blob.lower()

    # deployment: retrieve and re-validate against the CURRENT catalog; the sim agrees
    hits = retrieve(store, 'Go to the right room.', {'objects': ['cup']}, 'kobuki')
    cand = first_candidate(hits, env['validate'])
    assert cand and not env['validate'](cand.xml)
    obs, priv = real_run(cand.xml)
    assert env['sim'].score(task, obs, priv)

    # a memory that no longer validates is not offered
    assert first_candidate(hits, lambda x: ['catalog changed']) is None


def test_anthropic_chat_request_shape(tmp_path):
    from bt_rehearsal.llm import anthropic_chat
    seen = {}

    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            seen['path'], seen['key'] = self.path, self.headers['x-api-key']
            seen['ver'] = self.headers['anthropic-version']
            seen['body'] = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            data = json.dumps({'content': [{'type': 'text', 'text': 'use a listed place'}]}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass
    srv = HTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    kf = tmp_path / 'key'
    kf.write_text('sk-test-123\n')
    out = anthropic_chat(str(kf), 'm', base_url=f'http://127.0.0.1:{srv.server_port}')('sys', 'prompt')
    srv.shutdown()
    assert out == 'use a listed place' and seen['path'] == '/v1/messages' and seen['key'] == 'sk-test-123'
    assert seen['ver'] == '2023-06-01' and seen['body']['model'] == 'm' and seen['body']['system'] == 'sys'
