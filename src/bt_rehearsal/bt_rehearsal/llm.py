"""Planner and reflector backed by an LLM endpoint, with deterministic fallbacks. Pure Python.

`chat(system, prompt) -> str` is injected: an HTTP call to a llama-server style endpoint (`http_chat`)
or, inside ROS, a client of llm_bridge. Any exception from `chat` means "endpoint unreachable".
"""
import json
import os
import urllib.request

from bt_memory.schema import lint

GRAMMAR = ('Reply with ONE JSON object only: {"goal": str, "steps": [STEP, ...]} where '
           'STEP = {"node": "<NodeId>", "ports": {"<port>": "<value>"}} or '
           '{"group": "Sequence|Fallback", "retry": N?, "steps": [STEP, ...]}. "node" is always a plain string. '
           'Use only the nodes listed, and only the ports a task needs.')


class PlannerUnavailable(RuntimeError):
    pass


def http_chat(endpoint, timeout=30.0, api_key='', schema=None):
    """OpenAI-compatible /v1/chat/completions (llama-server). Raises on any network or HTTP error."""
    def chat(system, prompt):
        body = json.dumps({'messages': [{'role': 'system', 'content': system},
                                        {'role': 'user', 'content': prompt}],
                           'temperature': 0.2, 'max_tokens': 1500,
                           **({'response_format': {'type': 'json_schema',
                                                   'json_schema': {'name': 'task_spec', 'schema': schema}}}
                              if schema and system.endswith(GRAMMAR) else {})}).encode()
        req = urllib.request.Request(endpoint.rstrip('/') + '/v1/chat/completions', body,
                                     {'Content-Type': 'application/json',
                                      **({'Authorization': f'Bearer {api_key}'} if api_key else {})})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)['choices'][0]['message']['content']
    return chat


def anthropic_chat(key_file, model='claude-haiku-5-5', timeout=60.0, base_url='https://api.anthropic.com'):
    """Messages API as a `chat(system, prompt)`. The key is read from a file (never an argument or the repo).
    Sends only what the reflector already sees: task text, plan ports, validator/sim feedback."""
    with open(os.path.expanduser(key_file)) as f:
        key = f.read().strip()

    def chat(system, prompt):
        body = json.dumps({'model': model, 'max_tokens': 300, 'system': system,
                           'messages': [{'role': 'user', 'content': prompt}]}).encode()
        req = urllib.request.Request(base_url.rstrip('/') + '/v1/messages', body,
                                     {'Content-Type': 'application/json', 'x-api-key': key,
                                      'anthropic-version': '2023-06-01'})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)['content'][0]['text']
    return chat


def planner_prompt(task, listing, feedback=(), examples=()):
    parts = [f'Task: {task}', f'Available nodes:\n{listing}']
    if examples:
        parts.append('Earlier successful plans for similar tasks (they may not fit this scene):\n'
                     + '\n'.join(examples))
    if feedback:
        parts.append('Feedback from earlier attempts on this task (fix these):\n' + '\n'.join(feedback))
    return '\n\n'.join(parts)


def _json_only(text):
    a, b = text.find('{'), text.rfind('}')
    if a < 0 or b < a:
        raise ValueError('no JSON object in reply')
    json.loads(text[a:b + 1])
    return text[a:b + 1]


class LlmPlanner:
    def __init__(self, chat, listing, examples=(), knowledge=''):
        """knowledge: the production prompt text (param/llm_knowledge.md); '' = the bare grammar only."""
        self.chat, self.listing, self.examples = chat, listing, list(examples)
        self.system = f'{knowledge}\n\n{GRAMMAR}' if knowledge else GRAMMAR

    def propose(self, task, feedback=()):
        try:
            text = self.chat(self.system, planner_prompt(task, self.listing, feedback, self.examples))
        except Exception as e:  # noqa: BLE001  any transport failure = unreachable
            raise PlannerUnavailable(f'{type(e).__name__}: {e}') from e
        return _json_only(text)


# shortcut: remedies are hand-written per failure message, not learned; add a row when a new failure shows up
_REMEDY = (('timeout', 'leave out timeout_ms (do not invent time limits)'),
           ('unknown place', 'use only the place names listed under Places, spelled exactly'),
           ('no path', 'pick a reachable place or go via the door first'),
           ('unsupported', 'use only navigation and control nodes'))


def template_lesson(obs):
    """Deterministic lesson from Observable data only. Works offline."""
    if obs.validator_errors:
        return f'The validator rejected the plan: {obs.validator_errors[0]}'
    o = obs.outcome
    if o.get('result') == 'FAILURE':
        msg = o.get('message') or 'no message'
        fix = next((r for k, r in _REMEDY if k in msg), 'choose another approach for that step')
        return f"{o.get('failed_node') or 'a node'} failed ({msg}); {fix}"
    return 'The run succeeded but did not reach the goal; check the destination'


class Reflector:
    """One lesson per failed attempt: the template remedy, plus the LLM's sentence when reachable and lint-clean."""

    def __init__(self, chat=None):
        self.chat = chat

    def lesson(self, task, obs, detail=''):
        base = template_lesson(obs)
        if self.chat:
            try:
                text = self.chat('Write one short sentence: a lesson for a robot planner. No numbers, '
                                 'coordinates or ids.', f'Task: {task}\n{obs.context_text()}' + (f'\n{detail}' if detail else '')
                                 ).strip().splitlines()[0]
                if text and not lint(text):
                    return f'{base}. {text}'      # the concrete remedy is never replaced by a vague LLM sentence
            except Exception:  # noqa: BLE001
                pass
        return base
