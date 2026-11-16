"""Planner and reflector backed by an LLM endpoint, with deterministic fallbacks. Pure Python.

`chat(system, prompt) -> str` is injected: an HTTP call to a llama-server style endpoint (`http_chat`)
or, inside ROS, a client of llm_bridge. Any exception from `chat` means "endpoint unreachable".
"""
import json
import urllib.request

from bt_memory.schema import lint

GRAMMAR = ('Reply with ONE JSON object only: {"goal": str, "steps": [STEP, ...]} where '
           'STEP = {"node": "<NodeId>", "ports": {"<port>": "<value>"}} or '
           '{"group": "Sequence|Fallback", "retry": N?, "steps": [STEP, ...]}. Use only the nodes listed.')


class PlannerUnavailable(RuntimeError):
    pass


def http_chat(endpoint, timeout=30.0, api_key=''):
    """OpenAI-compatible /v1/chat/completions (llama-server). Raises on any network or HTTP error."""
    def chat(system, prompt):
        body = json.dumps({'messages': [{'role': 'system', 'content': system},
                                        {'role': 'user', 'content': prompt}],
                           'temperature': 0.2, 'max_tokens': 600}).encode()
        req = urllib.request.Request(endpoint.rstrip('/') + '/v1/chat/completions', body,
                                     {'Content-Type': 'application/json',
                                      **({'Authorization': f'Bearer {api_key}'} if api_key else {})})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)['choices'][0]['message']['content']
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
    def __init__(self, chat, listing, examples=()):
        self.chat, self.listing, self.examples = chat, listing, list(examples)

    def propose(self, task, feedback=()):
        try:
            text = self.chat(GRAMMAR, planner_prompt(task, self.listing, feedback, self.examples))
        except Exception as e:  # noqa: BLE001  any transport failure = unreachable
            raise PlannerUnavailable(f'{type(e).__name__}: {e}') from e
        return _json_only(text)


def template_lesson(obs):
    """Deterministic lesson from Observable data only. Works offline."""
    if obs.validator_errors:
        return f'The validator rejected the plan: {obs.validator_errors[0]}'
    o = obs.outcome
    if o.get('result') == 'FAILURE':
        return f"{o.get('failed_node') or 'a node'} failed ({o.get('message') or 'no message'}); choose another approach for that step"
    return 'The run succeeded but did not reach the goal; check the destination'


class Reflector:
    """One short lesson per failed attempt. LLM when reachable and the reply lints clean, else template."""

    def __init__(self, chat=None):
        self.chat = chat

    def lesson(self, task, obs):
        base = template_lesson(obs)
        if self.chat:
            try:
                text = self.chat('Write one short sentence: a lesson for a robot planner. No numbers, '
                                 'coordinates or ids.', f'Task: {task}\n{obs.context_text()}').strip().splitlines()[0]
                if text and not lint(text):
                    return text
            except Exception:  # noqa: BLE001
                pass
        return base
