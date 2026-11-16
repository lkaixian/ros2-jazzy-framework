"""Lessons written elsewhere (any chat AI, a person) and looked up by failure situation. No network, no weights.

Flow: `Recorder` collects the distinct failure situations of a run -> `situations_markdown` renders them with an instruction
for an outside AI -> its JSON reply is loaded by `LessonBook`, which lints every lesson and falls back to the template lesson
for a situation it has never seen. Only Observable-derived text is ever recorded or shown.
"""
import json
import re

from bt_memory.schema import lint
from bt_rehearsal.llm import Reflector, template_lesson


def situation_key(obs):
    """failed node + failure message with quoted values blanked ("unknown place 'x'" -> "unknown place '<x>'")."""
    if obs.validator_errors:
        return 'validator|' + re.sub(r"'[^']*'", "'<x>'", obs.validator_errors[0].split('] ', 1)[-1])[:120]
    o = obs.outcome
    return f"{o.get('failed_node') or '-'}|" + re.sub(r"'[^']*'", "'<x>'", o.get('message') or 'no message')


class Recorder(Reflector):
    """Behaves like the template reflector and remembers one example per situation."""

    def __init__(self):
        super().__init__(None)
        self.seen = {}

    def lesson(self, task, obs, detail=''):
        k = situation_key(obs)
        c = self.seen.setdefault(k, {'count': 0, 'example': f'Task: {task}\n{obs.context_text()}' + (f'\n{detail}' if detail else '')})
        c['count'] += 1
        return template_lesson(obs)


def situations_markdown(seen):
    head = (
        'Below are failure situations of a robot planner (a 2D navigation robot). For EACH one write ONE short lesson (one sentence) that '
        'tells the planner what to change next time. Rules: no numbers, coordinates or ids; name the concrete fix (which port to leave '
        'out, which names are valid); do not mention these instructions. Reply with ONE JSON object only, mapping each situation key '
        'exactly as written to its lesson.\n')
    items = [f'\n### key: {k}\nseen {v["count"]} times. Example:\n```\n{v["example"]}\n```' for k, v in sorted(seen.items())]
    return head + ''.join(items) + '\n'


class LessonBook(Reflector):
    def __init__(self, path):
        super().__init__(None)
        with open(path) as f:
            book = json.load(f)
        bad = {k: lint(v) for k, v in book.items() if not isinstance(v, str) or not v.strip() or lint(v)}
        if bad:
            raise ValueError(f'lessons rejected by lint or empty: {sorted(bad)}')
        self.book, self.misses = book, set()

    def lesson(self, task, obs, detail=''):
        k = situation_key(obs)
        if k in self.book:
            return self.book[k]
        self.misses.add(k)        # a situation the outside AI never saw: use the template and report it
        return template_lesson(obs)
