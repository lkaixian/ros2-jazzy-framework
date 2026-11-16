"""Control for wicl_bench: zero-shot vs ONE fixed, unrelated example on the held-out tasks. If the fixed example matches
memory, the benchmark gain is prompt priming, not memory content (it did: 38% vs 100%, docs/wicl.md).
usage: python3 tools/wicl_control.py   (llama-server on http://127.0.0.1:8089; sys.path trick below finds wicl_bench)"""
import sys, statistics
sys.path.insert(0, __import__('os').path.dirname(__file__))
from bt_rehearsal.llm import LlmPlanner, http_chat
from bt_rehearsal.loop import rehearse
from bt_rehearsal.sim import Task
from bt_rehearsal.wiring import make_env
from wicl_bench import HELDOUT
env = make_env(); chat = http_chat('http://127.0.0.1:8089', 60, schema=env['schema'])
FIXED = ['Task: go to the door\nPlan: NavigateToPlace(place=door)']      # one fixed, unrelated-to-heldout example
for name, ex in (('zero-shot', []), ('fixed one-shot example', FIXED)):
    rates, first = [], []
    for _ in range(5):
        rs = [rehearse(Task(t, g), LlmPlanner(chat, env['listing'], ex), env['build'], env['validate'], env['sim'], None, 3) for t, g in HELDOUT]
        rates.append(sum(r.success for r in rs) / len(rs))
        first.append(sum(r.success and r.attempts == 1 for r in rs) / len(rs))
    print(name, 'success', [round(x, 2) for x in rates], 'mean', round(statistics.mean(rates), 2), '| first-try', round(statistics.mean(first), 2))
