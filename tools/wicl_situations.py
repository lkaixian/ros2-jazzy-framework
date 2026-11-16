#!/usr/bin/env python3
"""Collect the distinct failure situations of the 2B planner and write a file you can paste into any chat AI.
usage: python3 tools/wicl_situations.py --endpoint http://127.0.0.1:8089 --out situations.md
Then save the AI's JSON reply as lessons.json and run: wicl_bench.py --lessons-file lessons.json ...
Only Observable-derived text is recorded (no ground truth), so the file is safe to paste outside."""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from bt_rehearsal.lessonbook import Recorder, situations_markdown  # noqa: E402
from bt_rehearsal.llm import LlmPlanner, http_chat  # noqa: E402
from bt_rehearsal.loop import rehearse  # noqa: E402
from bt_rehearsal.sim import Task  # noqa: E402
from bt_rehearsal.wiring import make_env  # noqa: E402
from wicl_bench import HARD_EVAL, HARD_TRAIN, HELDOUT, TASKS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--endpoint', required=True)
    ap.add_argument('--out', default='situations.md')
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--attempts', type=int, default=3)
    ap.add_argument('--prompt', choices=('production', 'bare'), default='production')
    a = ap.parse_args()
    env = make_env()
    chat = http_chat(a.endpoint, timeout=60, schema=env['schema'])
    rec = Recorder()
    tasks = [(t, g) for t, _, g in TASKS] + [(e, g) for _, e, g in TASKS] + HELDOUT + HARD_TRAIN + HARD_EVAL
    for _ in range(a.repeats):
        for t, g in tasks:
            planner = LlmPlanner(chat, env['listing'], (), env['knowledge'] if a.prompt == 'production' else '')
            rehearse(Task(t, g), planner, env['build'], env['validate'], env['sim'], rec, a.attempts, hint=env['hint'])
    open(a.out, 'w').write(situations_markdown(rec.seen))
    print(f'{len(rec.seen)} distinct situations -> {a.out}')


if __name__ == '__main__':
    main()
