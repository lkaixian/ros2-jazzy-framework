#!/usr/bin/env python3
"""WICL benchmark in the 2D kinematic nav sim. Compares, on held-out PARAPHRASES of the training tasks:
  (a) no memory: planner retried with raw validator/sim feedback (what the pipeline does today)
  (b) memory retrieval: examples (no lessons) injected, a re-validated memory is tried first
  (c) memory + reflection: examples WITH lessons, and a lesson written after every failed attempt
Sim-verified only (kinematic sim, not Nav2, not hardware). The LLM is whatever --endpoint serves.
usage: wicl_bench.py --endpoint http://127.0.0.1:8089 [--attempts 3] [--out docs/wicl-benchmark.json]
"""
import argparse
import json
import statistics
import tempfile
import time

from bt_memory.retrieve import first_candidate, format_examples, retrieve
from bt_memory.schema import MemoryStore
from bt_rehearsal.llm import LlmPlanner, Reflector, http_chat
from bt_rehearsal.loop import rehearse, to_memory
from bt_rehearsal.sim import Task
from bt_rehearsal.wiring import make_env

# (training phrasing, evaluation paraphrase, goal place the robot must end at)
TASKS = [
    ('go to the door', 'head over to the doorway', 'door'),
    ('go to the left room', 'please drive into the room on the left', 'left_room'),
    ('go to the right room', 'move to the right-hand room', 'right_room'),
    ('go to the far corner', 'travel to the farthest corner of the map', 'far_corner'),
    ('go back to the start', 'return to where you began', 'start'),
    ('go to the door then the right room', 'pass through the door and end up in the right room', 'right_room'),
    ('visit the left room then the far corner', 'first the left room, after that the far corner', 'far_corner'),
    ('go to the far corner then back to the start', 'drive to the far corner and then come home to the start', 'start'),
    ('go to the door then the left room', 'via the doorway reach the left room', 'left_room'),
    ('go to the right room then the door', 'enter the right room, then come back to the door', 'door'),
    ('go to the door, the right room, then the far corner', 'door first, then right room, finish at the far corner', 'far_corner'),
    ('go to the left room then the start', 'visit the left room and return to the starting point', 'start'),
]


def attempt_stats(task, planner, env, reflector, max_attempts, memories_dir, use_memory, with_lessons):
    """Returns (success, attempts, first_try_valid)."""
    env['sim'].reset()
    if use_memory:
        hits = retrieve(MemoryStore(memories_dir), task.text, env['sim'].scene(), 'kobuki', k=3, min_score=0.3)
        if with_lessons:
            planner.examples = format_examples(hits)
        else:   # examples without the lessons line
            planner.examples = [e.split('\nLessons:')[0] for e in format_examples(hits)]
        cand = first_candidate(hits, env['validate'])
        if cand:
            obs, priv = env['sim'].run(cand.xml)
            if env['sim'].score(task, obs, priv):
                return True, 1, True
    r = rehearse(task, planner, env['build'], env['validate'], env['sim'], reflector, max_attempts)
    return r.success, r.attempts, r.validated_first_try


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--endpoint', required=True)
    ap.add_argument('--attempts', type=int, default=3)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    env = make_env(scene={'objects': [], 'num_persons': 0})
    chat = http_chat(a.endpoint, timeout=60)
    mem_dir = tempfile.mkdtemp(prefix='wicl_bench_')
    store = MemoryStore(mem_dir)

    # learn: rehearse every TRAINING phrasing with reflection, store the successes
    learned = 0
    for train, _, goal in TASKS:
        t = Task(train, goal)
        r = rehearse(t, LlmPlanner(chat, env['listing']), env['build'], env['validate'], env['sim'], Reflector(chat), a.attempts)
        if r.success:
            store.save(to_memory(t, r, 'kobuki', '0.1.0', 'bench'))
            learned += 1
    print(f'learned {learned}/{len(TASKS)} memories')

    arms = {'a_no_memory': (False, False, None), 'b_memory': (True, False, None), 'c_memory_reflection': (True, True, Reflector(chat))}
    report = {'tasks': len(TASKS), 'memories_learned': learned, 'max_attempts': a.attempts, 'arms': {}}
    for name, (use_mem, lessons, refl) in arms.items():
        rows = []
        for _, ev, goal in TASKS:
            pl = LlmPlanner(chat, env['listing'])
            rows.append(attempt_stats(Task(ev, goal), pl, env, refl, a.attempts, mem_dir, use_mem, lessons))
        succ = [n for ok, n, _ in rows if ok]
        report['arms'][name] = {
            'sim_success_rate': round(sum(ok for ok, _, _ in rows) / len(rows), 3),
            'first_plan_validate_rate': round(sum(v for _, _, v in rows) / len(rows), 3),
            'mean_attempts_to_success': round(statistics.mean(succ), 2) if succ else None}
        print(name, report['arms'][name])
    report['note'] = 'kinematic 2D sim, one run per arm, single LLM sample per call: differences of 1 task (8 pts) are noise'
    if a.out:
        json.dump(report, open(a.out, 'w'), indent=2)


if __name__ == '__main__':
    t0 = time.time()
    main()
    print(f'{time.time() - t0:.0f}s')
