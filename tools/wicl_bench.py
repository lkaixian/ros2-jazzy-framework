#!/usr/bin/env python3
"""WICL benchmark in the 2D kinematic nav sim. Compares, on held-out PARAPHRASES of the training tasks:
  (a) no memory: planner retried with raw validator/sim feedback (what the pipeline does today)
  (b) memory retrieval: examples (no lessons) injected, a re-validated memory is tried first
  (c) memory + reflection: examples WITH lessons, and a lesson written after every failed attempt
Sim-verified only (kinematic sim, not Nav2, not hardware). The LLM is whatever --endpoint serves.
Two splits: REUSE (paraphrases of rehearsed tasks) and HELDOUT (new goal compositions never rehearsed and never
used while tuning). Each run repeats learn+eval; spread is reported.
usage: wicl_bench.py --endpoint URL [--reflector-endpoint URL2] [--runs 5] [--attempts 3] [--out docs/wicl-benchmark.json]
"""
import argparse
import filecmp
import os
import sys
import json
import statistics
import tempfile
import time

from bt_memory.retrieve import first_candidate, format_examples, retrieve
from bt_memory.schema import MemoryStore
from bt_rehearsal.lessonbook import LessonBook
from bt_rehearsal.llm import LlmPlanner, Reflector, anthropic_chat, http_chat
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


HELDOUT = [
    ('go to the far corner then the left room', 'left_room'),
    ('visit the right room then go to the start', 'start'),
    ('go to the left room then the door then the right room', 'right_room'),
    ('go to the start then the far corner', 'far_corner'),
    ('go to the door, then the far corner', 'far_corner'),
    ('go to the right room, the far corner and then the left room', 'left_room'),
    ('go to the left room then the right room', 'right_room'),
    ('go to the far corner then the door', 'door'),
]


# ALIAS tasks: the user names a place the planner must map to a real one (door, left_room, right_room, far_corner,
# start). Written before any run on them; never tuned against. Rehearsed aliases differ from evaluated aliases, so the
# task-specific mapping learned in rehearsal is the only thing that can help; a generic example cannot teach it.
HARD_TRAIN = [('go to the doorway', 'door'), ('go to the west room', 'left_room'), ('go to the east room', 'right_room'),
              ('go to the far end of the map', 'far_corner'), ('go to where you started', 'start')]
HARD_EVAL = [('go to the entrance between the two rooms', 'door'), ('go to the room in the west', 'left_room'),
             ('go to the room in the east', 'right_room'), ('go to the farthest corner from the start', 'far_corner'),
             ('go to your starting spot', 'start'), ('go to the west room then the doorway', 'door'),
             ('go to the east room then the far end of the map', 'far_corner'),
             ('go to the doorway then the room in the west', 'left_room')]
FIXED_EXAMPLE = ['Task: go to the door\nPlan: NavigateToPlace(place=door)']


def check_install():
    """The packages install by copy: refuse to benchmark code that differs from the source tree."""
    import bt_memory
    import bt_rehearsal
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
    for mod in (bt_memory, bt_rehearsal):
        inst = os.path.dirname(mod.__file__)
        src = os.path.join(root, mod.__name__, mod.__name__)
        for f in os.listdir(src):
            if f.endswith('.py') and not filecmp.cmp(os.path.join(src, f), os.path.join(inst, f), shallow=False):
                sys.exit(f'STALE INSTALL: {mod.__name__}/{f} differs from src/. Run colcon build --packages-select {mod.__name__}')


def attempt_stats(task, planner, env, reflector, max_attempts, memories_dir, use_memory, with_lessons, hint=''):
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
    r = rehearse(task, planner, env['build'], env['validate'], env['sim'], reflector, max_attempts, hint=hint)
    return r.success, r.attempts, r.validated_first_try


def mk_planner(a, env, chat, examples=()):
    return LlmPlanner(chat, env['listing'], examples, env['knowledge'] if a.prompt == 'production' else '')


def mk_refl(a, refl_chat):
    return LessonBook(a.lessons_file) if a.lessons_file else Reflector(refl_chat)


def load_external(path):
    d = json.load(open(path))
    places = {'start', 'left_room', 'door', 'right_room', 'far_corner'}
    out = {}
    for k in ('train', 'eval'):
        items = d[k]
        bad = [i for i in items if i.get('goal') not in places or not str(i.get('task', '')).strip()]
        if bad:
            sys.exit(f'{path}: {k} has items with a missing task or unknown goal: {bad[:3]}')
        out[k] = [(i['task'].strip(), i['goal']) for i in items]
    return out


def one_run(a, env, chat, refl_chat):
    mem_dir = tempfile.mkdtemp(prefix='wicl_bench_')
    store, hint = MemoryStore(mem_dir), env['hint']
    learned = 0
    ext = {os.path.basename(f)[len('wicl-tasks-'):-len('.json')]: load_external(f) for f in a.tasks_file}
    want = {x for x in a.only_arms.split(',') if x}
    need_memory = not want or want & {'b', 'c'}        # memories are only built when an arm uses them
    # learn: rehearse every TRAINING phrasing with reflection
    for train, goal in ([(t, g) for t, _, g in TASKS] + HARD_TRAIN + [x for e in ext.values() for x in e['train']]
                        if need_memory else []):
        t = Task(train, goal)
        r = rehearse(t, mk_planner(a, env, chat), env['build'], env['validate'], env['sim'],
                     mk_refl(a, refl_chat), a.attempts, hint=hint)
        if r.success:
            store.save(to_memory(t, r, 'kobuki', '0.1.0', 'bench'))
            learned += 1
    arms = {'a_no_memory': (False, False, None), 'd_fixed_example': (False, False, None),
            'e_lessons_only': (False, False, mk_refl(a, refl_chat)),
            'b_memory': (True, False, None),
            'c_memory_reflection': (True, True, mk_refl(a, refl_chat))}
    splits = {'reuse': [(ev, g) for _, ev, g in TASKS], 'heldout': HELDOUT, 'alias': HARD_EVAL}
    for name, e in ext.items():
        splits[f'external_{name}'] = e['eval']
    if want:
        arms = {k: v for k, v in arms.items() if k.split('_')[0] in want}
    prefixes = tuple(x for x in a.only_splits.split(',') if x)
    if prefixes:
        splits = {k: v for k, v in splits.items() if k.startswith(prefixes)}
    out = {'learned': learned}
    for sname, tasks in splits.items():
        for name, (use_mem, lessons, refl) in arms.items():
            rows = [attempt_stats(Task(t, g), mk_planner(a, env, chat, FIXED_EXAMPLE if name == 'd_fixed_example' else ()),
                                  env, refl, a.attempts, mem_dir,
                                  use_mem, lessons, hint) for t, g in tasks]
            out[f'{sname}/{name}'] = (sum(ok for ok, _, _ in rows) / len(rows),
                                      [n for ok, n, _ in rows if ok])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--endpoint', required=True)
    ap.add_argument('--reflector-endpoint', default='', help='separate (e.g. larger) model for lessons')
    ap.add_argument('--reflector-anthropic-key-file', default='', help='use the Anthropic API for lessons (key read from this file)')
    ap.add_argument('--reflector-model', default='claude-haiku-5-5')
    ap.add_argument('--lessons-file', default='', help='lessons by situation key (see tools/wicl_situations.py)')
    ap.add_argument('--prompt', choices=('production', 'bare'), default='production',
                    help='production = param/llm_knowledge.md + schema (what llm_bridge uses); bare = grammar only')
    ap.add_argument('--tasks-file', nargs='+', default=[], help='JSON {"train": [{task, goal}], "eval": [...]} written by someone else '
                    '(docs/wicl-task-writer-prompt.md)')
    ap.add_argument('--only-arms', default='', help='comma list of arm letters to run, e.g. a,e (skips learning if none needs memory)')
    ap.add_argument('--only-splits', default='', help='comma list of split-name prefixes, e.g. external')
    ap.add_argument('--attempts', type=int, default=3)
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    check_install()
    env = make_env(scene={'objects': [], 'num_persons': 0})
    chat = http_chat(a.endpoint, timeout=60, schema=env['schema'])
    refl_chat = (anthropic_chat(a.reflector_anthropic_key_file, a.reflector_model) if a.reflector_anthropic_key_file
                 else http_chat(a.reflector_endpoint, timeout=60) if a.reflector_endpoint else chat)
    runs = []
    for i in range(a.runs):
        runs.append(one_run(a, env, chat, refl_chat))
        print(f'run {i + 1}/{a.runs}:', {k: (round(v[0], 2) if isinstance(v, tuple) else v) for k, v in runs[-1].items()}, flush=True)
    report = {'runs': a.runs, 'max_attempts': a.attempts, 'constrained_decoding': True, 'prompt': a.prompt, 'lessons_file': a.lessons_file,
              'separate_reflector': bool(a.reflector_endpoint or a.reflector_anthropic_key_file),
              'reflector_model': a.reflector_model if a.reflector_anthropic_key_file else 'same as planner',
              'memories_learned': [r['learned'] for r in runs], 'results': {}}
    for k in runs[0]:
        if k == 'learned':
            continue
        rates = [r[k][0] for r in runs]
        att = [n for r in runs for n in r[k][1]]
        report['results'][k] = {'success_mean': round(statistics.mean(rates), 3), 'success_min': round(min(rates), 3),
                                'success_max': round(max(rates), 3),
                                'mean_attempts_to_success': round(statistics.mean(att), 2) if att else None}
        print(k, report['results'][k])
    if a.out:
        json.dump(report, open(a.out, 'w'), indent=2)


if __name__ == '__main__':
    t0 = time.time()
    main()
    print(f'{time.time() - t0:.0f}s')
