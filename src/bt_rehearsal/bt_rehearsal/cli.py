"""`wicl rehearse|list|show|adapt`. Offline by default: with no --endpoint, rehearse reports
planner_unavailable instead of guessing, and list/show/adapt need no network at all."""
import argparse
import sys

from bt_memory.adapter import AdaptError, adapt_memory, load_capabilities
from bt_memory.schema import MemoryStore
from bt_orchestrator.memory_hook import DEFAULT_DIR

from bt_rehearsal.llm import LlmPlanner, Reflector, http_chat
from bt_rehearsal.loop import rehearse, to_memory
from bt_rehearsal.sim import Task
from bt_rehearsal.wiring import FRAMEWORK_VERSION, make_env


def main(argv=None):
    ap = argparse.ArgumentParser(prog='wicl')
    ap.add_argument('--store', default=DEFAULT_DIR)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('rehearse')
    r.add_argument('task')
    r.add_argument('--goal-place', default=None, help='sim scoring only')
    r.add_argument('--endpoint', default='', help='llama-server URL; omit = offline (nothing is planned)')
    r.add_argument('--attempts', type=int, default=4)
    r.add_argument('--timeout', type=float, default=120.0)
    r.add_argument('--no-reflection', action='store_true')
    r.add_argument('--embodiment', default='kobuki')
    r.add_argument('--model-version', default='llm')
    sub.add_parser('list')
    s = sub.add_parser('show')
    s.add_argument('id')
    a = sub.add_parser('adapt')
    a.add_argument('id')
    a.add_argument('capabilities')
    args = ap.parse_args(argv)
    store = MemoryStore(args.store)

    if args.cmd == 'list':
        mems, problems = store.all()
        for m in mems:
            print(f'{m.id}  {m.embodiment:10} {m.task}')
        for p in problems:
            print('SKIPPED', p, file=sys.stderr)
        return 0
    if args.cmd == 'show':
        mems, _ = store.all()
        m = next((m for m in mems if m.id == args.id), None)
        print(m.to_json() if m else f'no memory {args.id}', file=None if m else sys.stderr)
        return 0 if m else 1
    env = make_env()
    if args.cmd == 'adapt':
        mems, _ = store.all()
        m = next((m for m in mems if m.id == args.id), None)
        if not m:
            print(f'no memory {args.id}', file=sys.stderr)
            return 1
        try:
            new = adapt_memory(m, load_capabilities(args.capabilities), env['validate'])
        except AdaptError as e:
            print(f'ADAPT FAILED: {e}', file=sys.stderr)
            return 2
        print(store.save(new))
        return 0
    chat = http_chat(args.endpoint) if args.endpoint else (lambda *_: (_ for _ in ()).throw(ConnectionError('no --endpoint')))
    task = Task(args.task, args.goal_place)
    res = rehearse(task, LlmPlanner(chat, env['listing']), env['build'], env['validate'], env['sim'],
                   None if args.no_reflection else Reflector(chat if args.endpoint else None),
                   args.attempts, args.timeout)
    print(f'{res.status} after {res.attempts} attempt(s)')
    for lesson in res.lessons:
        print(' lesson:', lesson)
    if not res.success:
        return 1
    print(store.save(to_memory(task, res, args.embodiment, FRAMEWORK_VERSION, args.model_version)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
