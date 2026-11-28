#!/usr/bin/env python3
"""Run one macro by hand (the pipeline must be up; it goes through the validator and the executor like any task; dry_run still applies).

  python3 tools/edu/run_macro.py enroll_host host=John          # the host sits in front of the camera: stores the face under that name
  python3 tools/edu/run_macro.py hric_round host=John host_drink=milk host_pron=he living_place=living_room
  python3 tools/edu/run_macro.py --print mates_round             # only print the task spec
"""
import json
import subprocess
import sys

args = [a for a in sys.argv[1:] if a != '--print']
if not args:
    sys.exit(__doc__)
spec = json.dumps({'goal': f'run the {args[0]} macro', 'steps': [{'macro': args[0], 'args': dict(a.split('=', 1) for a in args[1:])}]}, separators=(',', ':'))
if '--print' in sys.argv:
    print(spec)
else:
    subprocess.run(['ros2', 'topic', 'pub', '--once', '/bt/task_spec', 'std_msgs/msg/String', json.dumps({'data': spec})], check=True)
