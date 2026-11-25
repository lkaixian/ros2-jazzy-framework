#!/usr/bin/env python3
"""The arm-task macros (clean_the_table_round, store_groceries_round, dispose_rubbish, give_me_a_hand_round) in a LIVE executor
(dry_run:=false) against fake arm / pose / social / nav extensions (fake_arm_ext.py: real manifests, the real object table and answer parser,
scripted objects and failures), own DDS domain (ARMTASK_DOMAIN, default 91). Checks what the plans DO: the order of calls, that the values
NextObject writes reach PickObject / NavigateToPlace / PlaceObject, loops stopping when nothing is left, a failed pick not stopping the
round, and that the gripper closes on a person only after the voice cue. No robot and no arm are involved.
Workspace sourced: python3 tools/integration/arm_task_test.py"""
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ['ROS_DOMAIN_ID'] = os.environ.get('ARMTASK_DOMAIN', '91')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def spec(goal, macro, **args):
    return {'goal': goal, 'steps': [{'macro': macro, 'args': {k: str(v) for k, v in args.items()}}]}


def main():
    ros = ['--ros-args', '-p']
    cmds = [['ros2', 'run', 'bt_validator', 'bt_validator_node'],
            ['ros2', 'run', 'bt_slm', 'bt_slm_node'] + ros + ['model_path:=none'],
            ['ros2', 'run', 'bt_orchestrator', 'orchestrator_node'] + ros + ['max_repairs:=0'],
            ['ros2', 'run', 'bt_executor', 'bt_executor_node'] + ros + ['dry_run:=false', '-p', 'auto_run:=true', '-p', 'max_runtime_sec:=40.0'],
            [sys.executable, os.path.join(HERE, 'fake_arm_ext.py')]]
    procs = [subprocess.Popen(c, start_new_session=True, stdout=subprocess.DEVNULL) for c in cmds]
    fails = []
    try:
        rclpy.init()
        n = rclpy.create_node('arm_task_test')
        pub = {t: n.create_publisher(String, t, 10) for t in ('bt/task', 'bt/task_spec', 'fake/world')}
        box = {'state': {}, 'calls': []}
        n.create_subscription(String, 'task/state', lambda m: box.update(state=json.loads(m.data)), LATCHED)
        n.create_subscription(String, 'fake/calls', lambda m: box.update(calls=json.loads(m.data)), 10)

        def spin_until(pred, sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.05)
                if pred():
                    return True
            return False

        def check(name, ok):
            print(('ok   ' if ok else 'FAIL ') + name)
            if not ok:
                fails.append(name)

        spin_until(lambda: pub['fake/world'].get_subscription_count() and box['state'], 60)
        spin_until(lambda: False, 8)                       # manifests reach the validator and the executor on the 3 s republish

        def run(sp, world):
            box.update(state={}, calls=[])
            pub['fake/world'].publish(String(data=json.dumps(world)))
            spin_until(lambda: False, 0.6)
            box.update(calls=[])
            pub['bt/task'].publish(String(data=sp['goal']))
            spin_until(lambda: False, 0.3)
            pub['bt/task_spec'].publish(String(data=json.dumps(sp)))
            spin_until(lambda: box['state'].get('state') in ('done', 'failed') and box['state'].get('task') == sp['goal'], 60)
            spin_until(lambda: False, 0.5)
            return box['state'], [(c[0], c[1]) for c in box['calls']]

        def seq(calls, node, **ports):
            return [i for i, (nd, p) in enumerate(calls) if nd == node and all(p.get(k) == v for k, v in ports.items())]

        # clean the table: a cup (-> dishwasher) and a bottle (-> bin); then nothing is left but the loop still ends
        st, calls = run(spec('clean the table in the left room', 'clean_the_table_round', table_place='left_room', rounds=4),
                        {'objects': ['cup', 'bottle']})
        check('clean: plan done', st.get('state') == 'done')
        check('clean: the cup goes to the dishwasher (pick, drive, place with the table values)',
              bool(seq(calls, 'PickObject', label='cup')) and bool(seq(calls, 'NavigateToPlace', place='dishwasher'))
              and bool(seq(calls, 'PlaceObject', place='dishwasher')))
        check('clean: the bottle goes to the dustbin / bin',
              bool(seq(calls, 'PickObject', label='bottle')) and bool(seq(calls, 'NavigateToPlace', place='dustbin'))
              and bool(seq(calls, 'PlaceObject', place='bin')))
        check('clean: pick -> drive -> place in that order for the first item',
              seq(calls, 'PickObject', label='cup')[0] < seq(calls, 'NavigateToPlace', place='dishwasher')[0] < seq(calls, 'PlaceObject', place='dishwasher')[0])
        check('clean: it comes back to the table after each item and ends by saying it finished',
              len(seq(calls, 'NavigateToPlace', place='left_room')) >= 3 and any(c[0] == 'Speak' and 'finished cleaning' in c[1]['text'] for c in calls))
        check('clean: 4 rounds asked, 2 objects handled, no further pick', len(seq(calls, 'PickObject')) == 2)

        # a pick that fails does not stop the round: the next object is still handled
        st, calls = run(spec('clean the table in the left room', 'clean_the_table_round', table_place='left_room', rounds=3),
                        {'objects': ['cup', 'bottle'], 'fail': {'PickObject': 1}})
        check('failed pick: plan still done and the other object handled', st.get('state') == 'done' and len(seq(calls, 'PlaceObject')) >= 1)

        # storing groceries: shelf, with the group said aloud
        st, calls = run(spec('store the groceries from the left room', 'store_groceries_round', table_place='left_room', rounds=3),
                        {'objects': ['apple', 'bottle']})
        check('groceries: both go to the shelf', st.get('state') == 'done' and len(seq(calls, 'PlaceObject', place='shelf')) == 2)
        check('groceries: it says which group each belongs to',
              any(c[0] == 'Speak' and c[1].get('detail') in ('fruit and vegetables', 'drinks') for c in calls))

        # rubbish: one item to the dustbin, nothing else
        st, calls = run(spec('put the rubbish in the dustbin', 'dispose_rubbish', rounds=1), {'objects': ['bottle', 'cup']})
        check('rubbish: one item picked and binned', st.get('state') == 'done' and len(seq(calls, 'PickObject')) == 1
              and len(seq(calls, 'PlaceObject', place='bin')) == 1)
        st, calls = run(spec('put the rubbish in the dustbin', 'dispose_rubbish', rounds=1), {'objects': []})
        check('rubbish: nothing in view -> quietly done, the arm never moves', st.get('state') == 'done' and not seq(calls, 'PickObject'))

        # give me a hand: the gripper opens first and closes ONLY after the voice cue; then the place comes from the answer
        st, calls = run(spec('give me a hand', 'give_me_a_hand_round'), {'answer': 'put it in the dustbin please'})
        check('hand: done and placed in the bin the person named', st.get('state') == 'done' and bool(seq(calls, 'PlaceObject', place='bin')))
        check('hand: open gripper + handover pose BEFORE the voice cue, close only AFTER it',
              seq(calls, 'OpenGripper')[0] < seq(calls, 'MoveArmToPose')[0] < seq(calls, 'WaitForPhrase')[0] < seq(calls, 'CloseGripper')[0])
        st, calls = run(spec('give me a hand', 'give_me_a_hand_round'), {'answer': 'somewhere nice', 'fail': {}})
        check('hand: an answer that names no place -> no placing, plan failed', st.get('state') == 'failed' and not seq(calls, 'PlaceObject'))
        st, calls = run(spec('give me a hand', 'give_me_a_hand_round'), {'fail': {'WaitForPhrase': 1}})
        check('hand: no voice cue -> the gripper never closes', not seq(calls, 'CloseGripper') and st.get('state') == 'failed')
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGTERM)
        rclpy.try_shutdown()
    print('PASS' if not fails else f'FAIL: {fails}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
