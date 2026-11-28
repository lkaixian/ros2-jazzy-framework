import importlib.util
import os

import pytest

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch')


def preset(name):
    """The `preset` dict a task launch file passes to task_launch (captured instead of launching)."""
    import bt_common.launch_utils as lu
    got = {}
    real, lu.task_launch = lu.task_launch, lambda p: got.update(p)
    try:
        spec = importlib.util.spec_from_file_location(name, os.path.join(LAUNCH, name + '.launch.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.generate_launch_description()
    finally:
        lu.task_launch = real
    return got


TASKS = ['robocup', 'gpsr', 'receptionist', 'help_me_carry', 'storing_groceries', 'clean_the_table', 'egpsr', 'restaurant',
         'give_me_a_hand', 'inspection', 'edu_luggage', 'edu_mates', 'edu_reception']


def test_gpsr_reads_the_plan_aloud_and_has_no_face_or_arm():
    p = preset('gpsr')
    assert p['enable_audio'] == 'true' and 'announce_plans' not in p      # announce_plans defaults true = "demonstrate the plan"
    assert p['enable_face'] == p['enable_arm'] == 'false' and p['enable_object'] == 'true'


def test_receptionist_has_faces_and_no_plan_read_out():
    p = preset('receptionist')
    assert p['enable_face'] == 'true' and p['announce_plans'] == 'false' and p['enable_arm'] == 'false'


def test_robocup_starts_everything():
    assert all(v == 'true' for k, v in preset('robocup').items() if k.startswith('enable_'))


def test_arm_tasks_start_the_arm_and_the_arm_eye_and_the_inspection_nothing_else():
    for n in ('storing_groceries', 'clean_the_table', 'give_me_a_hand', 'egpsr'):
        assert preset(n)['enable_arm'] == preset(n)['enable_oak'] == 'true', n
    p = preset('inspection')
    off = ('enable_object', 'enable_person', 'enable_arm', 'enable_audio')
    assert p['enable_navigation'] == 'true' and not any(p[k] == 'true' for k in off)


@pytest.mark.parametrize('name', TASKS)
def test_every_switch_is_declared_by_the_framework_launch_files(name):
    files = ('framework.launch.py', 'perception.launch.py', 'extensions.launch.py', 'audio.launch.py', 'sensors.launch.py',
             'pipeline.launch.py')
    src = ''.join(open(os.path.join(LAUNCH, f)).read() for f in files)
    for k in preset(name):
        assert k in src or (k.startswith('enable_') and f"'{k[7:]}'" in src), (name, k)


@pytest.mark.parametrize('name', TASKS)
def test_each_launch_file_documents_its_gaps(name):
    assert name == 'robocup' or 'GAPS' in open(os.path.join(LAUNCH, name + '.launch.py')).read()


def test_edu_tasks_start_only_on_the_spoken_signal_and_publish_a_valid_spec():
    import json
    for n, macro in (('edu_luggage', 'luggage_round'), ('edu_mates', 'mates_round'), ('edu_reception', 'hric_round')):
        p = preset(n)
        assert 'start the task' in p['start_phrase'] and p['enable_audio'] == 'true', n
        spec = json.loads(p['start_spec'])
        assert spec['steps'] == [{'macro': macro, 'args': {}}], n
        assert not __import__('re').search(r'\b(take|bring|carry|pick|hand|open)\b', spec['goal']), 'a goal the no-arm policy would refuse'
    # the luggage and reception plans name arm nodes (inside Fallbacks): the validator refuses a plan whose nodes no running extension advertises, so arm_extension must run (live: false keeps it dry)
    assert preset('edu_luggage')['enable_arm'] == preset('edu_reception')['enable_arm'] == 'true'
    assert preset('edu_mates')['enable_face'] == preset('edu_reception')['enable_face'] == 'true' and preset('edu_luggage')['enable_face'] == 'false'
