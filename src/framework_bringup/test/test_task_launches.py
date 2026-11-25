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
         'give_me_a_hand', 'inspection']


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
