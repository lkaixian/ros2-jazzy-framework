"""tools/start.sh: what it would start, and that it never switches real motion on by itself (--list starts nothing)."""
import os
import subprocess

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
SCRIPT = os.path.join(ROOT, 'tools', 'start.sh')


def run(*args):
    return subprocess.run(['bash', SCRIPT, *args, '--list'], capture_output=True, text=True, timeout=30)


def test_a_task_is_dry_unless_move_is_asked_for():
    assert 'gpsr.launch.py dry_run:=true' in run('task', 'gpsr').stdout
    assert 'dry_run:=false' not in run('task', 'gpsr').stdout
    assert 'gpsr.launch.py dry_run:=false' in run('task', 'gpsr', '--move').stdout


def test_the_arm_profile_starts_the_camera_detector_and_arm_extension_but_not_the_arm_driver():
    out = run('arm').stdout
    assert 'enable_camera:=true' in out and 'perception.launch.py' in out and 'enable_arm:=true' in out
    assert 'arm_up.sh' in out.split('realsense')[0]            # only as the note that says to run it yourself
    assert 'tools/arm/arm_eye.sh' not in out and 'tools/arm/arm_eye.sh' in run('arm', '--oak').stdout


def test_every_task_launch_it_names_exists_and_an_unknown_task_is_refused():
    for t in 'gpsr egpsr inspection receptionist restaurant help_me_carry storing_groceries clean_the_table give_me_a_hand robocup'.split():
        assert os.path.exists(os.path.join(ROOT, 'src', 'framework_bringup', 'launch', f'{t}.launch.py')), t
        assert f'{t}.launch.py' in run('task', t).stdout
    assert run('task', 'nope').returncode == 2
