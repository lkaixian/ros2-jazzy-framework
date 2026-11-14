from bt_orchestrator.mode import (ACTIVE, INIT, INIT_FAILED, READY, SAFE_STOP, TELEOP, GO_HOME, ModeFsm, RecoveryPolicy, ask_human)
from bt_orchestrator.task_fsm import TaskFsm


def ready_fsm(**kw):
    m = ModeFsm(required=('executor', 'llm'), **kw)
    m.heard('executor')
    m.heard('llm')
    m.tick(0.0)
    return m


def test_init_waits_for_the_required_signals_then_fails_on_timeout():
    m = ModeFsm(required=('executor', 'llm'), init_timeout=10)
    m.heard('executor')
    m.tick(0.0)
    assert m.mode == INIT and m.snapshot(0.0)['missing'] == ['llm'] and not m.snapshot(0.0)['run_ok']
    m.tick(11.0)
    assert m.mode == INIT_FAILED
    m.heard('llm')
    m.tick(12.0)
    assert m.mode == READY and m.snapshot(12.0)['run_ok']


def test_stop_latch_and_battery_make_safe_stop_and_halt_a_running_task():
    m = ready_fsm()
    m.on_task_state('running', 1.0)
    m.tick(1.0)
    assert m.mode == ACTIVE
    m.on_stop_state('latched: bumper 1')
    assert m.tick(2.0) == [('halt', 'stop latch: bumper 1')] and m.mode == SAFE_STOP and not m.snapshot(2.0)['run_ok']
    assert m.tick(3.0) == []                                   # one halt per entry, not one per tick
    assert m.reset(3.0)[0] is False                            # still latched upstream
    m.on_stop_state('armed')
    assert m.reset(3.0)[0] and m.tick(3.0) == [] and m.mode == ACTIVE


def test_battery_critical_clears_when_the_guard_goes_quiet():
    m = ready_fsm(battery_clear=15)
    m.on_battery('critical', 5.0)
    m.tick(6.0)
    assert m.mode == SAFE_STOP
    m.tick(21.0)
    assert m.mode == READY


def test_teleop_takes_over_and_gives_back():
    m = ready_fsm(teleop_hold=1.0)
    m.on_task_state('running', 0.5)
    m.on_teleop(1.0)
    assert m.tick(1.2) == [('halt', 'a human took over')] and m.mode == TELEOP
    m.on_task_state('halted', 1.3)
    m.tick(2.5)
    assert m.mode == READY


def test_mission_budget_halts_once_and_resets_with_the_next_task():
    m = ready_fsm(budget_sec=30)
    m.on_task_state('planning', 10.0)
    m.on_task_state('running', 11.0)
    assert m.tick(20.0) == [] and m.snapshot(20.0)['mission_left_sec'] == 20.0
    assert m.tick(41.0) == [('halt', 'mission time budget used up')] and m.tick(42.0) == []
    m.on_task_state('halted', 43.0)
    assert m.snapshot(43.0)['mission_left_sec'] is None


def test_recovery_table():
    p = RecoveryPolicy()
    assert [p.decide('FAILURE', 'NavigateToPlace', i) for i in range(4)] == ['retry', 'replan', 'go_home', 'abort']
    assert [p.decide('FAILURE', 'DetectPerson', i) for i in range(4)] == ['retry', 'replan', 'ask_human', 'abort']
    assert [p.decide('FAILURE', 'WaitForTranscript', i) for i in range(3)] == ['retry', 'ask_human', 'abort']
    assert [p.decide('FAILURE', 'Speak', i) for i in range(3)] == ['replan', 'ask_human', 'abort']
    assert p.decide('TIMEOUT', '', 0) == 'go_home' and p.decide('TIMEOUT', '', 1) == 'abort'
    assert p.decide('ERROR: x', 'Speak', 0) == 'abort' and p.decide('HALTED', 'Speak', 0) == 'abort'


def test_task_fsm_follows_the_policy_and_its_budget():
    f = TaskFsm(max_repairs=3, policy=RecoveryPolicy())
    f.on_task('go to the door')
    f.on_plan(True)
    f.on_status('running', 'LIVE')
    r = f.on_status('finished', 'FAILURE', 'NavigateToPlace')
    assert r['action'] == 'retry' and f.state == 'repairing' and r['attempt'] == 1
    f.on_plan(True)
    f.on_status('running', 'LIVE')
    assert f.on_status('finished', 'FAILURE', 'NavigateToPlace')['action'] == 'replan'
    f.on_plan(True)
    f.on_status('running', 'LIVE')
    assert f.on_status('finished', 'FAILURE', 'NavigateToPlace')['action'] == 'go_home'
    f.on_plan(True)
    f.on_status('running', 'LIVE')
    assert f.on_status('finished', 'FAILURE', 'NavigateToPlace') is None and f.state == 'failed'


def test_task_fsm_aborts_on_a_crashed_tree_even_with_budget():
    f = TaskFsm(max_repairs=3, policy=RecoveryPolicy())
    f.on_task('x')
    f.on_plan(True)
    f.on_status('running', 'LIVE')
    assert f.on_status('finished', 'ERROR: boom') is None and f.state == 'failed' and 'policy: abort' in f.detail


def test_recovery_specs():
    assert GO_HOME['steps'][0]['macro'] == 'return_home'
    assert 'Please help' in ask_human('FAILURE at DetectPerson')['steps'][0]['args']['text']
