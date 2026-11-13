from bt_orchestrator.task_fsm import TaskFsm, repair_prompt


def start_running(f, task='go'):
    f.on_task(task)
    f.on_plan(True)
    f.on_status('running', 'dry run')


def test_happy_path():
    f = TaskFsm()
    assert f.state == 'idle'
    f.on_task('go')
    assert f.state == 'planning'
    f.on_plan(True)
    assert f.state == 'ready'
    f.on_status('plan_received', 'x')
    assert f.state == 'ready'
    f.on_status('running', 'LIVE')
    assert f.state == 'running'
    assert f.on_status('finished', 'SUCCESS') is None and f.state == 'done'


def test_invalid_plan_fails_planning():
    f = TaskFsm()
    f.on_task('go')
    f.on_plan(False, ['SLM failed: x'])
    assert f.state == 'failed' and 'SLM failed' in f.detail


def test_halt_is_not_repaired():
    f = TaskFsm(max_repairs=3)
    start_running(f)
    assert f.on_status('finished', 'HALTED') is None and f.state == 'halted'


def test_repair_disabled_by_default():
    f = TaskFsm()
    start_running(f)
    assert f.on_status('finished', 'FAILURE', 'DetectPerson') is None
    assert f.state == 'failed' and 'repair disabled' in f.detail


def test_repair_budget_then_failed():
    f = TaskFsm(max_repairs=1)
    f.on_task('find a person')
    f.on_spec('{"goal":"g"}')
    f.on_plan(True)
    f.on_status('running', 'dry run')
    r = f.on_status('finished', 'FAILURE', 'DetectPerson')
    assert r['failed_node'] == 'DetectPerson' and (r['attempt'], r['of']) == (1, 1)
    assert f.state == 'repairing'
    f.on_plan(True)                                  # the re-plan arrived
    assert f.state == 'ready'
    f.on_status('running', 'dry run')
    assert f.on_status('finished', 'FAILURE', 'DetectPerson') is None     # budget spent
    assert f.state == 'failed'


def test_new_task_resets_budget_and_plan_during_run_is_ignored():
    f = TaskFsm(max_repairs=1)
    start_running(f, 'a')
    f.on_plan(True)                                  # a hand-written tree submitted meanwhile
    assert f.state == 'running'
    f.on_status('finished', 'FAILURE', 'N')
    f.on_task('b')
    assert f.repairs == 0 and f.state == 'planning'


def test_finished_without_running_is_ignored():
    f = TaskFsm(max_repairs=1)
    f.on_task('a')
    f.on_plan(True)
    assert f.on_status('finished', 'FAILURE', 'N') is None and f.state == 'ready'


def test_repair_prompt_mentions_where_it_broke():
    p = repair_prompt({'task': 't', 'spec': '{"s":1}', 'failed_node': 'DetectPerson',
                       'result': 'FAILURE', 'attempt': 1, 'of': 2})
    assert 'DetectPerson' in p and 'REPAIR 1/2' in p and '{"s":1}' in p
