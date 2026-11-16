from bt_memory.schema import MemoryStore, TaskMemory
from bt_orchestrator.memory_hook import MemoryHook

XML = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">'
       '<NavigateToPlace place="door"/></BehaviorTree></root>')


def hook(tmp_path, **kw):
    MemoryStore(str(tmp_path)).save(TaskMemory(task='go to the door', scene={'objects': []}, xml=XML,
                                               lessons=['use a known place'], outcome={'success': True}))
    return MemoryHook(str(tmp_path), **kw)


def test_candidate_when_valid(tmp_path):
    cand, text = hook(tmp_path).on_task('Go to the door', {}, lambda x: [])
    assert cand.xml == XML and 'NavigateToPlace(place=door)' in text and text.startswith('Go to the door')


def test_stale_memory_is_not_a_candidate_but_still_an_example(tmp_path):
    cand, text = hook(tmp_path).on_task('go to the door', {}, lambda x: ['unknown place'])
    assert cand is None and 'Similar tasks' in text


def test_no_match_leaves_task_untouched(tmp_path):
    assert hook(tmp_path).on_task('dance', {}, lambda x: []) == (None, 'dance')


def test_few_shot_only_mode(tmp_path):
    assert hook(tmp_path, use_candidate=False).on_task('go to the door', {}, lambda x: [])[0] is None


def test_empty_store(tmp_path):
    assert MemoryHook(str(tmp_path / 'none')).on_task('x', {}, lambda x: []) == (None, 'x')


def test_report_lists_scores_and_offer(tmp_path):
    h = hook(tmp_path)
    h.on_task('go to the door', {}, lambda x: [])
    assert '1.00' in h.report[0] and '[offered: re-validated]' in h.report[0]
    h.on_task('dance', {}, lambda x: [])
    assert 'no memory scored' in h.report[0]


def test_place_missing_on_current_map_is_not_offered(tmp_path):
    h = hook(tmp_path, places=lambda: {'start', 'kitchen'})        # the map changed: no 'door' any more
    cand, text = h.on_task('go to the door', {}, lambda x: [])
    assert cand is None and 'Similar tasks' in text               # still an example, never the plan
    h.places = lambda: {'door'}
    assert h.on_task('go to the door', {}, lambda x: [])[0] is not None
