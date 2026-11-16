from bt_memory.retrieve import TextSceneRetriever, first_candidate, format_examples, retrieve, summarize_plan
from bt_memory.schema import MemoryStore, TaskMemory

XML = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>'
       '<NavigateToPlace place="door"/><Speak text="hi"/></Sequence></BehaviorTree></root>')


def mem(task, objs=(), emb='kobuki', xml=XML):
    return TaskMemory(task=task, scene={'objects': list(objs)}, xml=xml, lessons=['say it after arriving'],
                      outcome={'success': True}, embodiment=emb)


def test_ranking_and_threshold(tmp_path):
    s = MemoryStore(str(tmp_path))
    for m in (mem('go to the door and say hi'), mem('bring me a cup', ['cup']), mem('go to the door', emb='other')):
        s.save(m)
    hits = retrieve(s, 'Go to the door and say hi!', {'objects': []}, 'kobuki')
    assert [m.task for _, m in hits] == ['go to the door and say hi'] and hits[0][0] == 1.0
    assert retrieve(s, 'dance', {'objects': []}, 'kobuki') == []


def test_scene_overlap_breaks_ties():
    a, b = mem('fetch the item', ['cup']), mem('fetch the item', ['bottle'])
    r = TextSceneRetriever().rank('fetch an item', {'objects': ['bottle']}, [a, b])
    assert r[0][1] is b


def test_summary_and_examples():
    assert summarize_plan(XML) == 'NavigateToPlace(place=door) -> Speak(text=hi)'
    ex = format_examples([(1.0, mem('t'))])
    assert 'Plan: NavigateToPlace' in ex[0] and 'Lessons: say it' in ex[0]


def test_first_candidate_revalidates():
    old, new = mem('t', xml=XML.replace('door', 'gone')), mem('t')
    ok = lambda x: [] if 'gone' not in x else ['unknown place']  # noqa: E731
    assert first_candidate([(1, old), (0.9, new)], ok) is new
    assert first_candidate([(1, old)], ok) is None
