import json

import pytest

from bt_memory.schema import MemoryError_, MemoryStore, TaskMemory, lint, norm_task

XML = '<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sleep msec="1"/></BehaviorTree></root>'


def mem(**kw):
    d = dict(task='Go to the Door!', scene={'objects': ['cup']}, xml=XML, lessons=['use NavigateToPlace'],
             outcome={'success': True, 'attempts': 2}, framework_version='0.1.0', model_version='rules')
    d.update(kw)
    return TaskMemory(**d)


def test_norm():
    assert norm_task('  Go to the Door! ') == 'go to the door'


def test_roundtrip(tmp_path):
    s = MemoryStore(str(tmp_path))
    p = s.save(mem())
    assert s.load(p) == mem()
    assert json.loads(open(p).read())['schema_version'] == 1
    assert s.all() == ([mem()], [])


def test_id_depends_on_embodiment():
    assert mem().id != mem(embodiment='other').id


def test_rejects_failed_rehearsal(tmp_path):
    with pytest.raises(MemoryError_):
        MemoryStore(str(tmp_path)).save(mem(outcome={'success': False}))


@pytest.mark.parametrize('lesson', ['robot was at 1.25, 3.50', 'ground_truth said no', 'seed 4 failed'])
def test_lint_rejects(lesson, tmp_path):
    assert lint(lesson)
    with pytest.raises(MemoryError_):
        MemoryStore(str(tmp_path)).save(mem(lessons=[lesson]))


def test_incompatible_version_rejected(tmp_path):
    s = MemoryStore(str(tmp_path))
    p = s.save(mem())
    d = json.load(open(p))
    d['schema_version'] = 99
    json.dump(d, open(p, 'w'))
    with pytest.raises(MemoryError_, match='incompatible'):
        s.load(p)
    assert s.all()[0] == [] and 'incompatible' in s.all()[1][0]


def test_unknown_field_rejected():
    d = json.loads(mem().to_json())
    d['weights'] = 1
    with pytest.raises(MemoryError_, match='unknown'):
        TaskMemory.from_dict(d)
