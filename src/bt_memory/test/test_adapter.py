import pytest

from bt_memory.adapter import AdaptError, adapt_memory, adapt_xml, load_capabilities
from bt_memory.schema import TaskMemory

XML = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>'
       '<NavigateToPlace place="door"/><Speak text="hi"/></Sequence></BehaviorTree></root>')
CAP = {'embodiment': 'other', 'actions': {'NavigateToPlace': {'name': 'GoToPlace', 'ports': {'place': 'target'},
                                                              'defaults': {'speed': 0.2}},
                                          'Speak': {'name': 'Say', 'ports': {'text': 'message'}}}}


def mem():
    return TaskMemory(task='t', scene={}, xml=XML, outcome={'success': True})


def test_remap():
    out = adapt_xml(XML, CAP)
    assert '<GoToPlace target="door" speed="0.2" />' in out and '<Say message="hi" />' in out
    assert '<Sequence>' in out


def test_missing_action_fails_loudly():
    cap = {'embodiment': 'x', 'actions': {'Speak': {'name': 'Speak'}}}
    with pytest.raises(AdaptError, match='NavigateToPlace'):
        adapt_xml(XML, cap)


def test_adapt_memory_revalidates():
    m = adapt_memory(mem(), CAP, lambda x: [])
    assert m.embodiment == 'other' and m.adapted_from == 'kobuki' and m.id != mem().id
    with pytest.raises(AdaptError, match='fails validation'):
        adapt_memory(mem(), CAP, lambda x: ["port 'speed' unknown"])
    with pytest.raises(AdaptError, match='already'):
        adapt_memory(mem(), {'embodiment': 'kobuki', 'actions': {}}, lambda x: [])


def test_capability_file(tmp_path):
    p = tmp_path / 'c.yaml'
    p.write_text('embodiment: a\nactions:\n  Speak: {}\n')
    with pytest.raises(AdaptError, match='name'):
        load_capabilities(str(p))
    p.write_text('embodiment: a\nactions:\n  Speak: {name: Say}\n')
    assert load_capabilities(str(p))['actions']['Speak']['name'] == 'Say'
