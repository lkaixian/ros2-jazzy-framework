from bt_memory.retrieve import unknown_places

XML = '<root><BehaviorTree ID="M"><Sequence><NavigateToPlace place="door"/><NavigateToPlace place="{p}"/>' \
      '<NavigateToPlace place="gone"/></Sequence></BehaviorTree></root>'


def test_unknown_places():
    assert unknown_places(XML, {'door'}) == ["place 'gone' is not in the current places file"]
    assert unknown_places(XML, {'door', 'gone'}) == []
