import os

import pytest

from bt_common.catalog import Catalog
from bt_validator.validate import validate_xml

OK = """<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><Sequence>
    <DetectObject label="cup"/><Sleep msec="100"/>
  </Sequence></BehaviorTree></root>"""


@pytest.fixture(scope='module')
def catalog():
    core = os.path.join(os.path.dirname(__file__), '..', '..', 'bt_common', 'config',
                        'core_nodes.yaml')
    c = Catalog(core_file=core)
    c.add_manifest_dict({'extension': 'x', 'nodes': {
        'Go': {'category': 'Action', 'required': ['x']}}})
    return c


def test_valid(catalog):
    assert validate_xml(OK, catalog) == []


def test_extension_node(catalog):
    xml = OK.replace('<Sleep msec="100"/>', '<Go x="1"/>')
    assert validate_xml(xml, catalog) == []


@pytest.mark.parametrize('xml,needle', [
    ('<root', 'not well-formed'),
    (OK.replace('<DetectObject label="cup"/>', '<Fly/>'), 'unknown node <Fly>'),
    (OK.replace('label="cup"', ''), "missing required port 'label'"),
    (OK.replace('label="cup"', 'label="cup" bogus="1"'), "unknown port 'bogus'"),
    (OK.replace('main_tree_to_execute="Main"', 'main_tree_to_execute="Nope"'),
     'does not match'),
    (OK.replace('BTCPP_format="4"', ''), 'BTCPP_format'),
    (OK.replace('<Sequence>', '<Sequence></Sequence><Sequence>'), 'exactly 1 root node'),
    (OK.replace('<Sleep msec="100"/>', '<Inverter/>'), 'exactly 1 child'),
    (OK.replace('<Sleep msec="100"/>', '<Sleep msec="1"><AlwaysSuccess/></Sleep>'),
     'must not have children'),
    (OK.replace('<Sleep msec="100"/>', '<SubTree ID="Gone"/>'), 'not defined'),
    (OK.replace('<Sleep msec="100"/>', '<Sleep msec="{bad"/>'), 'malformed blackboard'),
])
def test_invalid(catalog, xml, needle):
    assert any(needle in e for e in validate_xml(xml, catalog)), validate_xml(xml, catalog)


def test_cycle(catalog):
    xml = """<root BTCPP_format="4" main_tree_to_execute="A">
      <BehaviorTree ID="A"><SubTree ID="B"/></BehaviorTree>
      <BehaviorTree ID="B"><SubTree ID="A"/></BehaviorTree></root>"""
    assert any('cycle' in e for e in validate_xml(xml, catalog))


def test_limits(catalog):
    body = '<AlwaysSuccess/>' * 70
    xml = f'<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>{body}</Sequence></BehaviorTree></root>'
    assert any('limit' in e for e in validate_xml(xml, catalog))
