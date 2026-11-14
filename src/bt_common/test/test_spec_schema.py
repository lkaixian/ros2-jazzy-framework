import pytest
from bt_common.catalog import NodeSpec
from bt_common.macros import MacroError, expand, spec_schema

LIB = {'go': {'params': {'place': {'required': True}, 'speed': {'default': '1'}}, 'description': '',
              'steps': [{'node': 'Speak', 'ports': {'text': 'x'}}], 'example_args': {}}}
NODES = [NodeSpec('Speak', 'Action', ['text'], ['voice']), NodeSpec('Seq', 'Control')]


def variants(schema):
    return schema['$defs']['step']['anyOf']


def test_schema_lists_macros_with_their_args_and_leaf_nodes_with_their_ports():
    v = variants(spec_schema(NODES, LIB))
    mac = next(x for x in v if 'macro' in x['properties'])
    assert mac['properties']['macro'] == {'const': 'go'} and mac['required'] == ['macro', 'args']
    assert set(mac['properties']['args']['properties']) == {'place', 'speed'}
    assert mac['properties']['args']['required'] == ['place']           # speed has a default
    leaf = next(x for x in v if 'node' in x['properties'])
    assert leaf['properties']['node'] == {'const': 'Speak'}              # Control nodes are not leaves
    assert set(leaf['properties']['ports']['properties']) == {'text', 'voice'}
    assert sum('node' in x['properties'] for x in v) == 1


def test_no_macros_no_macro_variant():
    assert not any('macro' in x['properties'] for x in variants(spec_schema(NODES, {})))


def test_whitelist_restricts_only_the_spec_not_macro_bodies():
    lib = dict(LIB, outer={'params': {}, 'description': '', 'example_args': {},
                           'steps': [{'macro': 'go', 'args': {'place': 'a'}}]})
    spec = {'goal': 'g', 'steps': [{'macro': 'outer'}]}
    assert expand(spec, lib, allowed=['outer'])['steps']                  # outer calls go: fine
    with pytest.raises(MacroError, match="'go' is not available"):
        expand({'goal': 'g', 'steps': [{'macro': 'go', 'args': {'place': 'a'}}]}, lib, allowed=['outer'])
