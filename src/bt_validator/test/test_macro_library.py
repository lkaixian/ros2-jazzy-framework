"""Every shipped macro must expand and yield a tree the validator accepts (no typo'd node/port)."""
import json
import os

import pytest
import yaml
from bt_common.catalog import Catalog
from bt_common.macros import expand, load_macros
from bt_slm.fallback import build_xml
from bt_validator.validate import validate_xml

PARAM = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param')
LIB = load_macros(os.path.join(PARAM, 'macros.yaml'))


@pytest.fixture(scope='module')
def cat():
    c = Catalog()
    src = os.path.join(os.path.dirname(__file__), '..', '..')
    for pkg in os.listdir(src):
        cfg = os.path.join(src, pkg, 'config', pkg + '.yaml')
        if pkg.endswith('_extension') and os.path.exists(cfg):
            c.add_manifest_dict(yaml.safe_load(open(cfg)))
    return c


def test_there_are_macros():
    assert len(LIB) >= 8


@pytest.mark.parametrize('name', sorted(LIB))
def test_macro_expands_to_a_valid_tree(name, cat):
    spec = {'goal': name, 'steps': [{'macro': name, 'args': LIB[name]['example_args']}]}
    xml = build_xml(json.dumps(expand(spec, LIB)))
    assert validate_xml(xml, cat, max_nodes=160) == [], xml   # = pipeline.yaml bt_validator_node.max_nodes


def test_macros_only_name_places_that_exist():
    places = set(yaml.safe_load(open(os.path.join(PARAM, 'places.yaml')))['places'])
    for name, m in LIB.items():
        for v in m['example_args'].values():
            if name in ('go_to_place', 'patrol', 'go_and_say') and v.endswith('room') or v == 'door':
                assert v in places, f'{name} example uses unknown place {v}'


def test_a_macro_call_is_much_shorter_than_its_expansion():
    spec = {'goal': 'g', 'steps': [{'macro': 'go_and_say', 'args': {'place': 'door', 'text': 'Hi'}}]}
    short, long_ = json.dumps(spec), json.dumps(expand(spec, LIB))
    assert len(long_) > 3 * len(short)


def test_whitelist_names_only_existing_macros():
    from bt_common.macros import load_whitelist
    wl = load_whitelist(os.path.join(PARAM, 'macros.yaml'))
    assert wl, 'macros.yaml has no whitelist'
    assert set(wl) <= set(LIB), f'whitelisted but not defined: {sorted(set(wl) - set(LIB))}'
    assert set(LIB) <= set(wl), f'defined but not whitelisted (hidden from the LLM): {sorted(set(LIB) - set(wl))}'


def test_macro_docs_are_up_to_date():
    import importlib.util
    root = os.path.join(os.path.dirname(__file__), '..', '..', '..')
    spec = importlib.util.spec_from_file_location('gen_macro_docs', os.path.join(root, 'tools', 'gen_macro_docs.py'))
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    from bt_common.macros import load_whitelist
    want = gen.render(LIB, load_whitelist(os.path.join(PARAM, 'macros.yaml')))
    have = open(os.path.join(root, 'docs', 'macros.md')).read()
    assert have == want, 'docs/macros.md is stale: run python3 tools/gen_macro_docs.py'


def _port_values(steps):
    for s in steps:
        if s.get('group'):
            yield from _port_values(s['steps'])
        elif s.get('macro'):
            yield from (s.get('args') or {}).items()
        else:
            yield from (s.get('ports') or {}).items()


def test_macro_port_values_are_strings():
    """YAML turns an unquoted yes/no/on/off/null into a bool/None (kind: yes -> True): every port value must be text."""
    raw = yaml.safe_load(open(os.path.join(PARAM, 'macros.yaml')))
    for name, m in (raw.get('macros') or raw).items():
        if not isinstance(m, dict) or 'steps' not in m:
            continue
        for port, value in _port_values(m['steps']):
            assert isinstance(value, str), f'{name}: port {port!r} is {value!r} ({type(value).__name__}): quote it'
