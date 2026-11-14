"""The GPSR-style command generator (tools/gpsr/commands.py): every gold plan must meet its own `expect`, expand, and validate."""
import importlib.util
import json
import os

import pytest
import yaml
from bt_common.catalog import Catalog
from bt_common.macros import expand, load_macros, load_whitelist
from bt_slm.fallback import build_xml
from bt_validator.validate import validate_xml

ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..')
PARAM = os.path.join(ROOT, 'src', 'framework_bringup', 'param')
spec = importlib.util.spec_from_file_location('gpsr_commands', os.path.join(ROOT, 'tools', 'gpsr', 'commands.py'))
gc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gc)
LIB = load_macros(os.path.join(PARAM, 'macros.yaml'))
WL = load_whitelist(os.path.join(PARAM, 'macros.yaml'))
ITEMS = gc.generate(400, seed=5)


@pytest.fixture(scope='module')
def cat():
    c = Catalog()
    src = os.path.join(ROOT, 'src')
    for pkg in os.listdir(src):
        cfg = os.path.join(src, pkg, 'config', pkg + '.yaml')
        if pkg.endswith('_extension') and os.path.exists(cfg):
            c.add_manifest_dict(yaml.safe_load(open(cfg)))
    return c


def ex(s):
    return expand(s, LIB, WL)


def test_generation_is_deterministic_and_varied():
    assert gc.generate(50, seed=1) == gc.generate(50, seed=1) and gc.generate(50, seed=1) != gc.generate(50, seed=2)
    assert len({i['command'] for i in ITEMS}) > 150                      # many different sentences
    assert {i['category'] for i in ITEMS} == {'navigation', 'people', 'vision', 'refuse'}
    json.dumps(ITEMS)                                                    # plain JSON for the JSONL file


def test_every_gold_plan_meets_its_expectation():
    bad = [(i['command'], i['expect']) for i in ITEMS if not gc.check(i['gold'], i['expect'], ex)]
    assert not bad, bad[:3]


def test_every_gold_plan_validates_against_the_real_catalog(cat):
    for i in ITEMS:
        xml = build_xml(json.dumps(ex(i['gold'])))
        assert validate_xml(xml, cat, max_nodes=160) == [], i['command']


def test_checker_rejects_wrong_plans():
    nav = next(i for i in ITEMS if i['expect'].get('visit') == ['door'] and 'macro' not in i['expect'])
    wrong = {'goal': 'x', 'steps': [{'macro': 'go_to_place', 'args': {'place': 'start'}}]}
    assert not gc.check(wrong, nav['expect'], ex)
    refuse = next(i for i in ITEMS if i['expect'].get('refuse'))
    drives = {'goal': 'x', 'steps': [{'macro': 'go_and_say', 'args': {'place': 'door', 'text': 'ok'}}]}
    assert not gc.check(drives, refuse['expect'], ex)                    # a refusal must not drive anywhere
    assert not gc.check({'goal': 'x', 'steps': [{'macro': 'nope'}]}, nav['expect'], ex)   # unexpandable plan = wrong
    order = {'visit': ['a', 'b']}
    two = {'goal': 'x', 'steps': [{'node': 'NavigateToPlace', 'ports': {'place': 'b'}},
                                  {'node': 'NavigateToPlace', 'ports': {'place': 'a'}}]}
    assert not gc.check(two, order, lambda s: s)                         # order matters
