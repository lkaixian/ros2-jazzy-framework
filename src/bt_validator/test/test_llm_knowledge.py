"""Drift guard for param/llm_knowledge.md: what the LLM is told must still exist."""
import os
import re

import pytest
import yaml
from bt_common.catalog import Catalog

PARAM = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param')
KNOWLEDGE = os.path.join(PARAM, 'llm_knowledge.md')


@pytest.fixture(scope='module')
def text():
    return open(KNOWLEDGE).read()


@pytest.fixture(scope='module')
def cat():
    c = Catalog()
    ext = os.path.join(os.path.dirname(__file__), '..', '..')
    for pkg in os.listdir(ext):
        cfg = os.path.join(ext, pkg, 'config', pkg + '.yaml')
        if pkg.endswith('_extension') and os.path.exists(cfg):
            c.add_manifest_dict(yaml.safe_load(open(cfg)))
    return c


def test_every_backticked_node_exists(text, cat):
    names = re.findall(r'`([A-Z][A-Za-z]+)`', text)   # node ids are CamelCase; `waving` is a JSON key
    assert names, 'knowledge file should name the nodes it talks about'
    unknown = [n for n in names if cat.get(n) is None]
    assert not unknown, f'llm_knowledge.md names nodes that do not exist: {unknown}'


def test_places_match_places_yaml(text):
    line = next(l for l in text.splitlines() if l.startswith('- Named places'))
    told = {p.strip() for p in line.split(':', 1)[1].rstrip('.').split(',')}
    told = {re.sub(r'.*\)\s*:\s*', '', p).strip() for p in told}
    real = set(yaml.safe_load(open(os.path.join(PARAM, 'places.yaml')))['places'])
    assert told == real, f'told {sorted(told)} but places.yaml has {sorted(real)}'


def test_stays_small_enough_for_a_prompt(text):
    assert len(text) < 4800          # ~1100 tokens (rules + worked examples): this goes into every planning request


def test_worked_examples_are_valid_plans(text, cat):
    import json

    from bt_common.macros import expand, load_macros
    from bt_slm.fallback import build_xml
    from bt_validator.validate import validate_xml
    lib = load_macros(os.path.join(PARAM, 'macros.yaml'))
    shown = [l for l in text.splitlines() if l.startswith('Example:')]
    assert len(shown) >= 5
    for line in shown:
        steps = json.loads(line.split('->', 1)[1])
        xml = build_xml(json.dumps(expand({'goal': 'example', 'steps': steps}, lib)))
        assert validate_xml(xml, cat) == [], line
