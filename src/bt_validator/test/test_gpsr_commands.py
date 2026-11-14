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


def test_handwritten_commands_are_well_formed():
    """tools/gpsr/handwritten.jsonl (scored by `llm_eval.py --jsonl`): known categories and `expect` keys, unique ids and sentences."""
    rows = [json.loads(line) for line in open(os.path.join(ROOT, 'tools', 'gpsr', 'handwritten.jsonl')) if line.strip()]
    keys = {'visit', 'says', 'macro', 'node', 'detects', 'any_node', 'laps', 'refuse'}
    assert len(rows) >= 30 and len({r['id'] for r in rows}) == len(rows) == len({r['command'] for r in rows})
    assert all(r['category'] in {'navigation', 'people', 'vision', 'refuse'} and r['expect'] and set(r['expect']) <= keys for r in rows)
    assert all(m in LIB for r in rows for m in [r['expect'].get('macro')] if m)


def test_no_arm_policy_matches_the_refuse_category():
    """The orchestrator's arm guard (goal text) fires on exactly the commands that expect a refusal, generated and hand-written."""
    from bt_orchestrator.policy import needs_arm
    rows = [{'command': i['command'], 'category': i['category']} for i in gc.generate(600, seed=7)]
    rows += [json.loads(line) for line in open(os.path.join(ROOT, 'tools', 'gpsr', 'handwritten.jsonl')) if line.strip()]
    bad = [(r['command'], r['category']) for r in rows if needs_arm(r['command']) != (r['category'] == 'refuse')]
    assert not bad, bad[:5]


def test_no_arm_policy_ignores_nouns_in_restated_goals():
    """The model restates the command as a goal ('Drive back to the start place'); nouns that look like arm verbs must not trigger it."""
    from bt_orchestrator.policy import needs_arm, refuse_no_arm
    for goal in ('Drive back to the start place', 'Go to the place where the guest waits', 'Make a lap between the door and the start',
                 'Take the guest to the door', 'Show me the way to the left room'):
        assert not needs_arm(goal), goal
    assert needs_arm('Place the cup on the table') and needs_arm('Bring me a cup')
    assert refuse_no_arm('{"goal": "Open the door", "steps": []}') and refuse_no_arm('not json') is None


def test_recovery_specs_expand_and_validate(cat):
    """The fixed specs the orchestrator publishes for 'go_home' / 'ask_human' recovery must be valid plans."""
    from bt_orchestrator.mode import GO_HOME, ask_human
    for spec in (GO_HOME, ask_human('FAILURE at DetectPerson')):
        xml = build_xml(json.dumps(ex(spec)))
        assert validate_xml(xml, cat) == [], spec['goal']
