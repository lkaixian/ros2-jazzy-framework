import json
import os

import pytest

from bt_common.catalog import Catalog, format_listing
from bt_slm import dataset, finetune
from bt_slm.fallback import build_xml
from bt_slm.prompt import extract_xml, normalize_spec
from bt_validator.validate import validate_xml

HERE = os.path.dirname(__file__)
CFG = os.path.normpath(os.path.join(HERE, '..', '..'))


@pytest.fixture(scope='module')
def cat():
    import yaml
    c = Catalog(core_file=os.path.join(CFG, 'bt_common', 'config', 'core_nodes.yaml'))
    for p in dataset.EXTENSION_PACKAGES:
        with open(os.path.join(CFG, p, 'config', p + '.yaml')) as f:
            c.add_manifest_dict(yaml.safe_load(f))
    return c


def rows(cat, n=400, seed=1):
    return list(dataset.generate(cat, n, seed))


def test_every_completion_validates(cat):
    for r in rows(cat):
        assert validate_xml(r['completion'], cat) == []


def test_deterministic_and_seed_sensitive(cat):
    assert rows(cat, 50, 7) == rows(cat, 50, 7)
    assert rows(cat, 50, 7) != rows(cat, 50, 8)


def test_repair_rows_carry_real_validator_errors(cat):
    rep = [r for r in rows(cat) if r['kind'] == 'repair']
    assert len(rep) > 40
    for r in rep:
        prev = r['prompt'].split('### PREVIOUS_XML\n')[1].split('\n### ERRORS\n')[0]
        errs = r['prompt'].split('### ERRORS\n')[1].split('\n### BT_XML')[0].split('\n')
        assert validate_xml(prev, cat), 'previous XML must actually be invalid'
        assert errs == validate_xml(prev, cat)


def test_prompt_listing_matches_inference_format(cat):
    gen = [r for r in rows(cat) if r['kind'] == 'generate']
    full = format_listing(cat.all().values())
    assert any(('### NODES\n' + full + '\n### TASK_SPEC') in r['prompt'] for r in gen)
    assert full == cat.prompt_listing()


def test_coverage_of_extension_nodes(cat):
    text = ' '.join(r['completion'] for r in rows(cat, 800))
    for node in ('NavigateTo', 'MoveArmToPose', 'IsPersonWaving', 'HoldHeading', 'DetectObject'):
        assert f'<{node}' in text, node


def test_blackboard_outputs_are_generated(cat):
    assert any('out_bbox="{bbox}"' in r['completion'] for r in rows(cat, 1500))


def test_normalize_spec_is_whitespace_independent():
    a = normalize_spec('{ "goal": "x",\n  "steps": [ ] }')
    assert a == normalize_spec('{"goal":"x","steps":[]}') == '{"goal":"x","steps":[]}'


def test_spec_in_prompt_is_normalized(cat):
    for r in rows(cat, 30):
        spec = r['prompt'].split('### TASK_SPEC\n')[1].split('\n')[0]
        assert spec == normalize_spec(spec)


def test_corrupt_never_returns_valid_xml_silently(cat):
    import random
    rng = random.Random(0)
    xml = build_xml(json.dumps({'goal': 'g', 'steps': [
        {'node': 'DetectObject', 'ports': {'label': 'cup'}}]}))
    for _ in range(50):
        bad = dataset.corrupt(xml, rng)
        assert bad is None or bad != xml


class FakeTok:
    eos_token_id = 99

    def __call__(self, text, add_special_tokens=True, **_):
        ids = [ord(c) % 90 for c in text]
        return {'input_ids': ([1] if add_special_tokens else []) + ids}


def test_encode_masks_prompt_and_appends_eos():
    ids, lab = finetune.encode(FakeTok(), 'ab', 'xyz', 100)
    assert ids[-1] == 99 and lab[-1] == 99
    n_prompt = 3                                   # BOS + 2 chars
    assert lab[:n_prompt] == [-100] * n_prompt and lab[n_prompt:] == ids[n_prompt:]
    assert len(ids) == len(lab)


def test_encode_drops_overlong_instead_of_truncating():
    assert finetune.encode(FakeTok(), 'a' * 10, 'b' * 10, 15) is None


def test_extract_xml_stops_at_root_end():
    assert extract_xml('<root>x</root> garbage </root>') == '<root>x</root>'


def test_inference_prompt_equals_training_prompt(cat):
    """Train/inference skew guard: rebuild each dataset prompt the way the SLM node does."""
    from bt_slm.prompt import prompt_for
    full = cat.prompt_listing()
    checked = 0
    for r in rows(cat, 300, 5):
        if ('### NODES\n' + full + '\n### TASK_SPEC') not in r['prompt']:
            continue                                   # subsetted listing: not comparable
        spec = r['prompt'].split('### TASK_SPEC\n')[1].split('\n###')[0].rstrip('\n')
        pretty = json.dumps(json.loads(spec), indent=2)        # what an LLM might send
        if r['kind'] == 'generate':
            assert prompt_for(pretty, cat) == r['prompt']
        else:
            prev = r['prompt'].split('### PREVIOUS_XML\n')[1].split('\n### ERRORS\n')[0]
            errs = r['prompt'].split('### ERRORS\n')[1].split('\n### BT_XML')[0].split('\n')
            assert prompt_for(pretty, cat, prev, errs) == r['prompt']
        checked += 1
    assert checked > 100


def test_dataset_uses_the_users_detector_labels(cat):
    labels = ['red_cup', 'screwdriver', 'gear_box']
    rs = list(dataset.generate(cat, 400, 3, labels=labels))
    used = set()
    import re
    for r in rs:
        assert validate_xml(r['completion'], cat, vocab={('DetectObject', 'label'): set(labels)}) == []
        used |= set(re.findall(r'<DetectObject label="([^"]+)"', r['completion']))
    assert used and used <= set(labels)
    assert not (used & set(dataset.LABELS))          # no stale default labels leak in


def test_default_model_path_finds_the_repo_models_folder(tmp_path):
    from bt_slm.slm_node import default_model_path
    deep = tmp_path / 'repo' / 'src' / 'bt_slm' / 'bt_slm'
    deep.mkdir(parents=True)
    f = deep / 'slm_node.py'
    f.write_text('')
    assert default_model_path(str(f), {}) == ''                                     # no model: rules only
    m = tmp_path / 'repo' / 'models' / 'bt-slm'
    m.mkdir(parents=True)
    (m / 'config.json').write_text('{}')
    assert default_model_path(str(f), {}) == str(m)
    assert default_model_path(str(f), {'ROBOCUP_SLM_MODEL': '/x/y'}) == '/x/y'     # the env var wins


@pytest.mark.parametrize('profile,low', [('heavy', 12), ('max', 30)])
def test_heavy_profiles_make_long_valid_trees(cat, profile, low):
    rows = list(dataset.generate(cat, 12, seed=3, repair_frac=0, profile=profile))
    sizes = [r['completion'].count('<') for r in rows]
    assert all(not validate_xml(r['completion'], cat) for r in rows)
    assert sum(sizes) / len(sizes) > low, sizes
