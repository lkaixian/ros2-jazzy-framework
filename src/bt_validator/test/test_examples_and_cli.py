import glob
import os

import pytest
import yaml

from bt_common.catalog import Catalog
from bt_validator import cli
from bt_validator.validate import validate_xml

SRC = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
TREES = sorted(glob.glob(os.path.join(SRC, 'framework_bringup', 'behavior_trees', '*.xml')))


@pytest.fixture(scope='module')
def cat():
    c = Catalog(core_file=os.path.join(SRC, 'bt_common', 'config', 'core_nodes.yaml'))
    for cfg in glob.glob(os.path.join(SRC, '*_extension', 'config', '*_extension.yaml')):
        with open(cfg) as f:
            c.add_manifest_dict(yaml.safe_load(f))
    return c


def test_examples_exist():
    assert len(TREES) >= 2


@pytest.mark.parametrize('path', TREES)
def test_example_trees_validate(cat, path):
    assert validate_xml(open(path).read(), cat) == []


def test_examples_use_the_social_nodes(cat):
    text = ' '.join(open(p).read() for p in TREES)
    for n in ('ExtractFaceFeature', 'Speak', 'EnrollFace', 'DetectPerson'):
        assert f'<{n}' in text


def test_breaking_an_example_is_caught(cat):
    xml = open(next(t for t in TREES if t.endswith('greet_person.xml'))).read()
    for broken, needle in [(xml.replace('<Speak ', '<Say '), 'unknown node <Say>'),
                           (xml.replace(' text="Hello {person}"', ''), "missing required port 'text'"),
                           (xml.replace('out_identity="{who}"', 'out_identity="{who"'), 'malformed')]:
        assert any(needle in e for e in validate_xml(broken, cat)), needle


def test_cli_ok_and_invalid(tmp_path, capsys):
    good = tmp_path / 'good.xml'
    good.write_text('''<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">
      <Sequence><Sleep msec="10"/></Sequence></BehaviorTree></root>''')
    bad = tmp_path / 'bad.xml'
    bad.write_text('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Fly/></BehaviorTree></root>')
    assert cli.main([str(good)]) == 0
    assert cli.main([str(good), str(bad)]) == 1
    out = capsys.readouterr().out
    assert 'good.xml: OK' in out and 'bad.xml: INVALID' in out and 'unknown node <Fly>' in out
    assert cli.main([str(tmp_path / 'missing.xml')]) == 1


def test_cli_list_nodes_mentions_core(capsys):
    assert cli.main(['--list-nodes']) == 0
    assert 'DetectPerson' in capsys.readouterr().out


def test_reserved_port_names_are_rejected_in_manifests():
    from bt_common.catalog import check_port_names
    for bad in ('name', 'ID', '_x', '1a', 'a-b'):
        with pytest.raises(ValueError):
            check_port_names('N', [bad])
    check_port_names('N', ['person', 'out_identity', 'x1'])


def test_every_shipped_manifest_uses_legal_ports():
    from bt_common.catalog import _from_dict
    for cfg in glob.glob(os.path.join(SRC, '*_extension', 'config', '*_extension.yaml')):
        with open(cfg) as f:
            _from_dict(yaml.safe_load(f))


VOCAB = {('DetectObject', 'label'): {'red_cup', 'screwdriver'}}
DETECT = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>'
          '<DetectObject label="{lab}"/></Sequence></BehaviorTree></root>')


def test_vocab_accepts_known_rejects_unknown_with_hint(cat):
    assert validate_xml(DETECT.format(lab='red_cup'), cat, vocab=VOCAB) == []
    errs = validate_xml(DETECT.format(lab='mug'), cat, vocab=VOCAB)
    assert len(errs) == 1 and "'mug'" in errs[0] and 'red_cup' in errs[0] and 'screwdriver' in errs[0]


def test_vocab_skips_blackboard_refs_and_absent_vocab(cat):
    assert validate_xml(DETECT.format(lab='{target}'), cat, vocab=VOCAB) == []
    assert validate_xml(DETECT.format(lab='anything'), cat) == []            # no vocab: no gate
    assert validate_xml(DETECT.format(lab='anything'), cat, vocab={}) == []


def test_vocab_only_applies_to_its_node_and_port(cat):
    xml = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>'
           '<RecognizeFace identity="mug"/></Sequence></BehaviorTree></root>')
    assert validate_xml(xml, cat, vocab=VOCAB) == []


def test_cli_labels_option(tmp_path, capsys):
    f = tmp_path / 't.xml'
    f.write_text(DETECT.format(lab='mug'))
    assert cli.main([str(f)]) == 0
    assert cli.main([str(f), '--labels', 'red_cup,screwdriver']) == 1
    assert "'mug'" in capsys.readouterr().out
    assert cli.main([str(f), '--labels', 'mug']) == 0


def test_llm_listing_has_descriptions_but_slm_listing_does_not(cat):
    from bt_common.catalog import format_listing
    specs = list(cat.all().values())
    with_desc = format_listing(specs, with_desc=True)
    plain = format_listing(specs)
    assert 'DetectObject[Condition,sync](label,min_conf?,out_bbox?) - ' in with_desc
    assert 'Sleep[Action,async](msec) - ' in with_desc          # mode tag is LLM-only
    assert '[Action,async]' not in plain and '[Condition,sync]' not in plain
    assert ' - ' not in plain                      # SLM prompt/training format must not change
    for core in ('DetectObject', 'DetectPerson', 'RecognizeFace', 'ExtractFaceFeature'):
        assert core in with_desc
