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
