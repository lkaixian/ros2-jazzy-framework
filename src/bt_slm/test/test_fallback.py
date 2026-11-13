import json

from bt_slm.fallback import build_xml
from bt_slm.prompt import extract_xml

SPEC = json.dumps({'goal': 'fetch cup', 'retry': 2, 'steps': [
    {'node': 'DetectObject', 'ports': {'label': 'cup'}}, {'node': 'Fly'}]})


def test_fallback_keeps_unknown_first_pass():
    assert '<Fly/>' in build_xml(SPEC, {'DetectObject'})


def test_fallback_drops_unknown_on_retry():
    xml = build_xml(SPEC, {'DetectObject'}, errors=['unknown node <Fly>'])
    assert '<Fly' not in xml and 'RetryUntilSuccessful' in xml


def test_extract_xml():
    assert extract_xml('junk <root a="1"></root> trailing') == '<root a="1"></root>'


NESTED = json.dumps({'goal': 'g', 'steps': [
    {'group': 'Fallback', 'retry': 3, 'steps': [
        {'node': 'DetectObject', 'ports': {'label': 'cup'}},
        {'group': 'Sequence', 'steps': [{'node': 'Fly'}]}]},
    {'node': 'Sleep', 'ports': {'msec': '10'}}]})


def test_nested_groups_render_and_validate_structure():
    import xml.etree.ElementTree as ET
    root = ET.fromstring(build_xml(NESTED))
    retry = root.find('.//RetryUntilSuccessful')
    assert retry.get('num_attempts') == '3' and retry[0].tag == 'Fallback'
    assert [c.tag for c in retry[0]] == ['DetectObject', 'Sequence']


def test_retry_filter_prunes_empty_groups_recursively():
    xml = build_xml(NESTED, {'DetectObject', 'Sleep'}, errors=['x'])
    assert 'Fly' not in xml and xml.count('<Sequence>') == 1      # inner Sequence removed
    assert '<Fallback>' in xml and 'Sleep' in xml
