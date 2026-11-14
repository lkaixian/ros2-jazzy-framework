import json

import pytest

from bt_slm.fallback import build_xml
from bt_slm.prompt import extract_xml

SPEC = json.dumps({'goal': 'fetch cup', 'retry': 2, 'steps': [
    {'node': 'DetectObject', 'ports': {'label': 'cup'}}, {'node': 'Fly'}]})


def test_fallback_keeps_unknown_first_pass():
    assert '<Fly/>' in build_xml(SPEC, {'DetectObject'})


def test_retry_fails_instead_of_dropping_unknown_steps():
    with pytest.raises(ValueError, match='Fly'):
        build_xml(SPEC, {'DetectObject'}, errors=['unknown node <Fly>'])


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


def test_retry_finds_unknown_steps_inside_nested_groups():
    with pytest.raises(ValueError, match='Fly'):
        build_xml(NESTED, {'DetectObject', 'Sleep'}, errors=['x'])
    assert 'Fly' in build_xml(NESTED, {'DetectObject', 'Sleep', 'Fly'}, errors=['x'])      # all known: kept whole


def test_repeat_wraps_the_group_and_retry_is_outermost():
    import json

    from bt_slm.fallback import build_xml
    spec = {'goal': 'loop', 'steps': [{'group': 'Sequence', 'repeat': 3, 'retry': 2,
                                        'steps': [{'node': 'Sleep', 'ports': {'msec': '10'}}]}]}
    xml = build_xml(json.dumps(spec))
    assert xml.index('RetryUntilSuccessful num_attempts="2"') < xml.index('Repeat num_cycles="3"') < xml.rindex('<Sequence>')
    from bt_common.catalog import Catalog
    from bt_validator.validate import validate_xml
    assert validate_xml(xml, Catalog()) == []


def test_parallel_group_carries_its_counts_and_validates():
    import json

    from bt_common.catalog import Catalog
    from bt_slm.fallback import build_xml
    from bt_validator.validate import validate_xml
    spec = {'goal': 'par', 'steps': [{'group': 'Parallel', 'success_count': 1, 'failure_count': 2, 'steps': [
        {'node': 'Sleep', 'ports': {'msec': '10'}}, {'node': 'Sleep', 'ports': {'msec': '20'}}]}]}
    xml = build_xml(json.dumps(spec))
    assert '<Parallel success_count="1" failure_count="2">' in xml
    assert validate_xml(xml, Catalog()) == []
