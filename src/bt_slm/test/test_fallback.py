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
