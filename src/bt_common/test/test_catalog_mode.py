import pytest
from bt_common.catalog import Catalog, NodeSpec, _from_dict, default_mode, format_listing


def test_default_mode_rule():
    assert default_mode('Condition', extension=False) == 'sync'      # SimpleCondition in bt_executor
    assert default_mode('Action', extension=False) == 'async'        # e.g. Sleep
    assert default_mode('Condition', extension=True) == 'async'      # every extension node is ExtNode
    assert default_mode('Action', extension=True) == 'async'
    assert default_mode('Control', extension=False) == ''
    assert default_mode('Decorator', extension=True) == ''


def test_core_catalog_modes():
    c = Catalog().core
    assert c['DetectObject'].mode == 'sync' and c['Sleep'].mode == 'async'
    assert c['AlwaysSuccess'].mode == 'sync' and c['Sequence'].mode == ''


def test_explicit_mode_must_be_valid():
    with pytest.raises(ValueError):
        _from_dict({'nodes': {'X': {'category': 'Action', 'mode': 'maybe'}}})
    with pytest.raises(ValueError):
        _from_dict({'nodes': {'X': {'category': 'Control', 'mode': 'sync'}}})
    assert _from_dict({'nodes': {'X': {'category': 'Action', 'mode': 'sync'}}})['X'].mode == 'sync'


def test_llm_listing_has_mode_but_slm_listing_is_unchanged():
    specs = [NodeSpec('Go', 'Action', ['m'], [], 'drive', 'async'), NodeSpec('Seq', 'Control')]
    assert format_listing(specs) == 'Go[Action](m)\nSeq[Control]()'                 # SLM: byte-identical
    assert format_listing(specs, with_desc=True) == 'Go[Action,async](m) - drive\nSeq[Control]()'
