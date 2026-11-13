import pytest
from bt_common.executor_node import PortError

from social_extension.social_extension_node import MAX_TEXT, render_text


def test_name_substitution():
    assert render_text('Hello {person}!', 'alice') == 'Hello alice!'
    assert render_text('{person} and {person}', 'bob') == 'bob and bob'


@pytest.mark.parametrize('name', ['', 'unknown', 'UNKNOWN', '  '])
def test_unknown_name_becomes_there(name):
    assert render_text('Hello {person}', name) == 'Hello there'


def test_text_without_placeholder_unchanged():
    assert render_text('Please look at the camera', 'alice') == 'Please look at the camera'


def test_empty_and_overlong_rejected():
    with pytest.raises(PortError):
        render_text('   ')
    with pytest.raises(PortError):
        render_text('x' * (MAX_TEXT + 1))
