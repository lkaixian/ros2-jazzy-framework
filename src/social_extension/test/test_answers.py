import pytest

from social_extension.answers import extract_drink, extract_name, is_yes


@pytest.mark.parametrize('said,name', [
    ('My name is Alice.', 'alice'), ("I'm Bob", 'bob'), ('you can call me Dana please', 'dana'),
    ('Hello, it is Erin', 'erin'), ('Carlos', 'carlos'), ("it's Mary-Jane", 'maryjane'),
    ('well hello there', ''), ('', '')])
def test_extract_name(said, name):
    assert extract_name(said) == name


@pytest.mark.parametrize('said,drink', [
    ('I like orange juice please.', 'orange juice'), ("My favourite drink is a cold cola", 'cold cola'),
    ('Coffee', 'coffee'), ('I would like some tea', 'tea'), ('hmm what a long sentence about nothing at all', '')])
def test_extract_drink(said, drink):
    assert extract_drink(said) == drink


@pytest.mark.parametrize('said,yes', [
    ('Yes.', True), ('Yeah, that is right', True), ('yep', True), ('Sure!', True), ('correct', True),
    ('No.', False), ('Nope, it is Larm', False), ("No, that's wrong", False), ('yes... no', False),
    ('my name is Larm', False), ('', False)])
def test_is_yes(said, yes):
    assert is_yes(said) is yes


def test_heard_any_matches_whole_words_only():
    from social_extension.answers import heard_any
    assert heard_any('Please STOP now', 'stop,we are here')
    assert heard_any("okay we are here, thanks", 'stop, we are here')
    assert not heard_any('nonstop music', 'stop')
    assert not heard_any('', 'stop') and not heard_any('hello', '')


def test_extract_place_names_one_arm_place():
    from social_extension.answers import extract_place
    assert extract_place('put it in the dustbin please') == 'bin'
    assert extract_place('on the table') == 'table' and extract_place('the shelf by the door') == 'shelf'
    assert extract_place('the table or the bin') == ''      # two places: ask again, do not guess
    assert extract_place('hello') == '' and extract_place('') == ''
