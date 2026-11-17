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
