import pytest

from social_extension.answers import extract_drink, extract_name


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
