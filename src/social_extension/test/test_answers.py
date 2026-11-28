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


def test_age_from_digits_and_number_words_only_between_1_and_120():
    from social_extension.answers import extract_age
    assert extract_age("I'm 25") == '25' and extract_age('I am twenty five years old') == '25' and extract_age('thirty-two') == '32'
    assert extract_age('nineteen') == '19' and extract_age('twenty') == '20' and extract_age('I have 2 cats and I am 41') == '2'
    assert extract_age('hello') == '' and extract_age('my number is 2000') == '' and extract_age('zero') == ''


def test_misheard_names_snap_to_the_venue_list_and_strict_mode_refuses_strangers():
    from social_extension.answers import extract_name, snap_word, soundex
    names = ['jane', 'jules', 'axel', 'morgan']
    assert soundex('jane') == soundex('jain') and snap_word('jain', names) == 'jane' and snap_word('axle', names) == 'axel'
    assert snap_word('bob', names) == ''
    assert extract_name('my name is jain', names) == 'jane' and extract_name('uh jules please', names) == 'jules'
    assert extract_name('my name is bob', names) == 'bob'                         # not on the list: kept as heard...
    assert extract_name('my name is bob', names, strict=True) == ''               # ...or refused when the list is the whole truth
    assert extract_name('my name is alice') == 'alice'                            # no list: unchanged behaviour


def test_drinks_snap_to_the_list_but_an_unlisted_drink_is_kept():
    from social_extension.answers import extract_drink
    drinks = ['orange juice', 'coke']
    assert extract_drink('I like orange juse please', drinks) == 'orange juice'
    assert extract_drink('I like green tea', drinks) == 'green tea' and extract_drink('I like green tea') == 'green tea'


def test_gender_from_the_words_a_guest_would_use_and_nothing_when_unclear():
    from social_extension.answers import extract_gender
    assert extract_gender('I am a woman') == 'woman' and extract_gender('male') == 'man' and extract_gender('She is a girl') == 'woman'
    assert extract_gender('hello') == '' and extract_gender('a man and a woman') == ''


def test_a_public_persons_name_is_kept_as_heard_when_snapping_names_is_off():
    from social_extension.answers import extract_name
    names = ['liam', 'noah', 'laura']
    assert extract_name('my name is lia', names, snap=False) == 'lia'                 # not turned into liam
    assert extract_name('my name is lia', names, snap=True) == 'liam'                  # the old behaviour, when switched on
    assert extract_name('uh noah please', names, snap=False) == 'noah'                 # a listed name is still taken exactly
    assert extract_name('my name is zebedee', names, snap=False) == 'zebedee'          # any word is a name


def test_the_default_vocabulary_has_the_listed_names_and_drinks_and_keeps_any_other_word():
    import os
    from social_extension.vocab import load_vocab
    v = load_vocab(os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param', 'vocab.yaml'))
    assert 'liam' in v['names'] and 'isabella' in v['names'] and len(v['names']) == 15 and 'root beer' in v['drinks'] and len(v['drinks']) == 10
    assert v['snap_names'] is False and v['strict_names'] is False


def test_the_drink_is_the_drink_not_the_verb_phrase_round_it():
    from social_extension.answers import extract_drink
    assert extract_drink('I like to drink milk.') == 'milk' and extract_drink('I would like to have a cup of tea please') == 'cup of tea'
    assert extract_drink('my favourite drink is root beer') == 'root beer'
    drinks = ['milk', 'root beer', 'tea', 'sparkling water', 'water']
    assert extract_drink('I like to drink milk.', drinks) == 'milk' and extract_drink('um sparkling water please', drinks) == 'sparkling water'
    assert extract_drink('I really love kombucha', drinks) == 'really love kombucha' or extract_drink('I really love kombucha', drinks) == 'kombucha'


def test_a_sentence_that_is_not_a_name_gives_no_name():
    from social_extension.answers import extract_name
    assert extract_name("I'm not sure what it is.") == ''
    assert extract_name("I'm sorry") == '' and extract_name('Hi, my name is Liam.') == 'liam'
