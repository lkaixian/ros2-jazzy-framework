from stt_pipeline.vocab import hint_prompt, place_phrases, snap

P = ['far corner', 'left room', 'right room', 'door', 'start']


def test_near_misses_snap_to_a_place():
    assert snap('go to the for corner', P) == 'go to the far corner'
    assert snap('go to the lift room please', P) == 'go to the left room please'
    assert snap('go to the right rum', P) == 'go to the right room'


def test_exact_words_and_other_text_stay():
    for s in ('go to the door', 'patrol between the start and the door', 'tell me a story', 'look at me'):
        assert snap(s, P) == s


def test_helpers():
    assert hint_prompt(['door', 'start']) == 'Go to the door, start.' and hint_prompt([]) == ''
    assert place_phrases('/nonexistent.yaml') == []


def test_hint_prompt_adds_names_and_drinks_and_load_vocab_reads_the_yaml(tmp_path):
    from stt_pipeline.vocab import hint_prompt, load_vocab
    p = hint_prompt(['far corner'], ['jane', 'axel'], ['coke'])
    assert 'far corner' in p and 'Jane, Axel' in p and 'coke' in p and hint_prompt([]) == ''
    f = tmp_path / 'v.yaml'
    f.write_text('names: [jane]\ndrinks: [milk]\nstrict_names: true\n')
    assert load_vocab(str(f)) == {'names': ['jane'], 'drinks': ['milk'], 'strict_names': True}
    assert load_vocab(str(tmp_path / 'missing.yaml'))['names'] == []
