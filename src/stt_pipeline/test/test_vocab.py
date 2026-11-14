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
