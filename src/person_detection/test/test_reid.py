import os

import numpy as np
import pytest

from person_detection.reid import Reid, preprocess

MODEL = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'models', 'reid', 'osnet_ain_x1_0.onnx')


def test_preprocess_gives_n_3_256_128_whatever_the_crop_size():
    x = preprocess([np.zeros((50, 20, 3), np.uint8), np.zeros((400, 300, 3), np.uint8)])
    assert x.shape == (2, 3, 256, 128) and x.dtype == np.float32


@pytest.mark.skipif(not os.path.exists(MODEL), reason='run tools/reid/export_osnet.py first')
def test_embeddings_are_unit_vectors_and_the_same_crop_matches_itself_best():
    rng = np.random.default_rng(0)
    a, b = (rng.integers(0, 255, (200, 90, 3), dtype=np.uint8) for _ in range(2))
    e = Reid(MODEL).embed([a, a, b])
    assert e.shape == (3, 512) and np.allclose(np.linalg.norm(e, axis=1), 1.0, atol=1e-4)
    assert e[0] @ e[1] > 0.999 and e[0] @ e[2] < e[0] @ e[1]


B1, B2, B3 = (10, 10, 110, 310), (400, 10, 500, 310), (12, 12, 112, 312)        # two places, and B1 moved a little


def test_identity_book_gives_a_returning_person_the_old_id_and_a_stranger_a_new_one():
    from person_detection.reid import IdentityBook
    a, b, c = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    book = IdentityBook(sim=0.70, keep_sec=10.0, confirm=1)
    assert book.assign([(14, a, B1), (3, b, B2)], 0.0) == [1, 2]
    assert book.assign([(14, a, B1)], 1.0) == [1]                               # same tracker id: same identity
    assert book.assign([(15, a * 0.97 + b * 0.1, B1)], 1.5) == [1]              # tracker restarted (14 -> 15) in the same place
    assert book.assign([(16, a, B2)], 8.0) == [1]                               # long gone, elsewhere: found again by looks
    assert book.assign([(40, c, B3)], 9.0) == [3]                              # nobody like them: a new id
    assert book.assign([(41, a, B3)], 100.0) == [4]                             # identity 1 forgotten after keep_sec


def test_identity_book_keeps_the_id_through_frames_the_tracker_gave_no_id():
    from person_detection.reid import IdentityBook
    a, odd = np.array([1.0, 0.0, 0.0]), np.array([0.5, 0.5, 0.7])               # one odd frame: cosine 0.5 to the person
    odd = odd / np.linalg.norm(odd)
    book = IdentityBook(sim=0.70, confirm=1)
    assert book.assign([(-1, a, B1)], 0.0) == [1]
    assert book.assign([(-1, odd, B3)], 0.07) == [1]                            # tracker id -1, the looks dipped: the place decides
    assert book.assign([(-1, a, B1)], 0.14) == [1]


def test_identity_book_remembers_a_second_look_that_still_resembles_the_first():
    from person_detection.reid import IdentityBook
    front, side = np.array([1.0, 0.0, 0.0]), np.array([0.7, 0.714, 0.0])        # cosine 0.7: the same person turned
    later = np.array([0.45, 0.893, 0.0])                                         # near the side look (0.95), far from the front (0.45)
    book = IdentityBook(sim=0.85, confirm=1)
    assert book.assign([(1, front, B1)], 0.0) == [1]
    assert book.assign([(1, side, B1)], 1.0) == [1]                             # the tracker kept the id while they turned: both looks stored
    assert book.assign([(2, later, B2)], 30.0) == [1]                           # lost; found again through the side look
    assert book.assign([(3, np.array([0.0, 0.0, 1.0]), B1)], 60.0) == [2]       # a stranger is not


def test_identity_book_does_not_create_a_person_from_a_flicker():
    from person_detection.reid import IdentityBook
    book = IdentityBook(sim=0.70, confirm=3)
    e = np.array([0.0, 0.0, 1.0])
    assert book.assign([(-1, e, B1)], 0.0) == [None] and book.assign([(-1, e, B3)], 0.1) == [None]
    assert book.assign([(-1, e, B1)], 0.2) == [1]                               # third look in the same place: confirmed
    assert book.assign([(-1, np.array([1.0, 0.0, 0.0]), B2)], 0.3) == [None]    # a stranger elsewhere needs its frames first
    assert book.assign([], 5.0) == [] and book.assign([(-1, e, B1)], 6.0) == [1]  # and a lone sighting is forgotten after `gap`


def test_only_a_tiny_look_is_unusable_and_makes_no_new_person():
    from person_detection.reid import IdentityBook, usable
    assert usable((100, 50, 200, 350))                                  # a standing person
    assert usable((176, 0, 639, 444))                                   # sitting close, filling the frame and cut by the side
    assert not usable((100, 50, 120, 90))                               # tiny
    book = IdentityBook(confirm=1)
    assert book.assign([(5, None, B1)], 0.0) == [None] and book.next == 1   # nobody created from nothing


def test_a_toss_up_between_two_people_gets_a_new_id_not_a_swap():
    from person_detection.reid import IdentityBook
    a, b = np.array([1.0, 0.0, 0.0]), np.array([0.6, 0.8, 0.0])
    book = IdentityBook(sim=0.70, confirm=1, margin=0.1)
    assert book.assign([(1, a, B1), (2, b, B2)], 0.0) == [1, 2]
    clear = a * 0.95 + b * 0.1
    assert book.assign([(8, clear / np.linalg.norm(clear), (200, 10, 300, 310))], 30.0) == [1]   # a clear winner is taken
    mid = (a + b) / np.linalg.norm(a + b)                                    # equally like both: 0.89 and 0.89
    assert book.assign([(7, mid, (200, 10, 300, 310))], 60.0) == [3]         # not a coin toss between 1 and 2


def test_a_look_that_resembles_nobody_is_not_stored_as_that_person():
    from person_detection.reid import IdentityBook
    a, blocker = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    book = IdentityBook(confirm=1)
    book.assign([(1, a, B1)], 0.0)
    book.assign([(1, blocker, B1)], 0.1)                                     # same tracker id, but the crop is the blocker
    assert len(book.gal[1][0]) == 1


def test_a_different_person_in_the_same_seat_does_not_inherit_the_id_but_a_blocker_for_a_moment_does():
    from person_detection.reid import IdentityBook
    a = np.array([1.0, 0.0, 0.0])
    other = np.array([0.57, 0.82, 0.0])                                      # cosine 0.57 to a: a different person
    book = IdentityBook(confirm=1)
    t = 0.0
    for _ in range(10):
        t += 0.07
        assert book.assign([(4, a, B1)], t) == [1]
    for _ in range(2):                                                       # a blocker for 2 frames
        t += 0.07
        assert book.assign([(4, other, B1)], t) == [1]
    for _ in range(3):
        t += 0.07
        assert book.assign([(4, a, B1)], t) == [1]
    ids = []
    for _ in range(14):                                                      # someone else sits down: same tracker id, same box
        t += 0.07
        ids.append(book.assign([(4, other, B1)], t)[0])
    assert ids[-1] not in (None, 1) and 1 not in ids[6:]                     # not person 1 once the recent looks disagree


def test_gallery_keeps_old_views_instead_of_only_the_latest():
    from person_detection.reid import IdentityBook
    book = IdentityBook(looks=8, add_min=-1.0, same=2.0)               # same=2: nothing is blended, every look is offered
    vs = [v / np.linalg.norm(v) for v in np.random.default_rng(1).normal(size=(200, 64))]
    for i, v in enumerate(vs):
        book._see(1, v, B1, i * 0.01)
    kept = book.gal[1][0]
    assert len(kept) == 8
    assert sum(any(k is v for v in vs[:100]) for k in kept) >= 1        # a latest-8 gallery would hold none of the first 100


def test_board_draws_one_row_per_person_with_a_thumbnail_per_look():
    from person_detection.reid import IdentityBook, board
    book = IdentityBook(confirm=1, looks=4)
    assert board(book, 0.0).shape[0] > 0                                  # empty book: a message, not a crash
    a, b = np.eye(3)[0], np.eye(3)[1]
    pic = np.full((80, 40, 3), 200, np.uint8)
    book.assign([(1, a, B1), (2, b, B2)], 0.0, [pic, None])
    img = board(book, 0.5)
    assert img.shape[0] == 2 * 84 and img[:, 130:170].max() == 200         # the thumbnail is on a row


def test_a_close_torso_is_not_compared_with_whole_body_looks_and_keeps_the_id():
    from person_detection.reid import IdentityBook
    size, body, torso = (640, 480), (200, 40, 300, 440), (150, 100, 450, 480)        # the torso box reaches the bottom edge
    full, close = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])                 # the network sees them as unrelated
    book = IdentityBook(confirm=1)
    assert book.assign([(5, full, body)], 0.0, size=size) == [1]
    for i in range(10):                                                                # walks up to the camera, tracker id unchanged
        assert book.assign([(5, close, torso)], 1.0 + i * 0.1, size=size) == [1]
    assert not any('not person' in e for e in book.events)
    assert sorted(book.gal[1][4]) == [0, 1]                                            # both kinds stored under the one id
    assert book.assign([(9, close, (160, 100, 460, 480))], 5.0, size=size) == [1]      # new tracker id, torso again: found by its torso looks


def test_a_crop_mostly_inside_another_persons_box_is_not_stored():
    from person_detection.reid import IdentityBook
    book = IdentityBook(confirm=1)
    a, b = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    book.assign([(1, a, (10, 10, 110, 310)), (2, b, (400, 10, 500, 310))], 0.0, size=(640, 480))
    n = len(book.gal[1][0])
    book.assign([(1, a, (10, 10, 110, 310)), (2, a * 0.8 + b * 0.6, (20, 10, 120, 310))], 1.0, size=(640, 480))   # 2 sits on top of 1
    assert len(book.gal[1][0]) == n and len(book.gal[2][0]) == 1                                                     # nothing new stored for either


def test_a_person_first_seen_whole_is_found_again_by_the_torso_alone():
    from person_detection.reid import IdentityBook
    size, body, torso = (640, 480), (200, 40, 300, 440), (150, 100, 450, 480)
    full, top, other = np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
    book = IdentityBook(confirm=1)
    assert book.assign([(5, full, body)], 0.0, size=size, uppers=[(top, None)]) == [1]
    assert book.assign([(9, top, torso)], 5.0, size=size) == [1]       # new tracker, far apart in time: the torso matches the stored upper half
    assert book.assign([(11, other, torso)], 9.0, size=size) == [2]    # a different torso is still someone else


def _hist(bgr):
    from person_detection.parts import histogram
    return histogram(np.full((60, 40, 3), bgr, np.uint8))


def test_part_boxes_follow_the_visible_keypoints_and_colour_tells_shirts_apart():
    from person_detection.parts import colour_similarity, part_boxes
    kx, ky, kc = [0.0] * 17, [0.0] * 17, [0.0] * 17
    for i, (x, y) in {5: (120, 100), 6: (200, 100), 11: (130, 250), 12: (190, 250), 15: (130, 400), 16: (190, 400)}.items():
        kx[i], ky[i], kc[i] = x, y, 0.9
    pb = part_boxes((100, 50, 220, 420), kx, ky, kc)
    assert pb['torso'] == (120, 100, 200, 250) and pb['legs'][1] == 250 and pb['legs'][3] == 400
    for i in (11, 12, 15, 16):
        kc[i] = 0.0                                                     # a close person: the hips are out of the picture
    pb = part_boxes((100, 50, 220, 480), kx, ky, kc)
    assert pb['torso'][3] == 480 and 'legs' not in pb                    # torso runs to the bottom of the box, no legs
    assert part_boxes((0, 0, 1, 1), [], [], []) == {}
    black, blue = _hist((10, 10, 10)), _hist((200, 90, 20))
    assert colour_similarity(black, black) == pytest.approx(1.0) and colour_similarity(black, blue) < 0.1


def test_matching_colour_lifts_a_posture_change_and_a_different_colour_blocks_a_lookalike():
    from person_detection.reid import IdentityBook
    base = np.array([1.0, 0.0, 0.0])
    turned = np.array([0.6, 0.8, 0.0])                                  # the same person turned: only 0.60 like the stored look
    lookalike = np.array([0.67, 0.0, 0.7416])                          # someone else the network finds 0.67 alike
    black, blue = _hist((10, 10, 10)), _hist((200, 90, 20))
    book = IdentityBook(confirm=1)
    assert book.assign([(1, base, B1)], 0.0, parts=[{'torso': black}]) == [1]
    assert book.assign([(7, turned, B2)], 9.0, parts=[{'torso': black}]) == [1]        # 0.6 x 0.6 + 0.4 x 1.0 = 0.76
    assert book.assign([(8, lookalike, B3)], 18.0, parts=[{'torso': blue}]) == [2]      # 0.6 x 0.67 + 0.4 x 0 = 0.40: a new person
    plain = IdentityBook(confirm=1)
    plain.assign([(1, base, B1)], 0.0)
    assert plain.assign([(7, turned, B2)], 9.0) == [2]                                  # without the colour cue it would have split


def test_a_split_person_is_merged_back_unless_the_two_were_in_view_together():
    from person_detection.reid import IdentityBook
    a, b, c = np.array([1.0, 0, 0]), np.array([0.5, 0.866, 0]), np.array([0.8, 0.6, 0])      # a-b 0.5 (a turned person), c between them
    book = IdentityBook(confirm=1, same=2.0)
    assert book.assign([(1, a, B1)], 0.0) == [1] and book.assign([(2, b, B2)], 5.0) == [2]   # split: b looks too different from a
    book._see(2, c, B2, 6.0)                                                                  # a look of person 2 that resembles both
    book.assign([], 8.0)
    assert sorted(book.gal) == [1] and book.map[2] == 1 and any('merged' in e for e in book.events)
    two = IdentityBook(confirm=1, same=2.0)
    assert two.assign([(1, a, B1), (2, c, B2)], 0.0) == [1, 2]                               # in one frame at once: two people
    two.assign([], 5.0)
    assert sorted(two.gal) == [1, 2]


def test_a_near_miss_is_taken_back_only_if_it_stays_a_clear_winner_over_its_frames_and_never_appears_with_that_person():
    from person_detection.reid import IdentityBook
    a = np.array([1.0, 0, 0])
    d = np.array([0.66, 0.7513, 0])                                   # 0.66 to a: under sim 0.70, within the near-miss band

    def known():                                                      # person 1 seen for the 3 frames a new person needs, then gone
        book = IdentityBook(confirm=3)
        assert [book.assign([(1, a, B1)], 0.1 * i) for i in range(3)][-1] == [1]
        return book
    book = known()
    got = [book.assign([(9, d, B2)], 5.0 + 0.1 * i) for i in range(3)]
    assert got == [[None], [None], [1]] and any('near miss' in e for e in book.events)
    far = known()
    e = np.array([0.5, 0.866, 0])                                     # 0.50: well below the band
    assert [far.assign([(9, e, B2)], 5.0 + 0.1 * i) for i in range(3)][-1] == [2]
    both = known()
    out = [both.assign([(1, a, B1), (9, d, B2)], 5.0 + 0.1 * i) for i in range(3)]
    assert out[-1] == [1, 2]                                          # in view at the same time as person 1: someone else


def test_find_model_auto_looks_for_the_repository_model_and_a_given_path_or_empty_is_respected(tmp_path):
    from person_detection.reid import find_model
    f = tmp_path / 'm.onnx'
    f.write_bytes(b'x')
    assert find_model(str(f)) == str(f) and find_model(str(tmp_path / 'missing.onnx')) == '' and find_model('') == ''
    auto = find_model('auto')
    assert auto == '' or auto.endswith(os.path.join('models', 'reid', 'osnet_ain_x1_0.onnx'))
