import numpy as np

from pose_extension import attributes as A


def mask(shape=(100, 100), **cls):
    m = np.zeros(shape, np.uint8)
    for i, (name, (r0, r1, c0, c1)) in enumerate(cls.items()):
        m[r0:r1, c0:c1] = getattr(A, name)
    return m


def crop(bgr):
    return np.full((100, 100, 3), bgr, np.uint8)


FACE = 2500      # face area in the crop


def test_glasses_yes_needs_real_coverage_and_no_needs_visible_eyes():
    yes = A.attributes(mask(EYEGLASSES=(40, 50, 20, 80)), crop((0, 0, 0)), FACE)       # 600 px
    assert yes['glasses'] == 'yes'
    no = A.attributes(mask(L_EYE=(40, 45, 20, 40), R_EYE=(40, 45, 60, 80)), crop((0, 0, 0)), FACE)
    assert no['glasses'] == 'no'
    unsure = A.attributes(mask(SKIN=(0, 100, 0, 100)), crop((0, 0, 0)), FACE)          # eyes not seen: no claim
    assert unsure['glasses'] == ''


def test_hair_colour_from_the_hair_pixels_only():
    bgr = crop((200, 200, 200))
    m = mask(HAIR=(0, 40, 0, 100))
    bgr[:40] = (60, 100, 150)                                       # BGR of a mid-brown-orange
    assert A.attributes(m, bgr, FACE)['hair'] == 'brown'
    bgr[:40] = (20, 20, 20)
    assert A.attributes(m, bgr, FACE)['hair'] == 'black'
    assert A.attributes(mask(HAIR=(0, 5, 0, 20)), bgr, FACE)['hair'] == ''           # a sliver of hair is not enough


def test_hair_colour_rules():
    assert A.hair_colour(15, 100, 200) == 'blonde'
    assert A.hair_colour(8, 210, 170) == 'red'
    assert A.hair_colour(0, 10, 150) == 'grey' and A.hair_colour(0, 10, 230) == 'white'
    assert A.hair_colour(120, 200, 200) == ''                      # blue dye: not named


def test_accessories_and_hat_only_when_clearly_there():
    a = A.attributes(mask(HAT=(0, 30, 0, 100), EARRING=(50, 54, 10, 14), NECKLACE=(90, 95, 20, 80)), crop((0, 0, 0)), FACE)
    assert a['hat'] == 'yes' and a['accessories'] == ['earrings', 'a necklace']
    assert A.attributes(mask(EARRING=(50, 52, 10, 12)), crop((0, 0, 0)), FACE)['accessories'] == []      # 4 px: noise


def test_sentence_speaks_positives_only():
    assert A.sentence({'glasses': 'no', 'hair': '', 'hat': '', 'accessories': []}) == ''
    assert A.sentence({'glasses': 'yes', 'hair': 'brown', 'hat': '', 'accessories': []}) == 'wearing glasses and with brown hair'
    assert A.sentence({'glasses': 'yes', 'hair': 'brown', 'hat': '', 'accessories': ['earrings']}) == \
        'wearing glasses, with brown hair and wearing earrings'


def test_head_crop_is_clipped_to_the_image():
    assert A.head_crop((10, 10, 110, 150), 640, 480) == (0, 0, 160, 192)


def test_find_model_prefers_the_large_checkpoint(tmp_path):
    assert A.find_model(str(tmp_path / 'none.pt')) == ''
    f = tmp_path / 'm.pt'
    f.write_bytes(b'x')
    assert A.find_model(str(f)) == str(f)
