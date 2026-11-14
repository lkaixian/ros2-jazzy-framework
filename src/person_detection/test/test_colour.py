import numpy as np
import pytest

from person_detection.colour import clothes_colour

BOX = (100, 50, 200, 350)


def frame_with(bgr):
    f = np.full((400, 300, 3), (90, 90, 90), np.uint8)       # grey background
    f[100:230, 125:175] = bgr                                  # the torso area of BOX
    return f


@pytest.mark.parametrize('bgr,name', [((0, 0, 220), 'red'), ((200, 40, 30), 'blue'), ((40, 200, 40), 'green'),
                                      ((20, 20, 20), 'black'), ((245, 245, 245), 'white'), ((0, 220, 220), 'yellow')])
def test_torso_colour_from_the_box(bgr, name):
    assert clothes_colour(frame_with(bgr), BOX) == name


def test_keypoints_pick_the_torso():
    f = np.full((400, 300, 3), 90, np.uint8)
    f[120:200, 130:170] = (0, 0, 220)                          # red between the shoulders and hips
    kx, ky = [0.0] * 17, [0.0] * 17
    for i, (x, y) in {5: (130, 120), 6: (170, 120), 11: (135, 200), 12: (165, 200)}.items():
        kx[i], ky[i] = x, y
    assert clothes_colour(f, BOX, kx, ky, [1.0] * 17) == 'red'


def test_tiny_region_is_unknown():
    assert clothes_colour(frame_with((0, 0, 220)), (10, 10, 14, 14)) == ''
