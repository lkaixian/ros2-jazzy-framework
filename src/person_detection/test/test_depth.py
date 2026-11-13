import numpy as np
import pytest

from person_detection.depth_ranging import depth_from_image, person_distance

BOX = (100.0, 50.0, 300.0, 450.0)           # a 200 x 400 px person in a 640 x 480 colour image
SIZE = (640, 480)


def scene(person_mm=2000, background_mm=5000, shape=(480, 640)):
    d = np.full(shape, background_mm, dtype=np.uint16)
    sx, sy = shape[1] / 640.0, shape[0] / 480.0
    d[int(50 * sy):int(450 * sy), int(100 * sx):int(300 * sx)] = person_mm
    return d


def test_median_depth_of_the_person_not_the_background():
    assert person_distance(scene(), BOX, SIZE) == pytest.approx(2.0)


def test_central_region_ignores_background_around_the_silhouette():
    d = scene()
    d[:, :110] = 5000                         # background strip left of the box edge
    d[50:90, 100:300] = 5000                  # background above the head
    assert person_distance(d, BOX, SIZE) == pytest.approx(2.0)


def test_depth_resolution_may_differ_from_colour():
    assert person_distance(scene(shape=(240, 320)), BOX, SIZE) == pytest.approx(2.0)


def test_float_metres_scale():
    d = scene().astype(np.float32) / 1000.0
    assert person_distance(d, BOX, SIZE, scale=1.0) == pytest.approx(2.0)


def test_invalid_pixels_are_ignored_and_outliers_do_not_win():
    d = scene()
    d[200:260, 150:250] = 0                   # holes (common on RealSense)
    d[300:310, 150:250] = 300                 # closer than min_m
    assert person_distance(d, BOX, SIZE) == pytest.approx(2.0)


def test_unknown_is_zero_not_a_guess():
    assert person_distance(np.zeros((480, 640), np.uint16), BOX, SIZE) == 0.0
    mostly_holes = scene()
    mostly_holes[:, :] = 0
    mostly_holes[250:252, 190:192] = 2000     # < 10 % valid pixels in the region
    assert person_distance(mostly_holes, BOX, SIZE) == 0.0
    assert person_distance(scene(person_mm=9000), BOX, SIZE) == 0.0   # beyond max_m
    assert person_distance(scene(), (700, 500, 800, 600), SIZE) == 0.0   # box outside image
    assert person_distance(scene(), BOX, (0, 0)) == 0.0


def test_depth_from_image_16uc1_with_row_padding():
    h, w = 4, 5
    step = w * 2 + 6                           # 3 padding pixels per row
    raw = np.zeros((h, step // 2), dtype='<u2')
    raw[:, :w] = np.arange(h * w).reshape(h, w) + 1000
    arr, scale = depth_from_image('16UC1', h, w, step, raw.tobytes())
    assert arr.shape == (h, w) and scale == 0.001 and arr[1, 2] == 1007


def test_depth_from_image_32fc1_and_unsupported():
    f = np.ones((2, 3), dtype='<f4') * 1.5
    arr, scale = depth_from_image('32FC1', 2, 3, 12, f.tobytes())
    assert scale == 1.0 and float(arr[0, 0]) == 1.5
    with pytest.raises(ValueError):
        depth_from_image('rgb8', 2, 3, 9, b'0' * 18)
