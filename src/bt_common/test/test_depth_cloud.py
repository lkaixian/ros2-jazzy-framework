import numpy as np

from bt_common.depth_cloud import depth_to_points

K = (100.0, 100.0, 4.0, 4.0)        # fx, fy, cx, cy for an 8x8 image


def test_centre_pixel_is_straight_ahead_and_off_centre_pixels_move_sideways_and_down():
    d = np.zeros((8, 8))
    d[4, 4], d[4, 8 - 1] = 2.0, 2.0                 # centre; far right column (u = 7)
    p = depth_to_points(d, K, stride=1)
    assert np.allclose(sorted(p.tolist()), [[0.0, 0.0, 2.0], [0.06, 0.0, 2.0]])
    d = np.zeros((8, 8))
    d[7, 4] = 1.0                                   # bottom row: y (down) is positive
    assert np.allclose(depth_to_points(d, K, stride=1), [[0.0, 0.03, 1.0]])


def test_no_reading_too_near_and_too_far_are_dropped():
    d = np.array([[0.0, np.nan, 0.1, 5.0, 1.0]])
    p = depth_to_points(d, (100.0, 100.0, 2.0, 0.0), stride=1, min_m=0.3, max_m=3.0)
    assert p.shape == (1, 3) and p[0, 2] == 1.0


def test_stride_thins_the_cloud():
    d = np.full((8, 8), 1.0)
    assert len(depth_to_points(d, K, stride=1)) == 64 and len(depth_to_points(d, K, stride=4)) == 4
