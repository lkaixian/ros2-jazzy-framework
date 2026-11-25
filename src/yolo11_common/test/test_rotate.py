import numpy as np
import pytest
from yolo11_common.base_node import rotate_frame


def _frame():
    f = np.zeros((2, 4, 3), np.uint8)       # 2 rows x 4 columns
    f[0, 0] = (255, 0, 0)                   # top-left marker
    return f


def test_zero_returns_the_same_frame():
    f = _frame()
    assert rotate_frame(f, 0) is f


def test_180_moves_the_top_left_marker_to_the_bottom_right():
    r = rotate_frame(_frame(), 180)
    assert r.shape == (2, 4, 3) and tuple(r[1, 3]) == (255, 0, 0) and r[0, 0].sum() == 0


def test_90_and_270_swap_the_dimensions_and_are_inverses():
    f = _frame()
    assert rotate_frame(f, 90).shape == (4, 2, 3)
    assert np.array_equal(rotate_frame(rotate_frame(f, 90), 270), f)


def test_other_angles_are_rejected():
    with pytest.raises(KeyError):
        rotate_frame(_frame(), 45)
