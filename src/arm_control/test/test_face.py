import math

from arm_control.face import turn_to_face


def after(x, y, a, pivot_x=0.16):
    return -math.sin(a) * (x + pivot_x) + math.cos(a) * y


def test_an_object_on_the_arm_centre_line_needs_a_left_turn_that_puts_it_at_the_camera_axis():
    a = turn_to_face(0.235, -0.004, -0.08)               # measured: the arm centre line is at the left edge of the RealSense picture
    assert 0.1 < a < 0.3 and abs(after(0.235, -0.004, a) - (-0.08)) < 1e-3


def test_an_object_far_to_the_right_needs_a_right_turn():
    a = turn_to_face(0.252, -0.195, -0.08)
    assert -0.45 < a < -0.15 and abs(after(0.252, -0.195, a) - (-0.08)) < 1e-3


def test_an_object_already_near_the_camera_axis_needs_no_turn_and_a_huge_turn_is_refused():
    assert turn_to_face(0.235, -0.069, -0.08) == 0.0     # the middle run: 1 cm off, inside the dead band
    assert turn_to_face(0.25, 0.40, -0.08) is None       # 40 cm to the left would need more than 0.6 rad
