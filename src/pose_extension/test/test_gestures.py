import math

from pose_extension import gestures as g


def person(over=None):
    """Standing person facing camera; shoulders 100 px apart. over: {idx: (x, y)}."""
    over = over or {}
    k = [(0.0, 0.0, 0.0)] * 17
    k = list(k)
    k[g.L_SH], k[g.R_SH] = (400, 200, 0.9), (300, 200, 0.9)
    k[g.L_EL], k[g.R_EL] = (410, 280, 0.9), (290, 280, 0.9)
    k[g.L_WR], k[g.R_WR] = (415, 360, 0.9), (285, 360, 0.9)
    for i, (x, y) in over.items():
        k[i] = (x, y, 0.9)
    return k


def test_has_pose():
    assert g.has_pose(person(), 0.5)
    assert not g.has_pose([(0, 0, 0.0)] * 17, 0.5)
    assert not g.has_pose(person(), 0.95)


def test_pointing_image_right_and_left():
    right = person({g.L_EL: (480, 205), g.L_WR: (560, 210)})
    assert g.pointing(right, direction='') == 'right'
    assert g.pointing(right, direction='left') is None
    left = person({g.R_EL: (220, 205), g.R_WR: (140, 210)})
    assert g.pointing(left) == 'left'


def test_arm_down_or_bent_is_not_pointing():
    assert g.pointing(person()) is None
    bent = person({g.L_EL: (480, 205), g.L_WR: (470, 130)})   # forearm up
    assert g.pointing(bent) is None


def test_waving_needs_raised_swinging_wrist():
    frames = [(i * 0.1, person({g.R_WR: (300 + 70 * math.sin(i * 1.2), 120)}))
              for i in range(15)]
    assert g.waving(frames)
    still_up = [(i * 0.1, person({g.R_WR: (300, 120)})) for i in range(15)]
    assert not g.waving(still_up)
    swinging_low = [(i * 0.1, person({g.R_WR: (300 + 70 * math.sin(i * 1.2), 360)}))
                    for i in range(15)]
    assert not g.waving(swinging_low)
    assert not g.waving(frames[:3])                   # too few samples
