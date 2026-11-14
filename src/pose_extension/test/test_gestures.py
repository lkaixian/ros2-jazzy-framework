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


def test_count_confident():
    from types import SimpleNamespace as N
    from pose_extension.pose_extension_node import count_confident
    items = [N(confidence=0.9, label='cup'), N(confidence=0.3, label='cup'), N(confidence=0.8, label='bowl')]
    assert count_confident(items, 0.5) == 2
    assert count_confident(items, 0.5, 'cup') == 1
    assert count_confident([], 0.5) == 0


def test_free_seats_skips_taken_and_unconfident_ones():
    from types import SimpleNamespace as N
    from pose_extension.pose_extension_node import free_seats
    chair_taken = N(label='chair', confidence=0.9, x_min=0, y_min=0, x_max=100, y_max=100)
    chair_free_small = N(label='chair', confidence=0.8, x_min=300, y_min=0, x_max=350, y_max=50)
    couch_free = N(label='couch', confidence=0.7, x_min=400, y_min=0, x_max=600, y_max=100)
    dim_chair = N(label='chair', confidence=0.2, x_min=700, y_min=0, x_max=800, y_max=100)
    table = N(label='dining table', confidence=0.9, x_min=0, y_min=0, x_max=900, y_max=100)
    person = N(x_min=10, y_min=0, x_max=90, y_max=100)           # sits on the first chair
    got = free_seats([chair_taken, chair_free_small, couch_free, dim_chair, table], [person])
    assert got == [couch_free, chair_free_small]                    # biggest first; taken/dim/non-seat dropped
    assert free_seats([chair_taken], []) == [chair_taken]          # nobody in view: free
