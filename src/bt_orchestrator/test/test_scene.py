import math

from bt_orchestrator.scene import describe_person
from pose_extension import gestures as g


def person(over=None):
    k = [(0.0, 0.0, 0.0)] * 17
    k = list(k)
    k[g.L_SH], k[g.R_SH] = (400, 200, 0.9), (300, 200, 0.9)
    k[g.L_EL], k[g.R_EL] = (410, 280, 0.9), (290, 280, 0.9)
    k[g.L_WR], k[g.R_WR] = (415, 360, 0.9), (285, 360, 0.9)
    for i, (x, y) in (over or {}).items():
        k[i] = (x, y, 0.9)
    return k


def test_no_skeleton_says_nothing_about_gestures():
    assert describe_person(3, [], 0.0) == {'id': 3}
    assert describe_person(3, [(0.0, [(0.0, 0.0, 0.0)] * 17)], 2.04) == {'id': 3, 'distance_m': 2.0}


def test_standing_person_is_explicitly_not_waving():
    d = describe_person(1, [(0.0, person())], 1.26)
    assert d == {'id': 1, 'distance_m': 1.3, 'waving': False}


def test_pointing_side_reported():
    kps = person({g.L_EL: (480, 205), g.L_WR: (560, 210)})
    assert describe_person(2, [(0.0, kps)])['pointing'] == 'right'


def test_waving_over_a_window():
    frames = [(i * 0.1, person({g.R_WR: (300 + 70 * math.sin(i * 1.2), 120)})) for i in range(20)]
    assert describe_person(4, frames)['waving'] is True
