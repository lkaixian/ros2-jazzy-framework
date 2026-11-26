import math

import numpy as np
from arm_control.track import Track, scan_yaws

RNG = np.random.default_rng(3)


def noisy(p, sd=0.02):
    return (np.asarray(p) + RNG.normal(0, sd, 3)).tolist()


def test_the_filter_averages_depth_noise_over_several_looks():
    tr = Track(noisy([0.4, 0.0, 0.05]), 0.0)
    for i in range(1, 12):
        assert tr.step(i * 0.3, [noisy([0.4, 0.0, 0.05])]) == 'seen'
    assert np.linalg.norm(tr.pos - [0.4, 0.0, 0.05]) < 0.015 and tr.sigma < 0.02 and tr.seen == 12


def test_it_keeps_the_pinned_object_when_a_second_one_of_the_same_label_is_in_view():
    tr = Track([0.40, 0.0, 0.05], 0.0)
    for i in range(1, 6):
        tr.step(i * 0.3, [[0.40, 0.0, 0.05], [0.40, 0.25, 0.05]])
    assert abs(tr.pos[1]) < 0.02                                   # not dragged to the other bottle
    assert tr.step(2.0, [[0.40, 0.25, 0.05]]) == 'coasting'        # only the other one visible: that is a miss, not our object


def test_it_coasts_through_missed_looks_along_the_estimated_velocity():
    tr = Track([0.30, 0.0, 0.05], 0.0)
    for i in range(1, 10):                                         # the object slides 2 cm per look in +y
        tr.step(i * 0.5, [[0.30, 0.02 * i, 0.05]])
    assert tr.vel[1] > 0.02
    y = tr.pos[1]
    for i in range(10, 13):
        assert tr.step(i * 0.5, []) == 'coasting'
    assert tr.missed == 3 and tr.pos[1] > y + 0.03 and tr.sigma > 0.02     # it moved on its prediction and is less sure


def test_a_found_object_resets_the_miss_count():
    tr = Track([0.3, 0, 0.05], 0.0)
    tr.step(0.5, [])
    assert tr.missed == 1
    assert tr.step(1.0, [[0.3, 0, 0.05]]) == 'seen' and tr.missed == 0


def test_scan_yaws_start_at_the_prediction_alternate_and_stay_in_the_taught_range():
    ws = ((0.05, -0.4, -0.3), (0.45, 0.4, 0.4))
    ys = scan_yaws([0.40, 0.10, 0.05], *ws, -0.5, 0.5, 0.3, n=4)
    b = math.atan2(0.10, 0.40)
    assert ys == [b, b - 0.3, b - 0.6]                            # b + 0.3 would leave the taught +-0.5 range, so it is skipped
    far = scan_yaws([2.0, 0.0, 0.0], *ws, -0.5, 0.5, 0.3)         # a prediction beyond reach is clipped into the box, not followed
    assert far[0] == 0.0
    assert scan_yaws([0.0, 0.0, 0.0], (0.0, 0.0, -0.3), (0.0, 0.0, 0.4), -0.5, 0.5, 0.3) == []     # nothing to look along above the base axis
