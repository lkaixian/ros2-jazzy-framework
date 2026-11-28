import math
from types import SimpleNamespace as NS

from pid_extension.excuse import other_person_near, should_excuse
from pid_extension.retrace import forward_speed, retrace_step
from pid_extension.trail import Trail


def test_empty_trail():
    assert retrace_step((0, 0, 0), [], 0) == (0, None, 0.0, False)


def test_facing_the_wrong_way_turns_in_place():
    path = [(0.0, 0.0), (-1.0, 0.0), (-2.0, 0.0)]       # newest first, robot at the newest crumb facing +x, path leads to -x
    idx, tgt, err, done = retrace_step((0.0, 0.0, 0.0), path, 0)
    assert tgt == (-1.0, 0.0) and not done and abs(abs(err) - math.pi) < 1e-6
    assert forward_speed(err, 0.2) == 0.0
    assert forward_speed(0.0, 0.2) == 0.2 and 0 < forward_speed(0.4, 0.2) < 0.2


def test_bending_path_steers_to_the_bend_then_on():
    path = [(0.0, 0.0), (-0.5, 0.0), (-1.0, 0.0), (-1.0, -0.5), (-1.0, -1.0), (-1.0, -1.5)]
    idx, tgt, err, _ = retrace_step((-0.4, 0.0, math.pi), path, 0)          # facing -x, driving back
    assert tgt == (-1.0, 0.0) and abs(err) < 1e-6
    idx, tgt, err, _ = retrace_step((-1.0, -0.1, math.pi), path, idx)       # at the bend: next crumb is to the left of a robot facing -x
    assert tgt in ((-1.0, -0.5), (-1.0, -1.0))
    assert err > 0.5                                                         # south of a robot facing west = left


def test_progress_never_goes_back_and_arrival():
    path = [(float(-i) * 0.25, 0.0) for i in range(9)]
    idx = 0
    for x in (-0.5, -1.0, -1.5):
        idx, _, _, _ = retrace_step((x, 0.0, math.pi), path, idx)
    assert idx >= 6
    assert retrace_step((0.0, 0.0, math.pi), path, idx)[0] == idx
    assert retrace_step((-1.8, 0.0, math.pi), path, idx)[3] is True
    assert retrace_step((-1.5, 0.0, math.pi), path, idx)[3] is False


def test_trail_history_survives_target_pruning():
    t = Trail()
    for i in range(10):
        t.add((i * 0.3, 0.0))
    t.target((2.0, 0.0))
    assert len(t.crumbs) < 10 and len(t.history) == 10


def test_excuse_decision():
    assert should_excuse(1.5, True, None)
    assert not should_excuse(0.5, True, None)          # not blocked long enough
    assert not should_excuse(None, True, None)         # not blocked
    assert not should_excuse(2.0, False, None)         # nobody to ask
    assert not should_excuse(2.0, True, 5.0)           # asked 5 s ago
    assert should_excuse(2.0, True, 12.0)


def test_other_person_near():
    ps = [NS(track_id=1, distance_m=1.0), NS(track_id=2, distance_m=3.0), NS(track_id=3, distance_m=0.0)]
    assert not other_person_near(ps, 1)                # only the followed one is close (2 too far, 3 unranged)
    assert other_person_near(ps, 2)
