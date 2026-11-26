import math
from types import SimpleNamespace as NS

import pytest

from pid_extension.follow import LostPolicy, Memory, bearing_from_image, select, wrap

W, HFOV = 640, 69.0


def person(tid, cx, dist=2.0, colour='', h=200.0):
    return NS(track_id=tid, clothes_colour=colour, distance_m=dist, x_min=cx - h / 4, x_max=cx + h / 4, y_min=240 - h / 2, y_max=240 + h / 2)


def test_bearing_is_zero_centred_and_positive_to_the_left():
    assert bearing_from_image(0.0, HFOV) == 0.0
    assert bearing_from_image(-1.0, HFOV) == pytest.approx(math.radians(34.5), abs=0.02)
    assert bearing_from_image(1.0, HFOV) < 0


def test_memory_records_the_position_from_the_depth_and_the_side_from_the_image():
    m = Memory()
    m.update(person(4, 320 + 160, 2.0, 'red'), (1.0, 1.0, math.pi / 2), 5.0, HFOV, W)     # right of centre, robot facing +y
    assert m.track_id == 4 and m.colour == 'red' and m.rel_bearing < 0 and m.seen_t == 5.0
    assert m.pos[1] > 2.5 and m.pos[0] > 1.0                                              # ahead (+y) and to its right (+x)
    m2 = Memory()
    m2.update(person(4, 320, 0.0), (0, 0, 0), 1.0, HFOV, W)
    assert m2.pos is None and m2.world_bearing == 0.0                                      # no depth: bearing only


def test_select_biggest_without_lock_and_the_locked_id_otherwise():
    m = Memory()
    ps = [person(1, 100, h=100.0), person(2, 500, h=300.0)]
    assert select(ps, m, (0, 0, 0), HFOV, W).track_id == 2
    assert select(ps, m, (0, 0, 0), HFOV, W, tid=1).track_id == 1
    m.track_id = 1
    assert select(ps, m, (0, 0, 0), HFOV, W, lock=True).track_id == 1


def test_reid_takes_the_same_colour_near_the_last_position_and_never_a_different_colour():
    m = Memory()
    m.update(person(1, 320, 2.0, 'red'), (0, 0, 0), 0.0, HFOV, W)                         # last seen at (2, 0), red
    back = person(9, 320, 2.3, 'red')                                                    # new id, same place, same colour
    other = person(8, 320, 2.1, 'blue')                                                  # a stranger in the same place
    far = person(7, 320, 5.0, 'red')                                                     # right colour but 3 m from where they were
    assert select([other, far], m, (0, 0, 0), HFOV, W, lock=True) is None
    assert select([other, far, back], m, (0, 0, 0), HFOV, W, lock=True).track_id == 9
    assert select([person(5, 320, 2.0, '')], m, (0, 0, 0), HFOV, W, lock=True).track_id == 5   # colour unknown now: position decides


def test_reid_without_depth_uses_the_bearing():
    m = Memory()
    m.update(person(1, 320, 0.0), (0, 0, 0), 0.0, HFOV, W)
    assert select([person(3, 330, 0.0)], m, (0, 0, 0), HFOV, W, lock=True).track_id == 3
    assert select([person(3, 620, 0.0)], m, (0, 0, 0), HFOV, W, lock=True) is None


def test_lost_policy_drives_to_the_last_position_then_sweeps_toward_the_side_they_left_then_fails():
    m = Memory()
    m.update(person(1, 100, 3.0), (0, 0, 0), 0.0, HFOV, W)                                # left of centre, 3 m away
    pol = LostPolicy(goto_sec=3.0, search_sec=12.0)
    v, w, ph = pol.step(m, (0, 0, 0), 10.0)
    assert ph == 'goto' and v > 0 and w > 0                                              # they are ahead-left: drive and veer left
    v, w, ph = pol.step(m, (m.pos[0] - 0.3, m.pos[1], 0.0), 10.5)
    assert ph == 'search' and v == 0 and w > 0                                          # arrived: sweep left first
    yaw, t = 0.0, 10.5
    while ph != 'fail' and t < 40:
        t += 0.05
        v, w, ph = pol.step(m, (m.pos[0] - 0.3, m.pos[1], yaw), t)
        yaw = wrap(yaw + w * 0.05)
    assert ph == 'fail' and t - 10.5 <= 12.5                                             # gave up inside search_sec


def test_lost_policy_waits_without_odometry_then_fails_and_skips_motion_the_lidar_forbids():
    m = Memory()
    m.update(person(1, 320, 2.0), (0, 0, 0), 0.0, HFOV, W)
    pol = LostPolicy(wait_sec=1.0)
    assert pol.step(m, None, 5.0)[2] == 'wait' and pol.step(m, None, 5.9)[2] == 'wait' and pol.step(m, None, 6.2)[2] == 'fail'
    pol = LostPolicy()
    assert pol.step(m, (0, 0, 0), 5.0, can_drive=False)[:2] == (0.0, 0.0)               # blocked ahead: do not drive
