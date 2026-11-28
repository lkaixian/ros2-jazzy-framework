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


def test_reid_by_appearance_vector_picks_the_most_similar_person_wherever_they_are():
    m = Memory()
    a = person(1, 320, 2.0)
    a.embedding = [1.0, 0.0, 0.0]
    m.update(a, (0, 0, 0), 0.0, HFOV, W)
    twin, other = person(9, 100, 6.0), person(8, 320, 2.0)                                # twin: far from the last position
    twin.embedding, other.embedding = [0.95, 0.1, 0.0], [0.0, 1.0, 0.0]
    assert select([other, twin], m, (0, 0, 0), HFOV, W, lock=True).track_id == 9
    assert select([other], m, (0, 0, 0), HFOV, W, lock=True) is None                      # below reid_sim: nobody is adopted


def test_face_visible_needs_nose_and_an_eye_inside_the_image():
    from pid_extension.follow import face_distance, face_visible
    f = lambda conf, ys: NS(keypoints_conf=conf, keypoints_y=ys)   # noqa: E731
    assert face_visible(f([.9, .9, .1], [100, 90, 90]), 480)
    assert not face_visible(f([.9, .1, .1], [100, 90, 90]), 480)         # nose only (profile or hair)
    assert not face_visible(f([.9, .9, .9], [100, 1, 90]), 480)          # eye on the very top edge: cut off
    assert not face_visible(f([], []), 480)                              # no pose
    # camera 0.8 m up, 42 deg lens: a 1.6 m face is in view from about 2.4 m, a lower camera needs more room
    assert face_distance(0.8, 1.6, 42.0) == pytest.approx(2.45, abs=0.05)
    assert face_distance(0.5, 1.6, 42.0) > face_distance(0.8, 1.6, 42.0) and face_distance(2.0, 1.6, 42.0) == 0.0


def test_trail_steers_to_a_crumb_ahead_not_straight_at_the_person_and_drops_crumbs_behind():
    from pid_extension.trail import Trail, trail_error
    t = Trail()
    for pos in [(1, 0), (2, 0), (3, 0), (3.5, 1), (3.5, 2)]:         # the person walks ahead, then turns left around a corner
        t.add(pos)
    assert t.target((0, 0)) == (1, 0)                                  # the first crumb at least 0.8 m away: along the walked path
    assert t.target((2.1, 0)) == (3, 0) and t.crumbs[0] == (2, 0)        # crumbs behind the robot's place on the path are dropped
    assert trail_error((0, 0, 0.0), (3.5, 2), 69.0) < 0 < trail_error((0, 0, 0.0), (3.5, -2), 69.0)   # a point on the left: turn left (negative offset)
    assert trail_error((0, 0, 0.0), (-1, 0), 69.0) in (-1.5, 1.5)    # behind the camera: the largest turn, never a huge number
    t2 = Trail()
    t2.add((1, 0))
    t2.add((1.1, 0))                                                   # too close to the last crumb: not a new one
    assert len(t2.crumbs) == 1 and Trail().target((0, 0)) is None


def walk(m, t, speed=1.0):
    """Feed a person walking along +x at `speed` m/s, seen from the origin facing +x (lateral noise in the bearing)."""
    x = 2.0 + speed * t
    p = person(1, 320, x)
    m.update(p, (0, 0, 0), t, HFOV, W)
    return x


def test_memory_filters_depth_jitter():
    import random
    rng, m, e_raw, e_f = random.Random(1), Memory(), [], []
    for i in range(60):
        t = i * 0.1
        true = 2.0 + t
        m.update(person(1, 320, true + rng.gauss(0, 0.08)), (0, 0, 0), t, HFOV, W)
        if i > 20:
            e_f.append(abs(m.pos[0] - true))
            e_raw.append(0.08)
    assert sum(e_f) / len(e_f) < 0.8 * sum(e_raw) / len(e_raw)


def test_predict_extrapolates_through_a_gap_and_holds_after_two_seconds():
    m = Memory()
    assert m.predict(0.0) is None
    for i in range(30):
        walk(m, i * 0.1)
    last = 2.0 + 2.9
    assert m.predict(2.9 + 1.5)[0] == pytest.approx(last + 1.5, abs=0.25)
    assert m.predict(2.9 + 10.0)[0] == pytest.approx(last + 2.0, abs=0.3)


def test_reid_matches_a_person_who_walked_on_while_hidden_only_with_the_prediction():
    m = Memory()
    for i in range(30):
        walk(m, i * 0.1)
    back = person(7, 320, 2.0 + 2.9 + 1.4)                              # 1.4 s later, 1.4 m further along (> reid_radius from the last sighting)
    assert select([back], m, (0, 0, 0), HFOV, W, lock=True) is None      # stale position: too far
    assert select([back], m, (0, 0, 0), HFOV, W, lock=True, now=2.9 + 1.4).track_id == 7


def test_an_outlier_does_not_move_the_position():
    m = Memory()
    for i in range(30):
        walk(m, i * 0.1)
    before = m.pos
    m.update(person(1, 320, 8.0), (0, 0, 0), 3.0, HFOV, W)               # 3 m jump in 0.1 s: someone else, or a glitch
    assert m.pos == before
