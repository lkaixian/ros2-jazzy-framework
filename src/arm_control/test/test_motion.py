import pytest

from arm_control.gripper import SAFE_CLOSE, SAFE_OPEN, gap_mm, joint_m_for_gap, ticks_for_gap, to_m
from arm_control.motion import Refused, plan_route, schedule

A = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0)
B = dict(joint1=0.3, joint2=-0.3, joint3=0.3, joint4=0.0)


def test_straight_path_when_the_map_allows_it():
    assert plan_route(A, B, verdict=lambda a, b, m=None: (True, 'ok')) == [A, B]


def test_no_map_is_a_refusal_not_a_guess():
    with pytest.raises(Refused, match='no map'):
        plan_route(A, B, verdict=lambda a, b, m=None: (None, 'no map'))


def test_taught_route_used_when_straight_is_refused_and_refused_without_one():
    mid = dict(A, joint1=0.1)
    assert plan_route(A, B, verdict=lambda a, b, m=None: (False, 'x'), taught=lambda a, b, m=None: ([A, mid, B], 'r')) == [A, mid, B]
    with pytest.raises(Refused, match='no taught route'):
        plan_route(A, B, verdict=lambda a, b, m=None: (False, 'x'), taught=lambda a, b, m=None: (None, 'none'))


def test_schedule_never_faster_than_rate_and_ends_at_the_goal():
    s = schedule([A, B], rate=0.03)
    assert len(s) == 1 and s[0][0] == [0.3, -0.3, 0.3, 0.0] and s[0][1] == pytest.approx(10.0)
    assert schedule([A, A])[0][1] == 1.0                                  # a no-op leg still takes the minimum time
    assert [t for _, t in schedule([A, B, A])] == pytest.approx([10.0, 20.0])


def test_gripper_gap_round_trip_and_safe_ends():
    assert gap_mm(ticks_for_gap(40.0)) == pytest.approx(40.0, abs=0.1)
    assert joint_m_for_gap(0.0) == pytest.approx(to_m(SAFE_CLOSE)) and joint_m_for_gap(500.0) == pytest.approx(to_m(SAFE_OPEN))
