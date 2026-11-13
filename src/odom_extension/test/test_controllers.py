import math

import pytest

from odom_extension.controllers import DriveDistance, Limits, RotateBy, goto_step, wrap

DT = 0.05


def sim(step, pose, steps=4000):
    """Unicycle simulation; also checks command limits on every step."""
    x, y, yaw = pose
    lim = Limits()
    for i in range(steps):
        v, w, done = step((x, y, yaw))
        assert abs(v) <= lim.max_v + 1e-9 and abs(w) <= lim.max_w + 1e-9
        if done:
            return (x, y, yaw), i
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = wrap(yaw + w * DT)
    raise AssertionError('did not converge')


@pytest.mark.parametrize('goal', [(1.0, 0.0), (-1.0, 0.5), (0.3, -1.2), (0.0, 0.5)])
def test_goto_converges(goal):
    (x, y, _), _ = sim(lambda p: goto_step(p, goal, None, 0.05, Limits()), (0.0, 0.0, 0.0))
    assert math.hypot(goal[0] - x, goal[1] - y) <= 0.06


def test_goto_final_yaw():
    (_, _, yaw), _ = sim(lambda p: goto_step(p, (0.5, 0.5), math.pi / 2, 0.05, Limits()),
                         (0.0, 0.0, 0.0))
    assert abs(wrap(yaw - math.pi / 2)) <= 0.06


def test_goto_speed_cap():
    seen = []
    sim(lambda p: (lambda r: (seen.append(r[0]), r)[1])(
        goto_step(p, (2, 0), None, 0.05, Limits(), max_v=0.05)), (0, 0, 0))
    assert max(seen) <= 0.05 + 1e-9


@pytest.mark.parametrize('m', [0.5, -0.5])
def test_drive_distance(m):
    d = DriveDistance((0, 0, 0), m, Limits())
    (x, _, _), _ = sim(d.step, (0.0, 0.0, 0.0))
    assert abs(abs(x) - 0.5) <= 0.02 and (x > 0) == (m > 0)


@pytest.mark.parametrize('r', [1.0, -1.0, 4.0, -7.0])
def test_rotate_by_including_multi_turn(r):
    rot = RotateBy(0.0, r, Limits())
    sim(rot.step, (0, 0, 0.0))
    assert abs(rot.acc - r) <= 0.04


def test_wrap():
    assert abs(wrap(3 * math.pi) - math.pi) < 1e-9 or abs(wrap(3 * math.pi) + math.pi) < 1e-9
