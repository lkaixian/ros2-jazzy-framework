import math
import os

import numpy as np
import pytest

from nav_sim.kinematics import approach, Robot, wrap
from nav_sim.occupancy import OccupancyMap

MAP_YAML = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup',
                                         'maps', 'my_map.yaml'))


def box_map(n=100, res=0.1, origin=(0.0, 0.0)):
    """n x n cells, free inside, one-cell wall around."""
    b = np.zeros((n, n), dtype=bool)
    b[0, :] = b[-1, :] = b[:, 0] = b[:, -1] = True
    return OccupancyMap(b, res, origin)


def test_world_to_cell_flips_y_and_origin():
    m = box_map(10, 1.0, (5.0, -2.0))
    row, col = m.to_cell(5.5, -1.5)             # first cell column, bottom image row
    assert (int(col), int(row)) == (0, 9)
    row, col = m.to_cell(14.5, 7.5)
    assert (int(col), int(row)) == (9, 0)


def test_outside_map_is_blocked_inside_is_free():
    m = box_map()
    assert m.is_blocked(-1.0, 5.0) and m.is_blocked(5.0, 100.0)
    assert not m.is_blocked(5.0, 5.0)


def test_raycast_distance_to_wall_and_range_limit():
    m = box_map()                                # free area x in [0.1, 9.9]
    d = m.raycast(5.0, 5.0, 0.0, np.array([0.0, math.pi / 2, math.pi]), 0.1, 20.0)
    assert d[0] == pytest.approx(4.9, abs=0.06)
    assert d[2] == pytest.approx(4.9, abs=0.06)
    short = m.raycast(5.0, 5.0, 0.0, np.array([0.0]), 0.1, 2.0)
    assert np.isinf(short[0])                    # nothing within 2 m


def test_raycast_is_rotation_consistent():
    m = box_map()
    a = m.raycast(3.0, 6.0, 0.0, np.array([math.pi / 2]), 0.1, 20.0)
    b = m.raycast(3.0, 6.0, math.pi / 2, np.array([0.0]), 0.1, 20.0)
    assert a[0] == pytest.approx(b[0])


def test_thin_wall_is_not_skipped():
    b = np.zeros((100, 100), dtype=bool)
    b[:, 50] = True                              # one-cell wall at x in [5.0, 5.1)
    m = OccupancyMap(b, 0.1, (0.0, 0.0))
    d = m.raycast(1.0, 5.0, 0.0, np.array([0.0]), 0.1, 20.0)
    assert d[0] == pytest.approx(4.0, abs=0.06)


def test_circle_collision():
    m = box_map()
    assert not m.collides(5.0, 5.0, 0.17)
    assert m.collides(0.2, 5.0, 0.17)            # touches the left wall
    assert m.collides(-5.0, 5.0, 0.1)            # outside


@pytest.mark.skipif(not os.path.exists(MAP_YAML), reason='sample map not installed')
def test_real_sample_map_loads_and_has_free_space():
    m = OccupancyMap.from_yaml(MAP_YAML)
    assert (m.w, m.h) == (418, 126) and m.res == pytest.approx(0.05)
    assert 0.2 < (~m.blocked).mean() < 0.9      # some free, some wall/unknown
    # the wall border/unknown outside means a far-away point is blocked
    assert m.collides(-50.0, 0.0, 0.1)


# ------------------------------------------------------------------ kinematics
def test_wrap_and_approach():
    assert wrap(3 * math.pi) == pytest.approx(math.pi) or wrap(3 * math.pi) == pytest.approx(-math.pi)
    assert approach(0.0, 1.0, 0.3) == pytest.approx(0.3)
    assert approach(0.9, 1.0, 0.3) == 1.0


def test_acceleration_limit_is_respected():
    r = Robot(max_lin_acc=1.0)
    r.command(0.5, 0.0)
    r.step(0.1)
    assert r.v == pytest.approx(0.1)


def test_command_timeout_stops_the_robot_like_the_kobuki():
    r = Robot(cmd_timeout=0.6, max_lin_acc=100.0)
    r.command(0.3, 0.0)
    for _ in range(5):
        r.step(0.1)
    assert r.v == pytest.approx(0.3)
    for _ in range(8):                           # no new command for 0.8 s
        r.step(0.1)
    assert r.v == pytest.approx(0.0)


def test_odom_frame_origin_is_the_start_pose():
    r = Robot(x=2.0, y=3.0, yaw=math.pi / 2, max_lin_acc=100.0)
    r.set_start()
    r.command(0.5, 0.0)
    for _ in range(20):
        r.command(0.5, 0.0)
        r.step(0.1)
    assert r.y - 3.0 == pytest.approx(1.0, abs=1e-6) and r.x == pytest.approx(2.0, abs=1e-6)
    assert r.odom_x == pytest.approx(1.0, abs=1e-6) and r.odom_y == pytest.approx(0.0, abs=1e-6)
    assert r.odom_yaw == pytest.approx(0.0, abs=1e-9)


def test_collision_blocks_translation_but_not_rotation():
    r = Robot(x=1.0, y=1.0, max_lin_acc=100.0, max_ang_acc=100.0)
    r.set_start()
    hit = False
    for _ in range(10):
        r.command(1.0, 0.5)
        hit |= r.step(0.1, collides=lambda x, y: x > 1.05)
    assert hit and r.x <= 1.05 + 1e-9
    assert abs(r.yaw) > 0.1                      # still turned


def test_yaw_wraps_and_odom_yaw_is_continuous():
    r = Robot(max_ang_acc=100.0)
    r.set_start()
    for _ in range(100):
        r.command(0.0, 1.0)
        r.step(0.1)
    assert -math.pi <= r.yaw <= math.pi
    assert r.odom_yaw == pytest.approx(wrap(10.0), abs=0.05)
