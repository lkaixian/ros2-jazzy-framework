import math

import numpy as np
import pytest

from nav_sim.actors import Actor, box_touches, detections, load_scenario, overhead_cloud, ray_circles
from nav_sim.occupancy import OccupancyMap


def free_map(n=200, res=0.1):
    return OccupancyMap(np.zeros((n, n), bool), res, (0.0, 0.0))


def test_actor_walks_its_path_at_its_speed_then_stops():
    a = Actor('p', 1, [(0, 0), (1, 0), (1, 1)], speed=0.5)
    for _ in range(40):
        a.step(0.05)                                   # 2 s -> 1.0 m: the corner
    assert (a.x, a.y) == pytest.approx((1.0, 0.0), abs=1e-6)
    for _ in range(400):
        a.step(0.05)
    assert (a.x, a.y) == pytest.approx((1.0, 1.0), abs=1e-6)


def test_actor_waits_then_loops_back_and_forth():
    a = Actor('p', 1, [(0, 0), (1, 0)], speed=1.0, start_after=1.0, loop=True)
    a.step(0.9)
    assert a.x == 0.0
    a.step(0.2 + 1.0)                                  # 0.1 s late start left + 1 s: at the end
    a.step(0.5)                                        # and 0.5 m back
    assert 0.0 < a.x < 1.0


def test_ray_circles_hits_front_misses_side():
    d = ray_circles(0, 0, np.array([0.0, math.pi / 2]), [(2.0, 0.0, 0.25)], 10)
    assert d[0] == pytest.approx(1.75) and math.isinf(d[1])


def test_person_straight_ahead_is_centred_with_depth_and_left_goes_left_of_image():
    a = Actor('p', 7, [(4.16, 1.92)], colour='red')    # 3 m ahead of the camera (robot at 1, 2; camera at x +0.16, y -0.08 of it)
    d = detections(1, 2, 0, [a], free_map())[0]
    assert d['track_id'] == 7 and d['colour'] == 'red' and d['distance_m'] == pytest.approx(3.0, abs=0.01)
    assert (d['x_min'] + d['x_max']) / 2 == pytest.approx(320, abs=1)
    a.y += 0.5                                         # to the robot's left -> left half of the image
    d = detections(1, 2, 0, [a], free_map())[0]
    assert (d['x_min'] + d['x_max']) / 2 < 320


def test_person_behind_robot_or_behind_wall_or_hidden_is_not_seen():
    omap = free_map()
    assert detections(5, 5, 0, [Actor('p', 1, [(2, 5)])], omap) == []        # behind
    assert detections(5, 5, 0, [Actor('p', 1, [(5.2, 5)])], omap) == []      # closer than the minimum range
    wall = np.zeros((200, 200), bool)
    wall[:, 80] = True                                                         # a wall at x = 8.0
    assert detections(5, 5, 0, [Actor('p', 1, [(11, 5)])], OccupancyMap(wall, 0.1, (0, 0))) == []
    near, far = Actor('n', 1, [(7, 5.08)]), Actor('f', 2, [(9, 5.08)])
    assert [d['track_id'] for d in detections(5, 5, 0, [near, far], omap)] == [1]


def test_overhead_box_is_in_the_cloud_at_its_height_and_the_floor_is_there_too():
    boxes = [dict(x=2.16, y=-0.08, w=0.4, d=0.4, z0=0.55, z1=0.8)]
    p = overhead_cloud(0, 0, 0, boxes, floor=False)
    assert len(p) > 20 and p[:, 0].min() == pytest.approx(1.8, abs=0.1)
    assert p[:, 2].max() == pytest.approx(0.8 - 0.755, abs=0.02)             # z is relative to the camera
    f = overhead_cloud(0, 0, 0, [], floor=True)
    assert len(f) > 50 and np.allclose(f[:, 2], -0.755)
    assert len(overhead_cloud(0, 0, math.pi, boxes, floor=False)) == 0       # behind the camera


def test_scenario_file_roundtrip_and_errors(tmp_path):
    f = tmp_path / 's.yaml'
    f.write_text('actors:\n  - {name: a, id: 3, path: [[1, 1], [2, 1]], speed: 0.2}\noverhead:\n  - {x: 1, y: 1, w: .5, d: .5, z0: .5, z1: .8}\n')
    actors, boxes = load_scenario(str(f))
    assert actors[0].track_id == 3 and boxes[0]['z1'] == 0.8
    assert load_scenario('') == ([], [])
    f.write_text('actors:\n  - {name: a}\n')
    with pytest.raises(ValueError, match='actor #0'):
        load_scenario(str(f))


def test_box_touches_only_when_the_robot_overlaps_the_footprint_and_the_box_is_low_enough():
    b = [dict(x=2.0, y=0.0, w=0.4, d=0.4, z0=0.55, z1=0.8)]
    assert box_touches(1.7, 0.0, 0.17, b) and not box_touches(1.5, 0.0, 0.17, b)
    assert not box_touches(1.7, 0.0, 0.17, [dict(b[0], z0=1.2, z1=1.4)])      # a shelf above the robot's top: it drives under


def test_actor_waits_instead_of_walking_into_the_robot_and_goes_on_when_it_leaves():
    a = Actor('p', 1, [(0, 0), (3, 0)], speed=1.0)
    for _ in range(20):
        a.step(0.05, robot=(1.5, 0.0, 0.17))          # a robot standing on the path
    assert a.x < 1.5 - 0.22 - 0.17 + 1e-6
    x = a.x
    for _ in range(20):
        a.step(0.05, robot=(1.5, 2.0, 0.17))          # robot gone from the path
    assert a.x > x + 0.5
