import math

import pytest

from bt_common.obstacle import min_range_in_sector, ScanGuard

# 8 beams, 45 degrees apart, from -180 deg: index 4 is straight ahead (0 rad), index 0 straight behind
INC = math.pi / 4
RANGES = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]


def r(**override):
    out = list(RANGES)
    for k, v in override.items():
        out[int(k[1:])] = v
    return out


def test_obstacle_ahead_and_behind():
    assert min_range_in_sector(r(i4=0.4), -math.pi, INC, 0.0, math.radians(30)) == 0.4
    assert min_range_in_sector(r(i4=0.4), -math.pi, INC, math.pi, math.radians(30)) == 3.0   # not behind us
    assert min_range_in_sector(r(i0=0.5), -math.pi, INC, math.pi, math.radians(30)) == 0.5   # wraps at +-pi


def test_beams_outside_the_sector_are_ignored():
    assert min_range_in_sector(r(i6=0.2), -math.pi, INC, 0.0, math.radians(30)) == 3.0       # 90 deg to the side


def test_no_echo_values_are_not_obstacles():
    rs = r(i4=float('inf'), i3=float('nan'), i5=0.0)
    assert min_range_in_sector(rs, -math.pi, INC, 0.0, math.radians(50)) is None
    assert min_range_in_sector([], 0.0, INC, 0.0, 1.0) is None


class Scan:
    def __init__(self, ranges):
        self.ranges, self.angle_min, self.angle_increment = ranges, -math.pi, INC
        self.range_min, self.range_max = 0.05, 12.0


def test_scan_guard_blocks_by_direction_and_staleness():
    now = [0.0]
    g = ScanGuard(stop_distance=0.3, half_width_deg=30, stale_sec=0.5, clock=lambda: now[0])
    assert g.blocked(0.2) is None                       # no scan, not required: do not block
    g.update(Scan(r(i4=0.2)))
    assert g.blocked(0.2) == 0.2 and g.blocked(-0.2) is None and g.blocked(0.0) is None
    now[0] = 1.0                                        # scan went stale
    assert g.blocked(0.2) is None
    assert ScanGuard(required=True, clock=lambda: 0.0).blocked(0.2) == 0.0    # required: no scan = blocked


def test_scan_guard_turning_clearance_looks_in_every_direction():
    g = ScanGuard(clock=lambda: 0.0)
    g.update(Scan(r(i6=0.15)))                       # something 15 cm away at the side
    assert g.too_close(0.22) == 0.15 and g.too_close(0.1) is None and g.too_close(0.0) is None
    g.update(Scan(r(i6=float('inf'), i0=0.0)))       # no echo / zero returns are not obstacles
    assert g.too_close(0.22) is None


def test_freer_side_turns_toward_the_side_with_more_room():
    now = [0.0]
    g = ScanGuard(stale_sec=0.5, clock=lambda: now[0])
    assert g.freer_side() == 0                          # no scan: the caller just stops
    g.update(Scan(r(i4=0.3, i5=0.5)))                   # wall ahead, something close on the left (index 5 = +45 deg)
    assert g.freer_side() == -1
    g.update(Scan(r(i4=0.3, i3=0.5)))                   # the same on the right (index 3 = -45 deg)
    assert g.freer_side() == 1
    g.update(Scan(r(i4=0.3)))                           # open on both sides: left, like the bump recovery
    assert g.freer_side() == 1
    now[0] = 1.0                                        # stale
    assert g.freer_side() == 0


def test_slow_factor_eases_from_full_speed_to_stop_and_clearance_ignores_the_stop_distance():
    from types import SimpleNamespace as NS
    clock = [0.0]
    g = ScanGuard(stop_distance=0.4, half_width_deg=35.0, stale_sec=0.5, clock=lambda: clock[0])
    assert g.clearance(0.1) is None and g.slow_factor(0.1) == 1.0                 # no scan yet, not required: no opinion
    n = 36
    g.update(NS(ranges=[0.55] * n, angle_min=-3.14, angle_increment=6.28 / n, range_min=0.05, range_max=12.0))
    assert g.clearance(0.1) == 0.55 and g.blocked(0.1) is None
    assert g.slow_factor(0.1, ramp=0.3) == pytest.approx(0.5)
    g.update(NS(ranges=[0.3] * n, angle_min=-3.14, angle_increment=6.28 / n, range_min=0.05, range_max=12.0))
    assert g.slow_factor(0.1) == 0.0 and g.blocked(0.1) == 0.3
    assert g.slow_factor(0.0) == 1.0                                              # not moving: nothing to ease
