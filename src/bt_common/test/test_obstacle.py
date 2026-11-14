import math

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
