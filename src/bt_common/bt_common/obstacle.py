"""Nearest obstacle in a sector of a LaserScan, and the guard motion extensions use to stop for it."""
import math
import time


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def min_range_in_sector(ranges, angle_min, angle_inc, centre, half_width, rmin=0.05, rmax=12.0):
    """Smallest valid range whose beam lies within +-half_width rad of `centre` (robot frame, rad).

    Beams that are nan/inf or outside [rmin, rmax] are ignored: a 0.0 or inf return is "no echo", not "wall at 0 m".
    Returns None when no valid beam falls in the sector.
    """
    best = None
    for i, r in enumerate(ranges):
        if not (rmin <= r <= rmax):          # also false for nan
            continue
        if abs(wrap(angle_min + i * angle_inc - centre)) <= half_width and (best is None or r < best):
            best = r
    return best


class ScanGuard:
    """Holds the latest LaserScan and answers "is something closer than stop_distance in the direction of travel?".

    blocked(v) -> the distance (m) or None. With no fresh scan: None, unless `required` (then 0.0 = blocked), so lidar-less
    sims keep working and the real robot can refuse to drive blind.
    """

    def __init__(self, stop_distance=0.30, half_width_deg=35.0, stale_sec=0.5, required=False, yaw_offset=0.0,
                 clock=time.monotonic):
        self.stop, self.half, self.stale = stop_distance, math.radians(half_width_deg), stale_sec
        self.required, self.yaw_offset, self.clock = required, yaw_offset, clock
        self.scan, self.t = None, 0.0

    def update(self, msg):
        self.scan, self.t = msg, self.clock()

    def blocked(self, v):
        if v == 0.0:
            return None
        if self.scan is None or self.clock() - self.t > self.stale:
            return 0.0 if self.required else None
        s = self.scan
        d = min_range_in_sector(s.ranges, s.angle_min, s.angle_increment, (0.0 if v > 0 else math.pi) - self.yaw_offset,
                                self.half, max(s.range_min, 0.05), s.range_max or 12.0)
        return d if d is not None and d < self.stop else None

    def too_close(self, clearance):
        """Turning in place sweeps the whole footprint: the nearest valid beam in ANY direction closer than `clearance` m
        (None = clear, or guard off with clearance <= 0, or no fresh scan and not required)."""
        if clearance <= 0:
            return None
        if self.scan is None or self.clock() - self.t > self.stale:
            return 0.0 if self.required else None
        s = self.scan
        lo, hi = max(s.range_min, 0.05), s.range_max or 12.0
        d = min((r for r in s.ranges if lo <= r <= hi), default=None)
        return d if d is not None and d < clearance else None
