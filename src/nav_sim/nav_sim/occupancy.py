"""Occupancy grid loaded from a ROS map_server YAML + PGM; ray casting and collision checks.

Pure numpy/cv2, no ROS. World coordinates are metres in the map frame; the image row 0 is the
top, so row = H - 1 - floor((y - oy) / res).
"""
import math
import os

import cv2
import numpy as np
import yaml


class OccupancyMap:
    def __init__(self, blocked, resolution, origin):
        """blocked: HxW bool array (True = occupied OR unknown: rays stop, robot cannot enter)."""
        self.blocked = np.asarray(blocked, dtype=bool)
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = self.blocked.shape

    # ---- construction -------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(path)), img_path)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f'cannot read map image {img_path}')
        p = img.astype(np.float32) / 255.0
        occ = (1.0 - p) if not meta.get('negate', 0) else p     # occupancy probability
        free_thresh = float(meta.get('free_thresh', 0.196))
        # free only when clearly free: unknown (grey) and occupied both block, so the simulated
        # robot never leaves the mapped area and the lidar never sees "through" unknown space
        blocked = occ >= free_thresh - 0.01      # unknown (205 -> 0.196) is NOT free
        return cls(blocked, meta['resolution'], meta['origin'])

    # ---- coordinates --------------------------------------------------------------------
    def to_cell(self, x, y):
        """Vectorised world -> (row, col); out-of-map cells are returned as is (check inside())."""
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        row = self.h - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        return row, col

    def inside(self, row, col):
        return (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)

    def is_blocked(self, x, y):
        """Blocked mask with the shape of x/y (arrays of any shape). Outside the map = blocked."""
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        row, col = self.to_cell(x, y)
        ok = self.inside(row, col)
        out = np.ones(x.shape, dtype=bool)
        out[ok] = self.blocked[row[ok], col[ok]]
        return out

    # ---- robot footprint ----------------------------------------------------------------
    def collides(self, x, y, radius):
        """True if a circle of `radius` at (x, y) touches a blocked cell (sampled boundary)."""
        angles = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
        xs = np.concatenate(([x], x + radius * np.cos(angles)))
        ys = np.concatenate(([y], y + radius * np.sin(angles)))
        return bool(np.any(self.is_blocked(xs, ys)))

    def free_point(self, x, y, radius):
        return not self.collides(x, y, radius)

    # ---- lidar --------------------------------------------------------------------------
    def raycast(self, x, y, yaw, angles, range_min, range_max, step=None):
        """Range per beam (np.inf when nothing is hit within range_max).

        angles are relative to `yaw`. A beam steps in `step` increments (default res / 2),
        so a thin wall one cell thick is never skipped.
        """
        step = step or self.res / 2.0
        r = np.arange(range_min, range_max + step, step)
        a = yaw + np.asarray(angles)
        px = x + np.outer(np.cos(a), r)
        py = y + np.outer(np.sin(a), r)
        hit = self.is_blocked(px, py)
        first = np.argmax(hit, axis=1)
        any_hit = hit[np.arange(len(a)), first]
        out = np.where(any_hit, r[first], np.inf)
        out[out > range_max] = np.inf
        return out
