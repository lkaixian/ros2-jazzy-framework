import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from floor_plane import depth_points, fit_floor, pitch_roll  # noqa: E402


def synth(height, pitch, roll, w=640, h=480, f=380.0):
    """Depth image (m) of a flat floor seen by a camera `height` above it, pitched/rolled (R = Ry(pitch) Rx(roll))."""
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    R = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])
    v, u = np.mgrid[0:h, 0:w]
    ray = np.stack([np.ones((h, w)), -(u - w / 2) / f, -(v - h / 2) / f], -1)      # camera_link, x = depth along the optical axis
    wz = ray @ R[2]                                                                 # world z of each ray
    return np.where(wz < -1e-6, -height / np.where(wz < -1e-6, wz, -1.0), 0.0), (f, w, h)


def test_recovers_height_pitch_roll():
    for height, pitch, roll in [(0.30, 0.0, 0.0), (0.30, 0.2, 0.0), (0.45, 0.35, -0.05)]:
        depth, (f, w, h) = synth(height, pitch, roll)
        k = [f, 0, w / 2, 0, f, h / 2, 0, 0, 1]
        n, d, frac = fit_floor(depth_points(depth, 1.0, k))
        p, r = pitch_roll(n)
        assert abs(d - height) < 0.01 and abs(p - pitch) < 0.01 and abs(r - roll) < 0.01 and frac > 0.9, (height, pitch, roll, d, p, r)
