"""Arm-eye geometry: image pixel + depth -> point in the arm base frame, through the hand-measured camera pose on the gripper.

Frames: raw optical (x right, y down, z forward in the SENSOR image, which is upside down on the arm), gripper = end_effector_link
(x forward, y left, z up), arm base = omx_fk's `world`. The detection nodes see the image rotated upright by `image_rotate_deg`;
a pixel in that image must be turned back to the raw image before the raw depth is read.
IDEAL CASE: +-5 mm hand measurement, unverified signs.
"""
import os

import numpy as np
import yaml

R_G_C = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])      # columns = raw optical x, y, z expressed in the gripper frame


def load(path=None):
    if path is None:
        src = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'framework_bringup', 'param')
        if not os.path.isdir(src):            # installed: the param dir is in framework_bringup's share
            from ament_index_python.packages import get_package_share_directory
            src = os.path.join(get_package_share_directory('framework_bringup'), 'param')
        path = os.path.join(src, 'arm_hand_eye.yaml')
    with open(path) as f:
        c = yaml.safe_load(f)['arm_hand_eye']
    k = c['rgb_intrinsics_1280x720']
    K = np.array([[k['fx'], 0, k['cx']], [0, k['fy'], k['cy']], [0, 0, 1]], float)
    return np.array(c['position_m'], float), K, int(c['image_rotate_deg'])


def raw_pixel(u, v, width, height, rotate):
    """Pixel (u, v) of the image turned clockwise by `rotate` degrees -> pixel in the raw image (width x height)."""
    if rotate == 0:
        return u, v
    if rotate == 180:
        return width - 1 - u, height - 1 - v
    if rotate == 90:           # upright (u', v') = (h-1-v, u)
        return v, height - 1 - u
    if rotate == 270:          # upright (u', v') = (v, w-1-u)
        return width - 1 - v, u
    raise ValueError(f'rotate must be 0, 90, 180 or 270, got {rotate}')


def box_depth_m(depth_mm, box, width, height, rotate, frac=0.4, band_mm=40, min_pixels=150):
    """Depth (m) of a detection box (x0, y0, x1, y1 in the UPRIGHT image) from the raw depth image; None if no band has enough pixels.

    Reads the central `frac` of the box, not one point: a bottle's centre is often a stereo hole. Returns the median of the NEAREST
    band of `band_mm` that holds at least `min_pixels` valid pixels. A clear bottle gives only ~5% valid pixels on itself and the rest is
    the wall behind it (measured: bottle 0.39 m, 10th pct 0.5 m, median 1-1.8 m), so no fixed percentile works; sparse speckle in front
    does not reach `min_pixels`.
    """
    h, w = depth_mm.shape
    (xa, ya), (xb, yb) = (raw_pixel(box[0], box[1], width, height, rotate), raw_pixel(box[2], box[3], width, height, rotate))
    cx, cy, hw, hh = (xa + xb) / 2, (ya + yb) / 2, abs(xb - xa) * frac / 2, abs(yb - ya) * frac / 2
    sx, sy = w / width, h / height
    min_pixels = max(20, int(min_pixels * w * h / (1280 * 720)))      # tuned on 1280x720; a smaller depth image has fewer pixels on the object
    win = depth_mm[max(0, int((cy - hh) * sy)):int((cy + hh) * sy) + 1, max(0, int((cx - hw) * sx)):int((cx + hw) * sx) + 1]
    v = np.sort(win[(win > 0) & (win < 20000)])
    if v.size < min_pixels:
        return None
    hi = np.searchsorted(v, v + band_mm, side='right')
    ok = np.nonzero(hi - np.arange(v.size) >= min_pixels)[0]
    return float(np.median(v[ok[0]:hi[ok[0]]])) / 1000.0 if ok.size else None


def clipped_at_side(box, width, margin=3):
    """'left' / 'right' when a detection box touches that side of the picture, else ''. Cut off sideways, the centre (so the sideways position
    the gripper needs) is biased. Top / bottom do not count: a tall object at close range always fills the 42 deg vertical view, and that only
    shifts the estimated height a little."""
    return 'left' if box[0] < margin else 'right' if box[2] > width - 1 - margin else ''


def fixed_cam_point(u, v, depth_m, k, xyz):
    """Pixel (u, v) + depth of a camera bolted to the robot, looking forward (parallel to the arm's x axis, upright image) -> arm base frame.
    k = (fx, fy, cx, cy); xyz = the lens in the arm frame (x forward, y left, z up), hand-measured. Optical: x right, y down, z forward."""
    fx, fy, cx, cy = k
    return np.array([depth_m + xyz[0], -(u - cx) * depth_m / fx + xyz[1], -(v - cy) * depth_m / fy + xyz[2]])


def backproject(u, v, depth_m, k):
    """Raw pixel + depth along the optical axis (metres) -> point in the raw optical frame."""
    return np.array([(u - k[0, 2]) * depth_m / k[0, 0], (v - k[1, 2]) * depth_m / k[1, 1], depth_m])


def optical_to_gripper(p_opt, position):
    return R_G_C @ np.asarray(p_opt, float) + position


def to_arm(arm, q, p_opt, position):
    """Point in the raw optical frame -> arm base frame, for joint angles q = {'joint1': .., ..}."""
    m = arm.pose('end_effector_link', q)
    return (m @ np.append(optical_to_gripper(p_opt, position), 1.0))[:3]


def upright_pixel_to_arm(arm, q, u, v, depth_m, width=1280, height=720):
    """A detection pixel in the UPRIGHT image plus the raw depth at its raw pixel -> arm-frame point."""
    position, k, rotate = load()
    ur, vr = raw_pixel(u, v, width, height, rotate)
    return to_arm(arm, q, backproject(ur, vr, depth_m, k), position)
