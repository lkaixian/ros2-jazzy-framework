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
