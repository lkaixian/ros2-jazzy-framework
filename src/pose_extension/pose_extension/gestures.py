"""Gesture rules on COCO-17 keypoints in image coordinates (y grows downward). ROS-free."""
import math

L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 5, 6, 7, 8, 9, 10
SIDES = {'L': (L_SH, L_EL, L_WR), 'R': (R_SH, R_EL, R_WR)}


def point(kps, idx, min_conf):
    """kps: list of (x, y, conf). Returns (x, y) or None when missing/low confidence."""
    if idx >= len(kps) or kps[idx][2] < min_conf:
        return None
    return kps[idx][0], kps[idx][1]


def shoulder_width(kps, min_conf):
    a, b = point(kps, L_SH, min_conf), point(kps, R_SH, min_conf)
    if not a or not b:
        return None
    w = math.hypot(a[0] - b[0], a[1] - b[1])
    return w if w > 1e-3 else None


def has_pose(kps, min_conf):
    return shoulder_width(kps, min_conf) is not None


def pointing(kps, min_conf=0.5, direction=''):
    """Arm held out roughly horizontally and straight -> 'left'/'right' (image side) or None."""
    sw = shoulder_width(kps, min_conf)
    if sw is None:
        return None
    for side, (si, ei, wi) in SIDES.items():
        s, e, wr = (point(kps, i, min_conf) for i in (si, ei, wi))
        if not (s and e and wr):
            continue
        reach, drop = wr[0] - s[0], wr[1] - s[1]
        if abs(reach) < 1.0 * sw or abs(drop) > 0.6 * sw:
            continue
        v1 = (s[0] - e[0], s[1] - e[1])
        v2 = (wr[0] - e[0], wr[1] - e[1])
        n = math.hypot(*v1) * math.hypot(*v2)
        if n < 1e-6:
            continue
        cosang = (v1[0] * v2[0] + v1[1] * v2[1]) / n
        if cosang > math.cos(math.radians(150)):   # elbow angle must exceed ~150 deg (straight)
            continue
        d = 'right' if reach > 0 else 'left'
        if not direction or direction == d:
            return d
    return None


def waving(history, min_conf=0.5):
    """history: [(t, kps)] over the window. Wrist above shoulder and swinging sideways."""
    for side, (si, _ei, wi) in SIDES.items():
        xs, up = [], 0
        for _t, kps in history:
            s, wr = point(kps, si, min_conf), point(kps, wi, min_conf)
            sw = shoulder_width(kps, min_conf)
            if s and wr and sw:
                xs.append((wr[0] - s[0]) / sw)
                up += wr[1] < s[1]
        if len(xs) < 6 or up < 0.8 * len(xs):
            continue
        # count direction reversals whose swing exceeds 0.25 shoulder widths
        reversals, direction, anchor = 0, 0, xs[0]
        for x in xs[1:]:
            if direction >= 0 and x - anchor < -0.25:
                reversals, direction, anchor = reversals + (direction != 0), -1, x
            elif direction <= 0 and x - anchor > 0.25:
                reversals, direction, anchor = reversals + (direction != 0), 1, x
            elif (direction > 0 and x > anchor) or (direction < 0 and x < anchor):
                anchor = x
        if reversals >= 2:
            return True
    return False
