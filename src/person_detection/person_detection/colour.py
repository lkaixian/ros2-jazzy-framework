"""Clothes colour of a detected person from one BGR frame (no ROS imports)."""
import cv2
import numpy as np

SHOULDERS, HIPS = (5, 6), (11, 12)      # COCO-17 keypoint indices


def torso_box(box, kx=None, ky=None, kc=None, min_conf=0.5):
    """(x1, y1, x2, y2) of the torso: shoulders to hips when all four keypoints are confident, else the middle of the person box."""
    x1, y1, x2, y2 = box
    if kx is not None and len(kx) >= 13 and all(kc[i] >= min_conf for i in SHOULDERS + HIPS):
        xs = [kx[i] for i in SHOULDERS + HIPS]
        ys = [ky[i] for i in SHOULDERS + HIPS]
        return min(xs), min(ys), max(xs), max(ys)
    w, h = x2 - x1, y2 - y1
    return x1 + 0.25 * w, y1 + 0.25 * h, x2 - 0.25 * w, y1 + 0.6 * h


def colour_name(h, s, v):
    """OpenCV HSV (h 0-179, s/v 0-255) -> a plain colour word."""
    if v < 50:
        return 'black'
    if s < 40:
        return 'white' if v > 190 else 'grey'
    if h < 8 or h >= 170:
        return 'red'
    for top, name in ((20, 'orange'), (35, 'yellow'), (85, 'green'), (100, 'cyan'), (130, 'blue'), (150, 'purple'), (170, 'pink')):
        if h < top:
            return name
    return 'red'


def clothes_colour(frame, box, kx=None, ky=None, kc=None):
    """The colour word of the torso region, '' if the region is too small to judge."""
    x1, y1, x2, y2 = torso_box(box, kx, ky, kc)
    ih, iw = frame.shape[:2]
    x1, x2 = int(max(0, x1)), int(min(iw, x2))
    y1, y2 = int(max(0, y1)), int(min(ih, y2))
    if x2 - x1 < 6 or y2 - y1 < 6:
        return ''
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h, s, v = (int(np.median(hsv[:, i])) for i in range(3))     # median: robust to arms, background and a bag strap
    return colour_name(h, s, v)
