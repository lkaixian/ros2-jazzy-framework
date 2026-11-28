"""Body parts from the pose keypoints, and a posture-independent colour fingerprint of each (no ROS imports).

A whole-box appearance vector changes when a person turns, sits or stands close; the colour of their shirt and trousers does not.
shortcut: only torso and legs, colour only; head/hair and body size are left out (a turned person's shoulders look half as wide, and the
hips are out of the picture at following distance). Upgrade to per-part network embeddings if colour alone is not enough on real footage.
"""
import cv2
import numpy as np

SHOULDERS, HIPS, KNEES, ANKLES = (5, 6), (11, 12), (13, 14), (15, 16)      # COCO-17 keypoint indices


def part_boxes(box, kx, ky, kc, conf=0.4, min_px=8):
    """{'torso': (x1, y1, x2, y2), 'legs': ...} for the parts whose keypoints are confident (a part that cannot be seen is absent).

    Torso: the shoulders down to the hips, or down to the bottom of the person box when the hips are out of the picture (a close
    person). Legs: the hips down to the lowest confident knee or ankle."""
    if kx is None or len(kx) < 17 or len(kc) < 17:
        return {}
    ok = lambda ids: all(kc[i] >= conf for i in ids)      # noqa: E731
    out = {}
    if ok(SHOULDERS):
        hips = ok(HIPS)
        ids = SHOULDERS + HIPS if hips else SHOULDERS
        x1, x2 = min(kx[i] for i in ids), max(kx[i] for i in ids)
        y1 = (ky[5] + ky[6]) / 2
        y2 = (ky[11] + ky[12]) / 2 if hips else box[3]
        out['torso'] = (x1, y1, x2, y2)
    if ok(HIPS):
        low = [ky[i] for i in KNEES + ANKLES if kc[i] >= conf]
        if low:
            out['legs'] = (min(kx[11], kx[12]) - 0.1 * abs(kx[11] - kx[12]), (ky[11] + ky[12]) / 2, max(kx[11], kx[12]) + 0.1 * abs(kx[11] - kx[12]), max(low))
    return {k: b for k, b in out.items() if b[2] - b[0] >= min_px and b[3] - b[1] >= min_px}


def histogram(crop):
    """12 x 4 x 4 HSV histogram of the middle of a BGR crop (the edges are background and arms), summing to 1."""
    h, w = crop.shape[:2]
    c = crop[int(0.15 * h):max(int(0.15 * h) + 1, int(0.85 * h)), int(0.15 * w):max(int(0.15 * w) + 1, int(0.85 * w))]
    hist = cv2.calcHist([cv2.cvtColor(c, cv2.COLOR_BGR2HSV)], [0, 1, 2], None, [12, 4, 4], [0, 180, 0, 256, 0, 256]).ravel()
    return hist / (hist.sum() + 1e-9)


def part_histograms(frame, box, kx, ky, kc):
    """{'torso': histogram, 'legs': histogram} of the parts visible in this frame."""
    ih, iw = frame.shape[:2]
    out = {}
    for name, (x1, y1, x2, y2) in part_boxes(box, kx, ky, kc).items():
        x1, x2, y1, y2 = int(max(0, x1)), int(min(iw, x2)), int(max(0, y1)), int(min(ih, y2))
        if x2 - x1 >= 8 and y2 - y1 >= 8:
            out[name] = histogram(frame[y1:y2, x1:x2])
    return out


def colour_similarity(a, b):
    """Bhattacharyya coefficient of two histograms: 1 = same colours, 0 = nothing in common."""
    return float(np.sqrt(a * b).sum())
