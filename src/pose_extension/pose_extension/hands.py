"""Hand landmarks and finger counting for the five-finger stop sign. ROS-free, onnxruntime + OpenCV.

Two ONNX models from the OpenCV model zoo (opencv/opencv_zoo, Apache-2.0; MediaPipe's palm detector and hand landmark network): the palm detector finds a hand in a
crop, the landmark network gives its 21 points; a finger counts as extended when its tip is clearly further from the wrist than its middle joint. The pre- and
post-processing follow the zoo's mp_palmdet.py / mp_handpose.py (anchors are generated here instead of stored; they match the zoo's table).
Put palm_detection_mediapipe_2023feb.onnx and handpose_estimation_mediapipe_2023feb.onnx in models/hand/ (docs/perception.md). Accuracy on THIS camera is unmeasured:
tools/hand/check.py shows what the robot sees.
"""
import math
import os

import cv2
import numpy as np

PALM, POSE = 'palm_detection_mediapipe_2023feb.onnx', 'handpose_estimation_mediapipe_2023feb.onnx'
FINGERS = ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20))      # (knuckle, middle joint, tip) of index, middle, ring, little finger


def find_models(folder=''):
    """(palm path, pose path) in `folder` or the nearest models/hand, else ('', '')."""
    d = folder or os.path.dirname(os.path.realpath(__file__))
    for _ in range(0 if folder else 8):
        p = os.path.join(d, 'models', 'hand')
        if os.path.isfile(os.path.join(p, PALM)) and os.path.isfile(os.path.join(p, POSE)):
            return os.path.join(p, PALM), os.path.join(p, POSE)
        d = os.path.dirname(d)
    if folder and os.path.isfile(os.path.join(folder, PALM)) and os.path.isfile(os.path.join(folder, POSE)):
        return os.path.join(folder, PALM), os.path.join(folder, POSE)
    return '', ''


def anchors():
    """The 2016 SSD anchor centres of the 192 x 192 palm detector: a 24 x 24 grid with 2 per cell, then a 12 x 12 grid with 6 per cell."""
    return np.array([((x + 0.5) / fm, (y + 0.5) / fm) for fm, per in ((24, 2), (12, 6)) for y in range(fm) for x in range(fm) for _ in range(per)])


def extended_fingers(lm, margin=1.1):
    """Which of thumb, index, middle, ring, little finger are straight, from 21 landmarks (rows of x, y[, z]; MediaPipe order: 0 wrist, 1-4 thumb, 5-8 index ...).
    A finger is extended when its tip is further from the wrist than its middle joint by `margin`; the thumb when its tip is further from the little-finger knuckle
    than its own middle joint. Returns [bool x5]."""
    lm = np.asarray(lm, float)[:, :2]
    d = lambda a, b: float(np.linalg.norm(lm[a] - lm[b]))      # noqa: E731
    out = [d(4, 17) > margin * d(3, 17)]
    out += [d(tip, 0) > margin * d(pip, 0) and d(tip, 0) > d(mcp, 0) for mcp, pip, tip in FINGERS]
    return out


class HandModel:
    def __init__(self, folder=''):
        self.paths, self.palm, self.pose, self.anc = find_models(folder), None, None, anchors()

    @property
    def available(self):
        return bool(self.paths[0])

    def _load(self):
        if self.palm is None:
            if not self.available:
                raise RuntimeError('no hand models: put the two ONNX files in models/hand/ (docs/perception.md)')
            import onnxruntime as ort
            self.palm = ort.InferenceSession(self.paths[0], providers=['CPUExecutionProvider'])
            self.pose = ort.InferenceSession(self.paths[1], providers=['CPUExecutionProvider'])

    def _palms(self, bgr, score_min=0.5):
        """Palm detections in a crop: rows of [x1, y1, x2, y2, 7 landmark points x, y ..., score], pixels of the crop; strongest first."""
        h, w = bgr.shape[:2]
        side = max(h, w)                                              # letterbox to a square, then 192 x 192
        pad = cv2.copyMakeBorder(bgr, (side - h) // 2, side - h - (side - h) // 2, (side - w) // 2, side - w - (side - w) // 2, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        blob = cv2.cvtColor(cv2.resize(pad, (192, 192)), cv2.COLOR_BGR2RGB).astype(np.float32)[None] / 255.0
        raw, score = (np.asarray(o) for o in self.palm.run(None, {'input_1': blob}))
        score = 1.0 / (1.0 + np.exp(-score[0, :, 0].astype(np.float64)))
        delta, lmd = raw[0, :, 0:4], raw[0, :, 4:]
        xy1 = (delta[:, :2] / 192.0 - delta[:, 2:] / 192.0 / 2 + self.anc) * side
        xy2 = (delta[:, :2] / 192.0 + delta[:, 2:] / 192.0 / 2 + self.anc) * side
        boxes = np.concatenate([xy1, xy2], axis=1) - [(side - w) // 2, (side - h) // 2] * 2
        keep = np.asarray(cv2.dnn.NMSBoxes(boxes.tolist(), score.tolist(), score_min, 0.3, top_k=5000)).reshape(-1)
        if len(keep) == 0:
            return np.empty((0, 19))
        lms = (lmd[keep].reshape(-1, 7, 2) / 192.0 + self.anc[keep][:, None, :]) * side - [(side - w) // 2, (side - h) // 2]
        return np.c_[boxes[keep].reshape(-1, 4), lms.reshape(-1, 14), score[keep]]

    def _landmarks(self, bgr, palm, conf_min=0.8):
        """21 landmarks (x, y, z; pixels of `bgr`) of the hand a palm detection describes, rotated upright first; None when the network is not sure it is a hand."""
        def crop(img, box, rotation):
            wh = box[1] - box[0]
            wh_shift = ([0, 0] if rotation else [0, -0.4]) * wh
            box = box + wh_shift
            c, wh = box.sum(axis=0) / 2, box[1] - box[0]
            half = wh * (4 if rotation else 3) / 2
            b = np.array([c - half, c + half]).astype(np.int32)
            b[:, 0], b[:, 1] = np.clip(b[:, 0], 0, img.shape[1]), np.clip(b[:, 1], 0, img.shape[0])
            im = img[b[0][1]:b[1][1], b[0][0]:b[1][0], :]
            side = int(np.linalg.norm(im.shape[:2])) if rotation else max(im.shape[:2])
            ph, pw = side - im.shape[0], side - im.shape[1]
            left, top = pw // 2, ph // 2
            im = cv2.copyMakeBorder(im, top, ph - top, left, pw - left, cv2.BORDER_CONSTANT, value=(0, 0, 0))
            return im, b, b[0] - [left, top]
        img, pbox, bias = crop(bgr, palm[0:4].reshape(2, 2), True)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pbox = pbox - bias
        lms = palm[4:18].reshape(7, 2) - bias
        p1, p2 = lms[0], lms[2]
        rad = math.pi / 2 - math.atan2(-(p2[1] - p1[1]), p2[0] - p1[0])
        rad -= 2 * math.pi * math.floor((rad + math.pi) / (2 * math.pi))
        angle = math.degrees(rad)
        centre = pbox.sum(axis=0) / 2
        rot = cv2.getRotationMatrix2D(tuple(centre), angle, 1.0)
        rimg = cv2.warpAffine(img, rot, (img.shape[1], img.shape[0]))
        hom = np.c_[lms, np.ones(len(lms))]
        rl = np.array([hom @ rot[0], hom @ rot[1]])
        rbox = np.array([rl.min(axis=1), rl.max(axis=1)])
        crop2, rbox, _ = crop(rimg, rbox, False)
        blob = cv2.resize(crop2, (224, 224), interpolation=cv2.INTER_AREA).astype(np.float32)[None] / 255.0
        lm, conf, _, _ = (np.asarray(o) for o in self.pose.run(None, {'input_1': blob}))
        if conf[0][0] < conf_min:
            return None
        lm = lm[0].reshape(-1, 3)
        scale = max((rbox[1] - rbox[0]) / 224.0)
        lm[:, :2] = (lm[:, :2] - 112.0) * scale
        lm[:, 2] *= scale
        cr = cv2.getRotationMatrix2D((0, 0), angle, 1.0)
        rl2 = lm[:, :2] @ cr[:, :2]
        inv = np.array([[rot[0][0], rot[1][0]], [rot[0][1], rot[1][1]]])
        tr = np.array([rot[0][2], rot[1][2]])
        inv_t = np.array([-inv[0] @ tr, -inv[1] @ tr])
        inv_rot = np.c_[inv, inv_t]
        c = np.append(rbox.sum(axis=0) / 2, 1)
        orig = np.array([c @ inv_rot[0], c @ inv_rot[1]])
        lm[:, :2] = rl2 + orig + bias
        return lm

    def read(self, bgr):
        """[(landmarks 21 x 3, extended [5 bools], palm score)] for the hands found in a BGR crop, strongest first; [] when none."""
        self._load()
        out = []
        for palm in self._palms(bgr)[:2]:
            lm = self._landmarks(bgr, palm)
            if lm is not None:
                out.append((lm, extended_fingers(lm), float(palm[-1])))
        return out


def wrist_crop(kps, shape, min_conf=0.4):
    """(x1, y1, x2, y2, (wx, wy)) of a square around the raised hand of a person, from body keypoints (COCO-17 [(x, y, conf)]), or None when no wrist is up.
    The hand lies beyond the wrist along the forearm: the square is centred 0.35 forearms past it and 1.4 forearms wide (at least 64 px), clipped to the image of
    `shape` (h, w, ...). wrist = the wrist's own position, for checking that a found hand is the one on this arm."""
    best = None
    for sh, el, wr in ((5, 7, 9), (6, 8, 10)):
        if max(sh, el, wr) < len(kps) and all(kps[i][2] >= min_conf for i in (sh, el, wr)) and kps[wr][1] < kps[sh][1]:
            if best is None or kps[wr][1] < best[2][1]:
                best = (kps[sh], kps[el], kps[wr])
    if best is None:
        return None
    _, e, w = best
    fx, fy = w[0] - e[0], w[1] - e[1]
    fore = max(24.0, math.hypot(fx, fy))
    cx, cy, half = w[0] + 0.35 * fx, w[1] + 0.35 * fy, max(64.0, 1.4 * fore) / 2
    h, wd = shape[:2]
    x1, y1, x2, y2 = int(max(0, cx - half)), int(max(0, cy - half)), int(min(wd, cx + half)), int(min(h, cy + half))
    return (x1, y1, x2, y2, (w[0], w[1])) if x2 - x1 >= 16 and y2 - y1 >= 16 else None


def five_fingers(model, bgr, kps, min_palm=0.6, near=0.6):
    """True when the hand on a person's raised arm shows five extended fingers, False when a hand is there with fewer, None when no hand was found or it is not
    on this arm. Only the hand whose wrist landmark lies within `near` crop widths of the body wrist counts: a palm detector also fires on faces, and a raised
    hand is often beside one."""
    box = wrist_crop(kps, bgr.shape)
    if box is None:
        return None
    x1, y1, x2, y2, (wx, wy) = box
    side = max(x2 - x1, y2 - y1)
    best = None
    for lm, ext, score in model.read(bgr[y1:y2, x1:x2]):
        d = math.hypot(lm[0][0] + x1 - wx, lm[0][1] + y1 - wy)
        if score >= min_palm and d <= near * side and (best is None or d < best[0]):
            best = (d, ext)
    return None if best is None else all(best[1])
