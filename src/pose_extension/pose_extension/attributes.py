"""Visible attributes of a guest's face from a face-parsing mask (CelebAMask-HQ, 19 classes). Pure numpy/cv2; the model loads on first use.

Only what the mask shows reliably: glasses, hair colour, hat, earrings / necklace. Hair length, style and jackets are NOT attempted.
Wrong information costs marks (RoboCup Receptionist: -40 each), so every attribute is conditional: below its threshold it is left
out ('' / unknown), never guessed.
shortcut: thresholds and hair-colour rules come from a few photos, tune them with real guests under the venue light.
"""
import glob
import os

import cv2
import numpy as np

SKIN, EYEGLASSES, L_EYE, R_EYE, HAIR, HAT, EARRING, NECKLACE = 1, 3, 4, 5, 13, 14, 15, 16
MIN_FACE_PX = 60        # a face narrower than this is too far to judge


def find_model(path=''):
    """The face-parsing checkpoint: `path`, else the large (more accurate) one, else the nano one, in the nearest models/face_parsing."""
    if path:
        return path if os.path.isfile(path) else ''
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        for pat in ('yolo26l-sem*.pt', 'yolo26n-sem*.pt'):
            hit = sorted(glob.glob(os.path.join(d, 'models', 'face_parsing', pat)))
            if hit:
                return hit[0]
        d = os.path.dirname(d)
    return ''


def head_crop(box, iw, ih):
    """(x1, y1, x2, y2) pixels of the head and its surroundings (hair, ears) around a face box, clipped to the image."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    return (int(max(0, x1 - 0.5 * w)), int(max(0, y1 - 0.7 * h)), int(min(iw, x2 + 0.5 * w)), int(min(ih, y2 + 0.3 * h)))


def hair_colour(h, s, v):
    """OpenCV HSV (h 0-179, s/v 0-255) medians of the hair pixels -> black / brown / blonde / red / grey / white; '' when unsure (dyed)."""
    if v < 70:
        return 'black'
    if s < 40:
        return 'white' if v >= 200 else 'grey'
    if (h <= 10 or h >= 172) and s >= 180 and v >= 120:
        return 'red'
    if 5 <= h <= 35:
        return 'blonde' if v >= 170 and s <= 170 else 'brown'
    return ''


def attributes(mask, bgr, face_area):
    """{'glasses': 'yes'|'no'|'', 'hair': colour|'', 'hat': 'yes'|'', 'accessories': [...]} from a class mask (HxW uint8) and its BGR crop.
    face_area = the face box area in the crop's pixels: all thresholds scale with it."""
    n = np.bincount(mask.ravel(), minlength=19)
    out = {'glasses': '', 'hair': '', 'hat': '', 'accessories': []}
    if n[EYEGLASSES] >= 0.04 * face_area:
        out['glasses'] = 'yes'
    elif n[L_EYE] + n[R_EYE] >= 0.01 * face_area and n[EYEGLASSES] < 0.004 * face_area:
        out['glasses'] = 'no'             # the eyes are clearly visible and nothing covers them
    if n[HAT] >= 0.15 * face_area:
        out['hat'] = 'yes'
    if n[HAIR] >= 0.15 * face_area:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[mask == HAIR]
        out['hair'] = hair_colour(*np.median(hsv, axis=0))
    if n[EARRING] >= max(12, 0.003 * face_area):
        out['accessories'].append('earrings')
    if n[NECKLACE] >= 0.01 * face_area:
        out['accessories'].append('a necklace')
    return out


def sentence(a):
    """'wearing glasses, with brown hair and earrings' from attributes(); '' when nothing is certain. Only positives are spoken."""
    parts = []
    if a['glasses'] == 'yes':
        parts.append('wearing glasses')
    if a['hat'] == 'yes':
        parts.append('wearing a hat')
    if a['hair']:
        parts.append(f"with {a['hair']} hair")
    if a['accessories']:
        parts.append('wearing ' + ' and '.join(a['accessories']))
    return ', '.join(parts[:-1]) + (' and ' if len(parts) > 1 else '') + parts[-1] if parts else ''


class FaceParser:
    """Lazy face-parsing model: loads on the first parse, so a node that never describes a guest never pays for it."""

    def __init__(self, path=''):
        self.path, self.model = path, None

    def parse(self, bgr):
        """Class mask (HxW uint8) of a BGR crop. Raises RuntimeError with a readable reason when the model cannot be loaded."""
        if self.model is None:
            path = find_model(self.path)
            if not path:
                raise RuntimeError('no face-parsing model: put yolo26l-sem-celebamaskhq.pt in models/face_parsing/ (docs/perception.md)')
            try:
                from ultralytics import YOLO
                self.model = YOLO(path)
            except Exception as e:      # noqa: BLE001 - any loader failure becomes one readable message
                raise RuntimeError(f'cannot load face-parsing model {path}: {e}') from None
        return self.model(bgr, verbose=False)[0].semantic_mask.data.cpu().numpy().astype(np.uint8)
