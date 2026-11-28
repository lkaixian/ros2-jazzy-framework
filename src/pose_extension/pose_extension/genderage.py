"""Gender and age of a face crop with the InsightFace `genderage` ONNX (96 x 96 input, output [female, male, age / 100]). ROS-free.

The weights are InsightFace's buffalo_l/genderage.onnx (public, MIT-licensed code but the models are for NON-COMMERCIAL research use): put the file at
models/genderage/genderage.onnx (see docs/perception.md). Accuracy on this robot's camera is unmeasured; the node votes over several frames and says nothing when
the frames disagree.
shortcut: the face is cropped as a square 1.5x its box instead of InsightFace's landmark alignment.
"""
import os

import cv2
import numpy as np


def find_model(path=''):
    if path:
        return path if os.path.isfile(path) else ''
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        p = os.path.join(d, 'models', 'genderage', 'genderage.onnx')
        if os.path.isfile(p):
            return p
        d = os.path.dirname(d)
    return ''


def face_square(box, iw, ih, scale=1.5):
    """(x1, y1, x2, y2) pixels of a square `scale` x the face box, clipped to the image."""
    cx, cy, s = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2, scale * max(box[2] - box[0], box[3] - box[1]) / 2
    return int(max(0, cx - s)), int(max(0, cy - s)), int(min(iw, cx + s)), int(min(ih, cy + s))


def decode(out):
    """Network output [female, male, age/100] -> ('woman' | 'man', margin 0..1, age)."""
    f, m, a = (float(v) for v in np.asarray(out).ravel()[:3])
    return ('man' if m > f else 'woman'), abs(m - f) / (abs(m) + abs(f) + 1e-9), int(round(a * 100))


def vote(readings, min_share=0.7, min_margin=0.2):
    """[(gender, margin, age)] over several frames -> ('man' | 'woman' | '', median age): the gender only when at least `min_share` of the frames
    with a clear margin agree; '' when they disagree or none was clear."""
    clear = [g for g, m, _ in readings if m >= min_margin]
    ages = sorted(a for _, _, a in readings)
    best = max(('man', 'woman'), key=clear.count)
    gender = best if clear and clear.count(best) / len(clear) >= min_share else ''
    return gender, (ages[len(ages) // 2] if ages else 0)


class GenderAge:
    def __init__(self, path=''):
        self.path, self.sess = path, None

    def read(self, bgr_face):
        """One crop -> (gender, margin, age). Raises RuntimeError with a readable reason when the model is missing."""
        if self.sess is None:
            p = find_model(self.path)
            if not p:
                raise RuntimeError('no gender/age model: put genderage.onnx in models/genderage/ (docs/perception.md)')
            import onnxruntime as ort
            self.sess = ort.InferenceSession(p, providers=['CPUExecutionProvider'])
        blob = cv2.dnn.blobFromImage(cv2.resize(bgr_face, (96, 96)), 1.0, (96, 96), (0, 0, 0), swapRB=True)
        return decode(self.sess.run(None, {self.sess.get_inputs()[0].name: blob})[0])
