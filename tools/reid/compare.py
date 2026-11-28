#!/usr/bin/env python3
"""Which appearance network tells YOUR people apart best? Compares backbones on crops saved by person_detection_node.

  1. ros2 run person_detection person_detection_node --ros-args -p reid_model:=models/reid/osnet_ain_x1_0.onnx -p dump_dir:=/tmp/reid_dump
     (walk about, turn, sit, come close, leave and re-enter; folders /tmp/reid_dump/<id>/ fill with crops, c0 = whole body, c1 = cut off)
  2. Look at the folders and decide which ids are one person (the node splits people it should not), then:
     python3 tools/reid/compare.py /tmp/reid_dump --same 4,5 --same 1,2 \
         --backbone osnet=models/reid/osnet_ain_x1_0.onnx --backbone clip=/path/clip_reid.onnx:clip

For every backbone: mean cosine between crops of the same person and of different people, and the AUC (the chance a same-person pair
scores above a different-person pair; 0.5 = useless, 1.0 = perfect). "cross" lists the same-person pairs of a whole-body and a cut-off
crop, the case that split your people. A backbone is `name=path.onnx` (input NCHW, size read from the model; `:clip` = CLIP mean/std,
else ImageNet); export other models to ONNX first (see export_osnet.py).
Rows named `colour` and `<name>+colour` add the colour fingerprint the node uses (parts.py): here the keypoints are not in the saved crops,
so it is taken from the middle of a cut-off crop and from the upper body (15-55% of the height) of a whole-body one, an approximation of
the node's torso box. `+colour` is 0.6 x network + 0.4 x colour, the node's score.
Ids named together in one --same are one person. Needs at least two people.
"""
import argparse
import glob
import os

import cv2
import numpy as np
from person_detection.parts import histogram

IMAGENET = ((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
CLIP = ((0.4815, 0.4578, 0.4082), (0.2686, 0.2613, 0.2758))


def load_onnx(spec):
    import onnxruntime as ort
    path, _, norm = spec.partition(':')
    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0]
    h, w = (inp.shape[2], inp.shape[3]) if all(isinstance(v, int) for v in inp.shape[2:]) else (256, 128)
    mean, std = (CLIP if norm == 'clip' else IMAGENET)

    def embed(crops):
        x = np.stack([(cv2.cvtColor(cv2.resize(c, (w, h)), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0 - mean) / std for c in crops])
        return sess.run(None, {inp.name: x.transpose(0, 3, 1, 2).astype(np.float32)})[0].reshape(len(crops), -1)
    return embed


def auc(pos, neg):
    """Chance that a random same-person pair scores above a random different-person pair (ties count half)."""
    if not len(pos) or not len(neg):
        return float('nan')
    p, n = np.asarray(pos)[:, None], np.asarray(neg)[None, :]
    return float(((p > n).sum() + 0.5 * (p == n).sum()) / (p.size * n.size))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dump')
    ap.add_argument('--same', action='append', default=[], help='comma separated ids that are one person (repeat for more people)')
    ap.add_argument('--backbone', action='append', required=True, help='name=path.onnx[:clip]')
    ap.add_argument('--max-per-id', type=int, default=60)
    a = ap.parse_args()
    person = {}
    for g in a.same:
        for i in g.split(','):
            person[i] = g
    files, who, cut = [], [], []
    for d in sorted(glob.glob(os.path.join(a.dump, '*'))):
        fs = sorted(glob.glob(os.path.join(d, '*.jpg')))
        fs = fs[::max(1, len(fs) // a.max_per_id)]
        files += fs
        who += [person.get(os.path.basename(d), os.path.basename(d))] * len(fs)
        cut += ['_c1' in os.path.basename(f) for f in fs]
    if len(set(who)) < 2:
        raise SystemExit('need crops of at least two different people')
    crops = [cv2.imread(f) for f in files]
    same = np.array([[x == y for y in who] for x in who])
    xk = np.array([[x != y for y in cut] for x in cut])
    iu = np.triu(np.ones_like(same), 1).astype(bool)
    print(f'{len(files)} crops, {len(set(who))} people ({sum(cut)} cut-off)')
    hs = np.sqrt(np.stack([histogram(c if k else c[int(0.15 * len(c)):int(0.55 * len(c))]) for c, k in zip(crops, cut)]))
    colour = hs @ hs.T                                        # Bhattacharyya coefficient of every pair

    def row(name, s):
        pos, neg, cross = s[same & iu], s[~same & iu], s[same & xk & iu]
        print(f'{name:14s} same {pos.mean():.2f}  different {neg.mean():.2f}  AUC {auc(pos, neg):.3f}   '
              f'cross-kind same {cross.mean() if len(cross) else float("nan"):.2f} ({len(cross)} pairs)')
    row('colour', colour)
    for spec in a.backbone:
        name, _, src = spec.partition('=')
        e = load_onnx(src)(crops)
        e = e / (np.linalg.norm(e, axis=1, keepdims=True) + 1e-9)
        s = e @ e.T
        row(name, s)
        row(name + '+colour', 0.6 * s + 0.4 * colour)


if __name__ == '__main__':
    main()
