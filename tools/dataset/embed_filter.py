#!/usr/bin/env python3
"""Score every downloaded photo with Qwen3-VL-Embedding-2B so the junk shows up before you review boxes.

    models/vlm/venv/bin/python tools/dataset/embed_filter.py [--root datasets/arena/web] [--model models/vlm/qwen3-vl-embedding-2b]

Writes <root>/embed_scores.csv: class,file,front,outlier,verdict.
  front   = similarity to "front of a <class> package" minus similarity to "barcode / nutrition table / ingredient text" (higher = a real front photo)
  outlier = 1 - cosine to the class's median-direction vector (higher = does not look like the rest of its class; catches mislabelled products)
  verdict = junk (front < FRONT_MIN) | odd (outlier in the worst 8% of its class) | ok     -- thresholds are a first guess: look at both ends before trusting them
Needs the venv (qwen-vl-utils) and a free GPU (~5 GB): stop llama-server first.
"""
import argparse, csv, glob, os, sys
import numpy as np, torch

FRONT_MIN = 0.15   # checked by eye on 43 Pocky photos: real fronts scored >= 0.198, barcodes/nutrition/wrong-product <= 0.099
ap = argparse.ArgumentParser()
ap.add_argument('--root', default='datasets/arena/web')
ap.add_argument('--model', default='models/vlm/qwen3-vl-embedding-2b')
ap.add_argument('--batch', type=int, default=8)
a = ap.parse_args()
sys.path.insert(0, os.path.join(a.model, 'scripts'))
from qwen3_vl_embedding import Qwen3VLEmbedder  # noqa: E402

m = Qwen3VLEmbedder(model_name_or_path=a.model, torch_dtype=torch.float16, max_pixels=448 * 448)
classes = sorted(d for d in os.listdir(a.root) if os.path.isdir(os.path.join(a.root, d)) and d != 'review')
files = [(c, os.path.relpath(p, a.root)) for c in classes for p in sorted(glob.glob(f'{a.root}/{c}/*.jpg'))]
E = []
for i in range(0, len(files), a.batch):
    E.append(m.process([{'image': os.path.join(a.root, f)} for _, f in files[i:i + a.batch]]).float().cpu().numpy())
    if (i // a.batch) % 20 == 0:
        print(i, '/', len(files), flush=True)
E = np.concatenate(E)
BAD = ['a barcode', 'a nutrition facts table', 'a block of printed ingredient text', 'the back of a food package with small print']
bad = m.process([{'text': t} for t in BAD]).float().cpu().numpy()
rows = []
for c in classes:
    idx = [i for i, (cc, _) in enumerate(files) if cc == c]
    if not idx:
        continue      # a class folder with no photos yet (yen-ta-four)
    good = m.process([{'text': f'a photo of the front of a {c} package, product shot'}]).float().cpu().numpy()[0]
    cen = np.median(E[idx], axis=0)
    cen /= np.linalg.norm(cen)
    out = {i: 1 - float(E[i] @ cen) for i in idx}
    cut = np.percentile(list(out.values()), 92)
    for i in idx:
        front = float(E[i] @ good) - float((E[i] @ bad.T).max())
        rows.append((c, files[i][1], round(front, 4), round(out[i], 4), 'junk' if front < FRONT_MIN else 'odd' if out[i] >= cut else 'ok'))
with open(os.path.join(a.root, 'embed_scores.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['class', 'file', 'front', 'outlier', 'verdict'])
    w.writerows(rows)
print({v: sum(r[4] == v for r in rows) for v in ('ok', 'odd', 'junk')})
