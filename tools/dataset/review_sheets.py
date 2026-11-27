#!/usr/bin/env python3
"""Contact sheets of the rough VLM boxes, to review by eye: python3 tools/dataset/review_sheets.py [--root datasets/arena/web] [--per 48]

With <root>/embed_scores.csv present (tools/dataset/embed_filter.py) only the photos it did not flag as junk are shown; --all shows everything.
Writes <root>/review/<class>_<n>.jpg: a grid of images with the box in red and the image number (row in boxes.csv) in the corner.
Note the numbers of the bad ones; nothing is deleted here.
"""
import argparse, csv, os
from PIL import Image, ImageDraw

ap = argparse.ArgumentParser()
ap.add_argument('--root', default='datasets/arena/web')
ap.add_argument('--per', type=int, default=48)
ap.add_argument('--all', action='store_true')
a = ap.parse_args()
rows = list(csv.DictReader(open(os.path.join(a.root, 'boxes.csv'))))
sc = os.path.join(a.root, 'embed_scores.csv')
skip = set() if a.all or not os.path.exists(sc) else {r['file'] for r in csv.DictReader(open(sc)) if r['verdict'] == 'junk'}
rev = os.path.join(a.root, 'review')
os.makedirs(rev, exist_ok=True)
for old in os.listdir(rev):
    os.remove(os.path.join(rev, old))
T, COLS = 220, 8
by = {}
for i, r in enumerate(rows):
    if r['file'] in skip:
        continue
    by.setdefault(r['class'], []).append((i, r))
for cls, items in by.items():
    for s in range(0, len(items), a.per):
        chunk = items[s:s + a.per]
        sheet = Image.new('RGB', (COLS * T, -(-len(chunk) // COLS) * T), 'white')
        for k, (i, r) in enumerate(chunk):
            im = Image.open(os.path.join(a.root, r['file'])).convert('RGB')
            d = ImageDraw.Draw(im)
            if r['x1'] != '':
                W, H = im.size
                d.rectangle([float(r['x1']) * W, float(r['y1']) * H, float(r['x2']) * W, float(r['y2']) * H], outline='red', width=max(3, W // 80))
            im.thumbnail((T - 4, T - 4))
            d = ImageDraw.Draw(im)
            d.rectangle([0, 0, 34, 14], fill='black')
            d.text((2, 1), str(i), fill='yellow')
            sheet.paste(im, ((k % COLS) * T, (k // COLS) * T))
        p = os.path.join(a.root, 'review', f'{cls.replace(" ", "_")}_{s // a.per + 1}.jpg')
        sheet.save(p, quality=80)
        print(p, len(chunk))
