#!/usr/bin/env python3
"""Fetch photos of packaged food / snacks from Open Food Facts (official APIs, user-uploaded photos, CC-BY-SA) for the arena detector.

    python3 tools/dataset/fetch_off.py --classes pocky pringles --target 400      # ~400 images per class (fewer if the site has fewer)
    python3 tools/dataset/fetch_off.py --target 12 --classes pocky --dry-run       # count candidates only, download nothing

Writes datasets/arena/web/<class>/<barcode>_<imgid>.jpg (400 px side) and datasets/arena/web/manifest.csv (class, barcode, imgid, url, product,
brand, uploader, licence). Keep them apart from your own arm-camera frames (datasets/arena/images/): they are product photos, not your camera.
Rate limits are kept well under the site's (search: 1 call / 7 s, product: ~1 call / s). Resumable: files already present are skipped.
These are IMAGES ONLY, no boxes: label them before training (docs/perception.md). Attribution: Open Food Facts contributors, CC-BY-SA 3.0.
"""
import argparse
import csv
import hashlib
import json
import os
import re
import time
import urllib.parse
import urllib.request

UA = 'arena-dataset-fetch/1.0 (RoboCup research; contact via GitHub)'
OUT = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'datasets', 'arena', 'web')
# class (names as in src/object_detection/config/arena.yaml) -> (search queries, regex a product's name/brand/categories must match)
CLASSES = {
    'cereal': (['breakfast cereal', 'corn flakes', 'chocolate cereal', 'oat cereal', 'cereal box', 'muesli'], r'cereal|flakes|muesli|granola|cornflakes'),
    'tom-yam cup noodle': (['tom yum cup noodle', 'tom yam instant noodle cup', 'mama tom yum', 'tom yum noodle'], r'tom\s*y[au]m|tomyum'),
    'yen-ta-four cup noodle': (['yentafo cup noodle', 'yen ta four noodle', 'yentafo instant noodle', 'mama yentafo'],
                               r'yen\s*-?\s*ta\s*-?\s*fo|yentafo|yen\s*ta\s*four'),
    'biscuit roll': (['biscuit roll', 'wafer roll', 'egg roll biscuit', 'cream roll', 'rolled wafer'], r'roll'),
    'paprika': (['paprika chips', "lay's paprika", 'paprika crisps', 'paprika snack'], r'paprika'),
    'potae': (['potae', 'potae potato snack', 'potae chips'], r'potae'),
    'cornae': (['cornae', 'cornae corn snack'], r'cornae'),
    'pringles': (['pringles', 'pringles original', 'pringles sour cream'], r'pringles'),
    'lay stax': (["lay's stax", 'stax potato crisps', 'lays stax'], r'stax'),
    'pocky': (['pocky', 'glico pocky', 'pocky chocolate', 'pocky strawberry'], r'pocky'),
}


def get(url, tries=3):
    for i in range(tries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': UA}), timeout=30) as r:
                return r.read()
        except Exception:  # noqa: BLE001 - a flaky request is retried, then skipped
            time.sleep(2 * (i + 1))
    return None


def product_path(code):
    return code if len(code) <= 8 else '/'.join([code[:3], code[3:6], code[6:9], code[9:]])


def find_products(cls, queries, rx, want):
    """[(code, name, brand)] whose text matches rx, from the search service, until `want` products are found."""
    seen, out = set(), []
    for q in queries:
        for page in range(1, 11):
            url = 'https://search.openfoodfacts.org/search?' + urllib.parse.urlencode(
                {'q': q, 'page_size': 100, 'page': page, 'fields': 'code,product_name,brands,categories_tags'})
            time.sleep(7)
            raw = get(url)
            hits = json.loads(raw).get('hits', []) if raw else []
            for h in hits:
                text = ' '.join([str(h.get('product_name', '')), ' '.join(h.get('brands') or []), ' '.join(h.get('categories_tags') or [])]).lower()
                if h['code'] not in seen and re.search(rx, text):
                    seen.add(h['code'])
                    out.append((h['code'], h.get('product_name', ''), ', '.join(h.get('brands') or [])))
            if len(hits) < 100 or len(out) >= want:
                break
        if len(out) >= want:
            break
    return out


def photos(code, per_product):
    """[(imgid, uploader)] user photos of one product: the front shot first, never the ingredients / nutrition / packaging panels."""
    time.sleep(0.8)
    raw = get(f'https://world.openfoodfacts.org/api/v2/product/{code}?fields=images')
    imgs = (json.loads(raw).get('product', {}).get('images') or {}) if raw else {}
    skip = {str(v.get('imgid')) for k, v in imgs.items() if re.match(r'(nutrition|ingredients|packaging)_', k)}
    front = [str(v.get('imgid')) for k, v in imgs.items() if k.startswith('front_')]
    ids = front + [k for k in imgs if k.isdigit() and k not in front]
    out = []
    for i in ids:
        v = imgs.get(i) or {}
        s = (v.get('sizes') or {}).get('400') or {}
        if i in skip or not s or not 0.35 <= s['w'] / s['h'] <= 2.5:       # a very wide image is a table, not a package
            continue
        out.append((i, v.get('uploader', '')))
        if len(out) >= per_product:
            break
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--classes', nargs='*', default=list(CLASSES))
    ap.add_argument('--target', type=int, default=400, help='images per class')
    ap.add_argument('--per-product', type=int, default=3)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    man = os.path.join(OUT, 'manifest.csv')
    new = not os.path.exists(man)
    with open(man, 'a', newline='') as mf:
        w = csv.writer(mf)
        if new:
            w.writerow(['class', 'barcode', 'imgid', 'url', 'product', 'brand', 'uploader', 'licence'])
        for cls in a.classes:
            queries, rx = CLASSES[cls]
            d = os.path.join(OUT, cls)
            os.makedirs(d, exist_ok=True)
            have = len([f for f in os.listdir(d) if f.endswith('.jpg')])
            products = find_products(cls, queries, rx, a.target // max(1, a.per_product) * 2)
            print(f'[{cls}] {len(products)} matching products, {have} images already on disk, target {a.target}', flush=True)
            hashes, n = set(), have
            for code, name, brand in products:
                if n >= a.target:
                    break
                for imgid, up in photos(code, a.per_product):
                    f = os.path.join(d, f'{code}_{imgid}.jpg')
                    url = f'https://images.openfoodfacts.org/images/products/{product_path(code)}/{imgid}.400.jpg'
                    if os.path.exists(f) or a.dry_run:
                        n += 0 if os.path.exists(f) else (1 if a.dry_run else 0)
                        continue
                    data = get(url)
                    if not data or hashlib.sha1(data).hexdigest() in hashes:
                        continue
                    hashes.add(hashlib.sha1(data).hexdigest())
                    open(f, 'wb').write(data)
                    w.writerow([cls, code, imgid, url, name, brand, up, 'CC-BY-SA 3.0 (Open Food Facts contributors)'])
                    mf.flush()
                    n += 1
                if n % 25 == 0:
                    print(f'[{cls}] {n}/{a.target}', flush=True)
            print(f'[{cls}] done: {n} images', flush=True)


if __name__ == '__main__':
    main()
