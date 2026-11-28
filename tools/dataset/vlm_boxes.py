#!/usr/bin/env python3
"""Rough boxes for the downloaded product photos from a local llama-server running a vision model (default: Cosmos-Reason2-2B GGUF).

    llama-server -m models/vlm/Cosmos-Reason2-2B-Q4_K_M.gguf --mmproj models/vlm/mmproj-Cosmos-Reason2-2B-F16.gguf -ngl 99 -c 8192 -np 1 --port 8091 --reasoning off
    python3 tools/dataset/vlm_boxes.py [--root datasets/arena/web] [--port 8091]

Writes <root>/boxes.csv (class,file,x1,y1,x2,y2 as 0-1 fractions, raw reply). Resumable. The boxes are ROUGH (about half usable on a 24-image look):
review them before training. Coordinates are 0-1000 relative (Qwen3-VL style); Qwen2.5-VL answers in pixels instead.
"""
import argparse, base64, csv, glob, json, os, re, urllib.request

ap = argparse.ArgumentParser()
ap.add_argument('--root', default='datasets/arena/web')
ap.add_argument('--port', type=int, default=8091)
a = ap.parse_args()
out = os.path.join(a.root, 'boxes.csv')
done = set()
if os.path.exists(out):
    done = {r['file'] for r in csv.DictReader(open(out))}
new = not os.path.exists(out)
f = open(out, 'a', newline='')
w = csv.writer(f)
if new:
    w.writerow(['class', 'file', 'x1', 'y1', 'x2', 'y2', 'reply'])
n = 0
for cls in sorted(d for d in os.listdir(a.root) if os.path.isdir(os.path.join(a.root, d))):
    for p in sorted(glob.glob(f'{a.root}/{cls}/*.jpg')):
        rel = os.path.relpath(p, a.root)
        if rel in done:
            continue
        q = (f'Locate the {cls} product package in the image. Output only JSON: [{{"bbox_2d":[x1,y1,x2,y2],"label":"{cls}"}}]. '
             'Coordinates are 0-1000 relative. If absent output [].')
        body = {'messages': [{'role': 'user', 'content': [
            {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + base64.b64encode(open(p, 'rb').read()).decode()}},
            {'type': 'text', 'text': q}]}], 'max_tokens': 200, 'temperature': 0}
        try:
            r = urllib.request.urlopen(urllib.request.Request(f'http://localhost:{a.port}/v1/chat/completions', json.dumps(body).encode(),
                                                               {'Content-Type': 'application/json'}), timeout=120)
            t = json.load(r)['choices'][0]['message']['content']
        except Exception as e:  # noqa: BLE001  one bad image must not stop a 1500-image run
            t = f'ERROR {type(e).__name__}'
        m = re.search(r'\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\]', t)
        box = [int(v) / 1000 for v in m.groups()] if m else ['', '', '', '']
        w.writerow([cls, rel, *box, t.replace('\n', ' ')[:200]])
        f.flush()
        n += 1
        if n % 100 == 0:
            print(n, 'done', flush=True)
print('wrote', out, n, 'new rows')
