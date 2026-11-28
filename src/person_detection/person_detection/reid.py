"""Person re-identification embeddings: an OSNet-AIN ONNX model (tools/reid/export_osnet.py) on person crops."""
import cv2
import numpy as np

from person_detection.parts import colour_similarity

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def preprocess(crops):
    """BGR crops (any size) -> N x 3 x 256 x 128 float32, RGB, ImageNet-normalised."""
    out = [(cv2.cvtColor(cv2.resize(c, (128, 256)), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0 - MEAN) / STD for c in crops]
    return np.stack(out).transpose(0, 3, 1, 2).astype(np.float32)


def usable(box, min_h=64, min_w=32):
    """False for a crop too small to embed. Not for cut-off or torso-only crops: a person sitting close fills the frame like that."""
    x1, y1, x2, y2 = box
    return y2 - y1 >= min_h and x2 - x1 >= min_w


class Reid:
    def __init__(self, path):
        import onnxruntime as ort
        self.session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])   # ~5 ms per crop; keeps the GPU for YOLO

    def embed(self, crops):
        """-> N x 512 unit vectors (empty list for no crops)."""
        if not crops:
            return []
        f = self.session.run(None, {'image': preprocess(crops)})[0]
        return f / (np.linalg.norm(f, axis=1, keepdims=True) + 1e-9)


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


UPPER = 0.55    # fraction of a whole-body box, from the top, that is the "torso view" a close-up camera sees


def is_cut(box, size, edge=4):
    """1 when the box touches the image edge (part of the person is out of the picture), else 0."""
    return int(size is not None and (box[0] <= edge or box[1] <= edge or box[2] >= size[0] - edge or box[3] >= size[1] - edge))


class IdentityBook:
    """Stable person ids that do not depend on the tracker (it gives no id while the camera moves).

    Per detection, in order: (1) the tracker id already bound to an identity; (2) the box of an identity seen a moment ago
    (IoU >= overlap, and not a visibly different person); (3) the unseen identity that looks most like it (cosine >= sim, best of its
    stored looks); (4) otherwise a candidate, which becomes a new identity only after `confirm` frames of the same thing (same place or
    same looks), so a flicker or one odd frame never makes a person. One detection <-> one identity per frame.
    shortcut: appearance and box overlap only, no motion model; two people crossing can swap. Upgrade to a Kalman/Hungarian tracker
    if swaps show up on real footage."""

    def __init__(self, sim=0.70, keep_sec=900.0, confirm=5, looks=40, same=0.92, overlap=0.3, gap=1.0, differ=0.4, margin=0.06,
                 add_min=0.7, window=6, verify=0.65, occlude=0.25, edge=4, colour_weight=0.4, near=0.06):
        self.sim, self.keep, self.confirm, self.looks, self.same = sim, keep_sec, confirm, looks, same
        self.overlap, self.gap, self.differ, self.margin, self.add_min = overlap, gap, differ, margin, add_min
        self.window, self.verify, self.occlude, self.edge, self.cw, self.near = window, verify, occlude, edge, colour_weight, near
        self.col = {}      # identity id -> {part: running mean colour histogram}
        self.together = set()   # (a, b) pairs of identities that were in one frame at once: two people, never merged
        self.merged_t = 0.0
        self.win = {}      # identity id -> its last few similarity scores against whoever sat in its place
        self.gal = {}      # identity id -> [list of unit vectors, last seen]
        self.map = {}      # tracker id -> identity id
        self.last = {}     # identity id -> (box, time) of its latest detection
        self.pend = []     # candidates: [box, vector, frames, time]
        self.next = 1
        self.rng = np.random.default_rng(0)
        self.events = []   # human-readable decisions since the last read, for the log

    def _covered(self, box, others):
        """Is more than `occlude` of this box's area inside other people's boxes?"""
        a = (box[2] - box[0]) * (box[3] - box[1]) + 1e-9
        inter = sum(max(0.0, min(box[2], o[2][2]) - max(box[0], o[2][0])) * max(0.0, min(box[3], o[2][3]) - max(box[1], o[2][1])) for o in others)
        return inter / a > self.occlude

    def _colour(self, sid, ps):
        """Mean colour similarity over the parts (torso, legs) seen both now and before; None when they share none."""
        c = [colour_similarity(self.col[sid][n], h) for n, h in (ps or {}).items() if n in self.col.get(sid, {})]
        return sum(c) / len(c) if c else None

    def _learn(self, sid, ps):
        """Fold this frame's part colours into the person's, unless one is far from what is known (a blocker, a bad crop)."""
        mine = self.col.setdefault(sid, {})
        for n, h in (ps or {}).items():
            if n not in mine:
                mine[n] = h
            elif colour_similarity(mine[n], h) >= 0.5:
                mine[n] = 0.95 * mine[n] + 0.05 * h

    def _score(self, sid, e, k=0, ps=None):
        """Best cosine to the stored looks of the same kind k (0 = whole body, 1 = cut off by the image edge: a torso close to the
        camera does not look like a whole body to the network, so the two are never compared); None when there is none of that kind.
        With part colours ps the result is (1 - colour_weight) x embedding + colour_weight x colour similarity."""
        g = self.gal[sid]
        emb = max((float(v @ e) for v, kk in zip(g[0], g[4]) if kk == k), default=None)
        c = self._colour(sid, ps)
        return emb if emb is None or c is None else (1.0 - self.cw) * emb + self.cw * c

    def _same(self, sid, e, t, k=0, ps=None):
        """Does whoever is in this identity's place still look like it? The mean of the last `window` scores, so a blocker for a
        few frames passes and a different person sitting down in the same seat (same tracker id, same box) does not."""
        sc = None if e is None else self._score(sid, e, k, ps)
        if sc is None:
            return True
        w = self.win.setdefault(sid, [])
        w.append(sc)
        del w[:-self.window]
        ok = len(w) < self.window or sum(w) / len(w) >= self.verify
        if not ok:
            self.map.pop(t, None)
            self.events.append(f'tracker {t}: not person {sid} any more (recent similarity {sum(w) / len(w):.2f})')
        return ok

    def _see(self, sid, e, box, now, thumb=None, k=0):
        g = self.gal.setdefault(sid, [[], now, 0, [], []])      # looks, last seen, how many distinct looks were ever offered, a small picture per look (for the memory board), the kind of each look
        g[1] = now
        self.last[sid] = (box, now)
        if e is None:
            return
        best = max((j for j in range(len(g[0])) if g[4][j] == k), key=lambda j: float(g[0][j] @ e), default=None)
        if best is not None and float(g[0][best] @ e) < self.add_min:
            return                    # looks like nobody we know (a blocker in front, a bad crop): not stored as this person
        if best is not None and float(g[0][best] @ e) >= self.same:
            v = 0.9 * g[0][best] + 0.1 * e
            g[0][best] = v / (np.linalg.norm(v) + 1e-9)
        else:
            self._store(g, e, thumb, k)

    def _store(self, g, e, thumb, k):
        g[2] += 1
        if len(g[0]) < self.looks:
            g[0].append(e)
            g[3].append(thumb)
            g[4].append(k)
        else:                      # reservoir: every distinct look ever seen is equally likely to stay, so old views (back, side) are not pushed out
            j = int(self.rng.integers(0, g[2]))
            if j < self.looks:
                g[0][j], g[3][j], g[4][j] = e, thumb, k

    def _merge_pass(self):
        """Two identities that look like one person and were never in view together are one person that was split (the toss-up rule
        then blocks either from taking the person back): fold the younger into the older."""
        for a in sorted(self.gal):
            for b in sorted(self.gal):
                if b <= a or a not in self.gal or b not in self.gal or (a, b) in self.together:
                    continue
                sc = max((x for look, k in zip(self.gal[b][0], self.gal[b][4]) if (x := self._score(a, look, k, self.col.get(b))) is not None),
                         default=None)
                if sc is not None and sc >= self.sim:
                    self._merge(a, b, sc)

    def _merge(self, keep, drop, sc):
        gk, gd = self.gal[keep], self.gal.pop(drop)
        for e, th, k in zip(gd[0], gd[3], gd[4]):
            self._store(gk, e, th, k)
        gk[1] = max(gk[1], gd[1])
        for n, h in self.col.pop(drop, {}).items():
            mine = self.col.setdefault(keep, {})
            mine[n] = h if n not in mine else (mine[n] + h) / 2
        self.map = {t: (keep if i == drop else i) for t, i in self.map.items()}
        if drop in self.last and (keep not in self.last or self.last[drop][1] > self.last[keep][1]):
            self.last[keep] = self.last[drop]
        self.last.pop(drop, None)
        self.win.pop(drop, None)
        self.together = {tuple(sorted(keep if i == drop else i for i in p)) for p in self.together}
        self.events.append(f'person {drop} was person {keep} (similarity {sc:.2f}, never in view together): merged')

    def assign(self, dets, now, thumbs=None, size=None, uppers=None, parts=None):
        """dets: [(tracker id or -1, unit vector or None, (x1, y1, x2, y2))], size = (width, height) of the image -> identity ids
        (None = not confirmed yet). A crop that another person's box covers by `occlude` of its area is dropped to "no appearance":
        it still follows its tracker id and box, but is never compared and never stored, so a neighbour cannot pollute a person.
        uppers: per detection (vector of the top UPPER of its box, thumbnail) or None. A whole-body look also stores that vector as a
        look of kind 1, so a person first seen from afar can be found again when they stand close and only the torso is in view.
        parts: per detection {'torso': histogram, 'legs': histogram} of its visible body parts (parts.py), or None."""
        hid = [e is not None and self._covered(box, dets[:i] + dets[i + 1:]) for i, (t, e, box) in enumerate(dets)]
        dets = [(t, None if h else e, box) for h, (t, e, box) in zip(hid, dets)]
        ups = [None if h or not uppers else uppers[i] for i, h in enumerate(hid)]
        ps = [None if h or not parts else parts[i] for i, h in enumerate(hid)]
        ks = [is_cut(b, size, self.edge) for _, _, b in dets]
        for sid in [k for k, g in self.gal.items() if now - g[1] > self.keep]:
            del self.gal[sid]
            self.col.pop(sid, None)
            self.win.pop(sid, None)
            self.last.pop(sid, None)
            self.map = {t: i for t, i in self.map.items() if i != sid}
        out, taken = [None] * len(dets), set()

        def take(i, sid, how=None):
            out[i] = sid
            taken.add(sid)
            t, e, box = dets[i]
            self._see(sid, e, box, now, thumbs[i] if thumbs else None, ks[i])
            if e is not None:
                self._learn(sid, ps[i])
            if ups[i] is not None and ks[i] == 0:
                self._see(sid, ups[i][0], box, now, ups[i][1], 1)
            if t >= 0:
                self.map[t] = sid
            if how:
                self.events.append(f'tracker {t}: person {sid} {how}')

        for i, (t, e, box) in enumerate(dets):                                       # 1. tracker id
            sid = self.map.get(t)
            if sid is not None and sid in self.gal and sid not in taken and self._same(sid, e, t, ks[i], ps[i]):
                take(i, sid)
        pairs = sorted(((iou(box, self.last[s][0]), i, s) for i, (t, e, box) in enumerate(dets) if out[i] is None
                        for s in self.last if s not in taken and now - self.last[s][1] <= self.gap), reverse=True)
        for ov, i, s in pairs:                                                       # 2. same place a moment ago
            e = dets[i][1]
            if ov < self.overlap or out[i] is not None or s in taken:
                continue
            sc = None if e is None else self._score(s, e, ks[i], ps[i])
            if (sc is None or sc >= self.differ) and self._same(s, e, dets[i][0], ks[i], ps[i]):
                take(i, s)
        pairs = []
        for i, (t, e, box) in enumerate(dets):
            if out[i] is None and e is not None:
                sc = sorted(((x, s) for s in self.gal if (x := self._score(s, e, ks[i], ps[i])) is not None), reverse=True)
                if sc and sc[0][0] >= self.sim and sc[0][1] not in taken and (len(sc) == 1 or sc[0][0] - sc[1][0] >= self.margin):
                    pairs.append((sc[0][0], i, sc[0][1]))                            # only a clear winner, never a toss-up between two
        for sc, i, s in sorted(pairs, reverse=True):                                 # 3. looks like someone unseen
            if out[i] is None and s not in taken:
                take(i, s, f'again (similarity {sc:.2f})')
        self.pend = [c for c in self.pend if now - c[3] <= self.gap]
        used = set()
        for i, (t, e, box) in enumerate(dets):                                       # 4. candidate -> new person after `confirm` frames
            if out[i] is not None or e is None:
                continue
            c = next((c for k, c in enumerate(self.pend) if k not in used and (iou(box, c[0]) >= self.overlap or (c[4] == ks[i] and float(c[1] @ e) >= self.sim))), None)
            if c is None:
                c = [box, e, 0, now, ks[i], {}, set()]      # ..., score per known person per frame, who was in view alongside
                self.pend.append(c)
            used.add(self.pend.index(c))
            c[0], c[1], c[2], c[3], c[4] = box, e, c[2] + 1, now, ks[i]
            c[6] |= taken
            for sid in self.gal:
                x = self._score(sid, e, ks[i], ps[i])
                if x is not None:
                    c[5].setdefault(sid, []).append(x)
            if c[2] >= self.confirm:
                # a near miss is checked over all the candidate's frames: one clear winner on average, and never in view alongside it
                avg = sorted(((sum(v) / len(v), sid) for sid, v in c[5].items() if len(v) >= self.confirm and sid not in c[6] and sid not in taken),
                             reverse=True)
                if avg and avg[0][0] >= self.sim - self.near and (len(avg) == 1 or avg[0][0] - avg[1][0] >= self.margin):
                    self.pend.remove(c)
                    take(i, avg[0][1], f'again (near miss, mean similarity {avg[0][0]:.2f} over {len(c[5][avg[0][1]])} frames)')
                    continue
                best = max(((x, s) for s in self.gal if s not in taken and (x := self._score(s, e, ks[i], ps[i])) is not None), default=None)
                self.pend.remove(c)
                take(i, self.next, 'is new' + (f' (closest unseen person {best[1]} at {best[0]:.2f}, ' + ('below' if best[0] < self.sim else 'not taken as a clear winner; sim') + f' {self.sim})' if best else ''))
                self.next += 1
        seen = sorted(i for i in out if i is not None)
        self.together |= {(a, b) for n, a in enumerate(seen) for b in seen[n + 1:]}
        if now - self.merged_t >= 1.0:
            self.merged_t = now
            self._merge_pass()
        return out


def board(book, now, rows=6, tw=40, th=80):
    """BGR picture of what the book remembers: one row per person (most recently seen first), one thumbnail per stored look."""
    ids = sorted(book.gal, key=lambda k: -book.gal[k][1])[:rows]
    img = np.full((max(1, len(ids)) * (th + 4), 130 + book.looks * tw, 3), 30, np.uint8)
    if not ids:
        cv2.putText(img, 'nobody remembered yet', (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
    for r, k in enumerate(ids):
        y, (looks, seen, _, thumbs, _) = r * (th + 4), book.gal[k]
        ago = now - seen
        col = (0, 255, 0) if ago < 1.0 else (0, 200, 255)
        cv2.putText(img, f'person {k}', (6, y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        cv2.putText(img, 'in view' if ago < 1.0 else f'{ago:.0f}s ago', (6, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.putText(img, f'{len(looks)} looks', (6, y + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        for c, t in enumerate(thumbs):
            if t is not None:
                img[y:y + th, 130 + c * tw:130 + (c + 1) * tw] = cv2.resize(t, (tw, th))
    return img
