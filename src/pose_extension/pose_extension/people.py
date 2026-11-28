"""Who is in the room and what each one looks like, remembered across frames; unique descriptions; where a person is; which bag is pointed at.

ROS-free. A person is the stable id from person_detection (re-identification), so what is learned about someone stays with them while they sit
still, turn, or leave and come back. Only what the detectors report is said (clothes colours, hair colour and glasses from the face parser, sitting or
standing from the keypoints, a name from a recognised face): nothing is guessed. shortcut: colours as words, no height, no age or gender model.
"""
import math
from collections import Counter

from bt_common.kalman import Kalman2D

SEATS = ('chair', 'couch', 'bench', 'bed')
LANDMARKS = ('tv', 'potted plant', 'dining table', 'refrigerator', 'oven', 'sink', 'bed', 'laptop', 'book', 'vase', 'clock', 'bottle', 'cup')
BAGS = ('backpack', 'handbag', 'suitcase')
L_SH, R_SH, L_HIP, R_HIP, L_KNEE, R_KNEE = 5, 6, 11, 12, 13, 14


def posture(kps, min_conf=0.4):
    """'sitting' when the knees are about as high as the hips, 'standing' when they hang well below, None when the hips or knees are not seen.
    kps: [(x, y, conf)] COCO-17 in image coordinates (y grows downward)."""
    pts = {i: kps[i] for i in (L_SH, R_SH, L_HIP, R_HIP, L_KNEE, R_KNEE) if i < len(kps) and kps[i][2] >= min_conf}
    if not all(i in pts for i in (L_HIP, R_HIP)) or not any(i in pts for i in (L_KNEE, R_KNEE)):
        return None
    hip_y = (pts[L_HIP][1] + pts[R_HIP][1]) / 2
    knee_y = sum(pts[i][1] for i in (L_KNEE, R_KNEE) if i in pts) / sum(i in pts for i in (L_KNEE, R_KNEE))
    sh = [pts[i][1] for i in (L_SH, R_SH) if i in pts]
    torso = hip_y - sum(sh) / len(sh) if sh else None
    if not torso or torso <= 0:
        return None
    drop = (knee_y - hip_y) / torso            # a standing person's knee is about as far below the hip as the hip is below the shoulder
    return 'sitting' if drop < 0.45 else 'standing' if drop > 0.7 else None


class Known:
    """What is known about one person id."""

    def __init__(self, pid):
        self.id, self.name, self.box, self.seen = pid, '', None, 0.0
        self.votes = {'top': Counter(), 'legs': Counter(), 'posture': Counter(), 'hair': Counter(), 'glasses': Counter()}
        self.extra = {}
        self.kf = None      # Kalman filter of the person's position in the odom frame (needs depth and odometry)

    def vote(self, key, value, weight=1):
        if value:
            self.votes[key][value] += weight

    def features(self):
        """{'top': 'blue', 'legs': 'black', 'hair': 'brown', 'glasses': 'yes', 'posture': 'sitting', ...}: the most voted value of each, when it has at least 2 votes
        (one odd frame is not a fact)."""
        out = {k: c.most_common(1)[0][0] for k, c in self.votes.items() if c and c.most_common(1)[0][1] >= 2}
        out.update(self.extra)
        return out


def _inside(x, y, box):
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


class PeopleBook:
    def __init__(self, keep_sec=900.0):
        self.keep, self.people = keep_sec, {}

    def update(self, persons, faces, now, width, height, fw=0, fh=0, pose=None, hfov_deg=69.0):
        """persons: PersonArray.persons; faces: FaceArray.faces (their coordinates are in fw x fh pixels, the persons' in width x height). pose = the robot's (x, y, yaw)
        in the odom frame: with it every depth-ranged person gets a Kalman filter of their place, so they can still be pointed at while hidden for a moment."""
        for pid in [k for k, v in self.people.items() if now - v.seen > self.keep]:
            del self.people[pid]
        sx, sy = (width / fw if fw else 1.0), (height / fh if fh else 1.0)
        for p in persons:
            if p.track_id < 0:
                continue
            k = self.people.setdefault(p.track_id, Known(p.track_id))
            k.box, k.seen = (p.x_min, p.y_min, p.x_max, p.y_max), now
            if pose is not None and p.distance_m > 0 and width:
                b = -math.atan(((p.x_min + p.x_max) / 2.0 - width / 2.0) / (width / 2.0) * math.tan(math.radians(hfov_deg) / 2.0))
                pos = (pose[0] + p.distance_m * math.cos(pose[2] + b), pose[1] + p.distance_m * math.sin(pose[2] + b))
                if k.kf is None:
                    k.kf = Kalman2D(pos, now)
                else:
                    k.kf.update(pos, now, 0.03 + 0.03 * p.distance_m)
            k.vote('top', p.clothes_colour)
            k.vote('legs', getattr(p, 'legs_colour', ''))
            if len(p.keypoints_x) == len(p.keypoints_y) == len(p.keypoints_conf) > 0:
                k.vote('posture', posture(list(zip(p.keypoints_x, p.keypoints_y, p.keypoints_conf))))
        for f in faces:
            if f.identity in ('', 'unknown') or f.similarity < 0.5:
                continue
            owner = self.owner(((f.x_min + f.x_max) / 2 * sx, (f.y_min + f.y_max) / 2 * sy))
            if owner is not None:
                owner.name = f.identity

    def owner(self, xy):
        """The person (smallest box) whose box holds the image point, or None."""
        inside = [k for k in self.people.values() if k.box and _inside(xy[0], xy[1], k.box)]
        return min(inside, key=lambda k: (k.box[2] - k.box[0]) * (k.box[3] - k.box[1])) if inside else None

    def remember_at(self, xy, **kv):
        """Attach attributes (hair, glasses, age ...) to the person whose box holds the point (e.g. a face's centre)."""
        k = self.owner(xy)
        if k is None:
            return None
        for key, value in kv.items():
            if key in k.votes:
                k.vote(key, value, 2)
            elif value:
                k.extra[key] = value
        return k.id

    def bearing(self, k, pose, now, width, hfov_deg=69.0, fresh=0.4):
        """(radians, direct) from the robot's forward (+ = left) to person k: straight from the box in the picture when they were seen within `fresh` s, else from the Kalman
        prediction (at most 2 s past the last sighting) and the robot's current pose; None when neither is available."""
        if k.box is not None and width and now - k.seen <= fresh:
            return -math.atan(((k.box[0] + k.box[2]) / 2.0 - width / 2.0) / (width / 2.0) * math.tan(math.radians(hfov_deg) / 2.0)), True
        if k.kf is not None and pose is not None and now - k.seen <= 2.0:
            px, py = k.kf.position(now)
            a = math.atan2(py - pose[1], px - pose[0]) - pose[2]
            return math.atan2(math.sin(a), math.cos(a)), False
        return None

    def bearings(self, pose, now, width, hfov_deg=69.0, max_age=3.0):
        """{'<name>' or '#<id>': {'rad', 'age', 'direct'}, 'nearest': {...}} for everyone seen within max_age s, for the arm to point at ('nearest' = the biggest box in view)."""
        out = {}
        for k in self.people.values():
            b = self.bearing(k, pose, now, width, hfov_deg)
            if b is not None and now - k.seen <= max_age:
                out[k.name.lower() if k.name else f'#{k.id}'] = {'rad': round(b[0], 3), 'age': round(now - k.seen, 2), 'direct': b[1]}
        near = self.nearest(now)
        if near is not None:
            b = self.bearing(near, pose, now, width, hfov_deg)
            if b is not None:
                out['nearest'] = {'rad': round(b[0], 3), 'age': round(now - near.seen, 2), 'direct': b[1]}
        return out

    def by_name(self, name):
        for k in self.people.values():
            if k.name and k.name.lower() == name.lower():
                return k
        return None

    def nearest(self, now, max_age=1.0):
        """The biggest person box seen within max_age s."""
        cands = [k for k in self.people.values() if k.box and now - k.seen <= max_age]
        return max(cands, key=lambda k: (k.box[2] - k.box[0]) * (k.box[3] - k.box[1])) if cands else None


_PRIORITY = ('top', 'legs', 'hair', 'glasses', 'posture')


def _phrase(feats):
    """{'top': 'blue', 'legs': 'black', 'hair': 'brown', 'glasses': 'yes', 'posture': 'sitting'} -> 'wearing a blue top and black bottoms, with brown hair and glasses, sitting'"""
    wear = [f'a {feats["top"]} top'] if 'top' in feats else []
    wear += [f'{feats["legs"]} bottoms'] if 'legs' in feats else []
    withs = ([f'{feats["hair"]} hair'] if 'hair' in feats else []) + (['glasses'] if feats.get('glasses') == 'yes' else [])
    parts = ['wearing ' + ' and '.join(wear)] if wear else []
    parts += ['with ' + ' and '.join(withs)] if withs else []
    parts += [feats['posture']] if 'posture' in feats else []
    return ', '.join(parts)


def unique_description(book, pid, min_features=2, max_features=4):
    """(text, unique): the fewest features of person `pid` that no other known person shares, at least `min_features` and at most `max_features`;
    unique is False when even all of them do not tell this person apart from someone else (then say so, do not pretend)."""
    me = book.people.get(pid)
    if me is None:
        return '', False
    mine = {k: v for k, v in me.features().items() if k in _PRIORITY and v}
    others = [o.features() for o in book.people.values() if o.id != pid]
    shared = {k: sum(o.get(k) == v for o in others) for k, v in mine.items()}
    order = sorted(mine, key=lambda k: (shared[k], _PRIORITY.index(k)))     # the rarest features first
    chosen = []
    for k in order:
        chosen.append(k)
        alike = [o for o in others if all(o.get(c) == mine[c] for c in chosen)]
        if len(chosen) >= max_features or (not alike and len(chosen) >= min_features):
            break
    alike = [o for o in others if all(o.get(c) == mine[c] for c in chosen)]
    chosen.sort(key=_PRIORITY.index)
    return _phrase({k: mine[k] for k in chosen}), not alike


def location_phrase(box, objects, sitting=None):
    """Where a person is, from the detected objects around them: 'sitting on the couch', 'standing in front of the couch', 'next to the tv', or '' when
    nothing is near enough to say. objects: [(label, (x1, y1, x2, y2))] in the same pixels as `box`."""
    px1, py1, px2, py2 = box
    pw = max(1.0, px2 - px1)
    for label, (x1, y1, x2, y2) in objects:
        overlap = min(px2, x2) - max(px1, x1)
        if label in SEATS and overlap >= 0.5 * pw and y1 <= py2 <= y2 + 0.25 * (py2 - py1):
            return f'sitting on the {label}' if sitting != 'standing' else f'standing in front of the {label}'
    best = None
    for label, (x1, y1, x2, y2) in objects:
        if label in SEATS and sitting == 'sitting':
            continue
        gap = max(x1 - px2, px1 - x2, 0.0)
        if (label in LANDMARKS or label in SEATS) and gap <= 1.5 * pw and (best is None or gap < best[0]):
            best = (gap, label)
    return f'{sitting + " " if sitting else ""}next to the {best[1]}'.strip() if best else ''


def pointed_object(kps, objects, labels=BAGS, min_conf=0.4, max_angle=30.0):
    """The object (label, box) the extended arm points at: the ray from the elbow through the wrist, compared with the direction from the wrist to each
    candidate's centre; the smallest angle under max_angle degrees wins. kps: [(x, y, conf)]; objects: [(label, box)]. None when no arm is out or none lines up."""
    arms = []
    for sh, el, wr in ((5, 7, 9), (6, 8, 10)):
        if max(sh, el, wr) < len(kps) and all(kps[i][2] >= min_conf for i in (sh, el, wr)):
            arms.append((math.hypot(kps[wr][0] - kps[sh][0], kps[wr][1] - kps[sh][1]), kps[el], kps[wr]))
    if not arms:
        return None
    _, e, w = max(arms, key=lambda a: a[0])            # the arm held out furthest from the shoulder
    ray = (w[0] - e[0], w[1] - e[1])
    if math.hypot(*ray) < 1e-6:
        return None
    best = None
    for label, (x1, y1, x2, y2) in objects:
        if label not in labels:
            continue
        v = ((x1 + x2) / 2 - w[0], (y1 + y2) / 2 - w[1])
        n = math.hypot(*v)
        if n < 1e-6:
            continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (ray[0] * v[0] + ray[1] * v[1]) / (math.hypot(*ray) * n)))))
        if ang <= max_angle and (best is None or ang < best[0]):
            best = (ang, (label, (x1, y1, x2, y2)))
    return best[1] if best else None


WIDE_SEATS = ('couch', 'bench')       # seat several people: free room is looked for beside the people on it


def free_spots(seats, people, min_conf=0.4, chair_covered=0.25, slot_fraction=0.4):
    """Places a guest could sit, from detected seats and the boxes of the people near them: [(label, cx, width, shared, seat_box)], widest first.

    A chair is one place, free when people cover at most `chair_covered` of it. A couch or bench is cut into as many seats as people of the width of those
    sitting on it fit (3 for a sofa three people wide), and a seat is free when no person covers more than `slot_fraction` of it: a host in the MIDDLE of a
    three-seater leaves both ends, a host as wide as half the sofa leaves nothing. A sofa nobody sits on is one place. `shared` is True for a seat of a
    sofa somebody already uses ("the free end"). The seat box is the detector's, backrest and arms included, and a person's box includes their arms and
    shoulders, so the count of seats is an estimate: a coat or bag on a seat is NOT seen, and a person leaning on the back may count as sitting.
    seats: [(label, (x1, y1, x2, y2), conf)]; people: [(x1, y1, x2, y2)]."""
    out = []
    for label, (x1, y1, x2, y2), conf in seats:
        if conf < min_conf or label not in ('chair', 'couch', 'bench'):
            continue
        w, h = x2 - x1, max(1.0, y2 - y1)
        on = [p for p in people if min(x2, p[2]) - max(x1, p[0]) > 0.15 * (p[2] - p[0]) and p[3] > y1 + 0.3 * h and p[1] < y2]   # overlaps it and reaches low into it
        if label in WIDE_SEATS:
            if not on:
                out.append((label, (x1 + x2) / 2, w, False, (x1, y1, x2, y2)))
                continue
            pw = sorted(p[2] - p[0] for p in on)[len(on) // 2]                       # the typical width of a person sitting there
            n = max(1, round(w / (1.15 * pw)))
            sw = w / n
            for k in range(n):
                a, b = x1 + k * sw, x1 + (k + 1) * sw
                if all(max(0.0, min(b, p[2]) - max(a, p[0])) <= slot_fraction * sw for p in on):
                    out.append((label, (a + b) / 2, sw, True, (x1, y1, x2, y2)))
        else:
            covered = sum(max(0.0, min(x2, p[2]) - max(x1, p[0])) * max(0.0, min(y2, p[3]) - max(y1, p[1])) for p in people) / max(1e-9, w * h)
            if covered <= chair_covered:
                out.append((label, (x1 + x2) / 2, w, False, (x1, y1, x2, y2)))
    return sorted(out, key=lambda s: (-s[2], s[1]))


def dominant_colour(bgr):
    """The colour word of the middle of a BGR crop (a bag): black / white / grey / red / orange / yellow / green / cyan / blue / purple / pink; '' when the crop is too
    small. The median of the middle 60%, so straps, a hand or the floor around it do not decide. shortcut: one word, no patterns, a two-coloured bag gets its larger half."""
    import cv2
    import numpy as np
    h, w = bgr.shape[:2]
    if h < 8 or w < 8:
        return ''
    c = bgr[int(0.2 * h):int(0.8 * h), int(0.2 * w):int(0.8 * w)]
    hh, ss, vv = (int(np.median(cv2.cvtColor(c, cv2.COLOR_BGR2HSV).reshape(-1, 3)[:, i])) for i in range(3))
    if vv < 50:
        return 'black'
    if ss < 40:
        return 'white' if vv > 190 else 'grey'
    if hh < 8 or hh >= 170:
        return 'red'
    for top, name in ((20, 'orange'), (35, 'yellow'), (85, 'green'), (100, 'cyan'), (130, 'blue'), (150, 'purple'), (170, 'pink')):
        if hh < top:
            return name
    return 'red'
