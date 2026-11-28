import math
import os
import time
from collections import defaultdict, deque

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from bt_common.executor_node import ExtensionExecutor, port_float, port_str, run_executor
from audio_msgr.msg import Transcript
from face_recognition_msgr.msg import FaceArray
from nav_msgs.msg import Odometry
from object_detection_msgr.msg import ObjectArray
from person_detection_msgr.msg import PersonArray
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from pose_extension import attributes, gestures
from pose_extension.genderage import face_square, GenderAge, vote
from pose_extension.stop import StopGate
from pose_extension.people import BAGS, dominant_colour, free_spots, location_phrase, PeopleBook, pointed_object, unique_description
from pose_extension.categories import load_table, pick_next


def count_confident(items, min_conf, label=None, colour=None):
    """How many detections reach min_conf (and carry `label` / a `clothes_colour`, when given)."""
    colour = (colour or '').lower().replace('gray', 'grey')          # the detector says grey (person_detection/colour.py); people say gray
    return sum(1 for i in items if i.confidence >= min_conf and (label is None or i.label == label)
               and (not colour or getattr(i, 'clothes_colour', '') == colour))


def _area(o):
    return (o.x_max - o.x_min) * (o.y_max - o.y_min)


# shortcut: size is the box area in the image, so it is only comparable for objects at about the same distance; upgrade with depth.
EXTREME = {'biggest': (_area, max), 'largest': (_area, max), 'smallest': (_area, min),
           'thinnest': (lambda o: min(o.x_max - o.x_min, o.y_max - o.y_min), min)}


def extreme_object(objects, prop, min_conf=0.5):
    """The confident detection that is the `prop` (biggest / largest / smallest / thinnest) by box size, or None."""
    key, pick = EXTREME[prop]
    cands = [o for o in objects if o.confidence >= min_conf]
    return pick(cands, key=key) if cands else None


def seat_place(label, bearing):
    """'chair on my left' from the image bearing (-1 left edge .. +1 right edge of the camera view); 'straight ahead' within +-0.25."""
    return f"{label} {'straight ahead' if abs(bearing) < 0.25 else 'on my left' if bearing < 0 else 'on my right'}"


def describe(persons, min_conf=0.5, track_id=None):
    """'a person wearing a red top' (and 'and blue bottoms' when the legs are in view) for the biggest (nearest) confident person that has
    a colour; '' if none. Only colours: shirt type, pattern and shoes are not judged (wrong information costs marks)."""
    cands = [p for p in persons if p.confidence >= min_conf and p.clothes_colour and (track_id is None or p.track_id == track_id)]
    if not cands:
        return ''
    best = max(cands, key=lambda p: (p.x_max - p.x_min) * (p.y_max - p.y_min))
    legs = getattr(best, 'legs_colour', '')
    return f'a person wearing a {best.clothes_colour} top' + (f' and {legs} bottoms' if legs else '')


class PoseExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('pose_extension')
        for k, v in dict(persons_topic='persons', objects_topic='objects', window_sec=1.5, max_age_sec=0.7,
                         faces_topic='faces', image_topic='/camera/image_raw', face_parsing_model='',
                         min_face_px=attributes.MIN_FACE_PX, candidates_topic='/arm_eye/objects', categories_file='', hfov_deg=69.0, genderage_model='', odom_topic='odom').items():
            self.declare_parameter(k, v)
        self.hist = defaultdict(lambda: deque(maxlen=120))   # track_id -> [(t, kps)]
        self.book = PeopleBook()    # who is in the room and what each looks like (stable ids), for unique descriptions, locations and names
        self.create_subscription(PersonArray, self.get_parameter('persons_topic').value,
                                 self._on_persons, 10)
        self.last = {}     # 'persons'/'objects' -> (monotonic time, message)
        self.create_subscription(ObjectArray, self.get_parameter('objects_topic').value,
                                 lambda m: self.last.__setitem__('objects', (time.monotonic(), m)), 10)
        self._cat, self._last_obj = None, ''
        self.create_subscription(ObjectArray, self.get_parameter('candidates_topic').value,
                                 lambda m: self.last.__setitem__('arm_objects', (time.monotonic(), m)), 10)
        self.parser = attributes.FaceParser(self.get_parameter('face_parsing_model').value)
        self.genderage = GenderAge(self.get_parameter('genderage_model').value)
        # the stop gate: where the followed person is (odom + depth), whether a hand is up, what was said, and who the operator is (pid_extension's FollowTarget)
        self.gate, self.operator_id, self.odom = StopGate(), None, None
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(Transcript, 'stt/transcript', lambda m: m.is_final and self.gate.heard(m.text, time.monotonic()), 10)
        self.create_subscription(Int32, 'follow/operator', lambda m: setattr(self, 'operator_id', m.data),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(FaceArray, self.get_parameter('faces_topic').value,
                                 lambda m: self.last.__setitem__('faces', (time.monotonic(), m)), 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 lambda m: self.last.__setitem__('image', (time.monotonic(), m)), qos_profile_sensor_data)
        self.handlers = {'GetPersonPose': self._get_pose, 'IsPersonWaving': self._waving,
                         'IsPersonPointing': self._pointing, 'CountPeople': self._count_people,
                         'CountObjects': self._count_objects, 'FindExtremeObject': self._extreme_object, 'FindFreeSeat': self._find_seat,
                         'DescribePerson': self._describe, 'DescribeAttributes': self._attributes, 'NextObject': self._next_object,
                         'FindPerson': self._find_person, 'DescribeUnique': self._describe_unique, 'DescribeLocation': self._describe_location,
                         'PointedObject': self._pointed_object, 'EstimateGender': self._estimate_gender, 'WaitForStop': self._wait_stop}

    def _on_persons(self, m):
        now = time.monotonic()
        self.last['persons'] = (now, m)
        if self.odom is not None:          # the stop gate: each person's place in the odom frame (depth + bearing) and whether a hand is up
            for p in m.persons:
                pos = None
                if p.distance_m > 0:
                    ex = ((p.x_min + p.x_max) / 2.0 - m.image_width / 2.0) / (m.image_width / 2.0)
                    b = -math.atan(ex * math.tan(math.radians(self.get_parameter('hfov_deg').value) / 2.0))
                    pos = (self.odom[0] + p.distance_m * math.cos(self.odom[2] + b), self.odom[1] + p.distance_m * math.sin(self.odom[2] + b))
                kp = list(zip(p.keypoints_x, p.keypoints_y, p.keypoints_conf)) if len(p.keypoints_x) >= 11 else []
                self.gate.see(p.track_id, pos, kp, now)
        f = self._fresh('faces')
        self.book.update(m.persons, f.faces if f else [], now, m.image_width, m.image_height, f.image_width if f else 0, f.image_height if f else 0)
        for p in m.persons:
            if len(p.keypoints_x) == len(p.keypoints_y) == len(p.keypoints_conf) > 0:
                kps = list(zip(p.keypoints_x, p.keypoints_y, p.keypoints_conf))
                self.hist[p.track_id].append((now, kps))

    def _tracks(self, ports):
        now = time.monotonic()
        max_age = self.get_parameter('max_age_sec').value
        want = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        return [(tid, [(t, k) for t, k in h if now - t <= self.get_parameter('window_sec').value])
                for tid, h in self.hist.items()
                if (want is None or tid == want) and h and now - h[-1][0] <= max_age]

    def _fresh(self, kind):
        t, m = self.last.get(kind, (0.0, None))
        return m if m is not None and time.monotonic() - t <= self.get_parameter('max_age_sec').value else None

    def _count_people(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        m = self._fresh('persons')
        if m is None:    # a silent detector is not an empty room
            return False, 'no recent person detections'
        colour = port_str(ports, 'colour', '')
        n = count_confident(m.persons, mc, colour=colour)
        return True, f'{n} people' + (f' wearing {colour}' if colour else ''), {'out_count': n}

    def _count_objects(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        label = port_str(ports, 'label')
        m = self._fresh('objects')
        if m is None:
            return False, 'no recent object detections'
        n = count_confident(m.objects, mc, label)
        return True, f'{n} {label}', {'out_count': n}

    def _extreme_object(self, ports, ctl):
        prop = port_str(ports, 'property')
        if prop not in EXTREME:
            return False, f"I can only compare size, not {prop}: use one of {', '.join(sorted(EXTREME))}"
        m = self._fresh('objects')
        if m is None:
            return False, 'no recent object detections'
        o = extreme_object(m.objects, prop, port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0))
        if o is None:
            return False, 'no object in view'
        return True, f'the {prop} object is the {o.label}', {'out_label': o.label}

    def _describe(self, ports, ctl):
        m = self._fresh('persons')
        if m is None:
            return False, 'no recent person detections'
        want = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        text = describe(m.persons, port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0), want)
        return (True, text, {'out_text': text}) if text else (False, 'no person with a visible clothes colour')

    def _attributes(self, ports, ctl):
        """Glasses / hair colour / hat / earrings of the biggest confident face, from the latest camera frame (see attributes.py)."""
        from yolo11_common.base_node import image_to_bgr
        mc = port_float(ports, 'min_conf', 0.6, lo=0.0, hi=1.0)
        faces, img = self._fresh('faces'), self._fresh('image')
        if faces is None or img is None:
            return False, 'no recent face detections or camera frame'
        big = [f for f in faces.faces if f.confidence >= mc and f.x_max - f.x_min >= self.get_parameter('min_face_px').value]
        if not big:
            return False, 'no face close enough to judge'
        f = max(big, key=lambda f: (f.x_max - f.x_min) * (f.y_max - f.y_min))
        frame = image_to_bgr(img)
        sx, sy = frame.shape[1] / max(1, faces.image_width), frame.shape[0] / max(1, faces.image_height)
        box = (f.x_min * sx, f.y_min * sy, f.x_max * sx, f.y_max * sy)
        x1, y1, x2, y2 = attributes.head_crop(box, frame.shape[1], frame.shape[0])
        crop = frame[y1:y2, x1:x2]
        try:
            a = attributes.attributes(self.parser.parse(crop), crop, (box[2] - box[0]) * (box[3] - box[1]))
        except RuntimeError as e:
            return False, str(e)
        text = attributes.sentence(a)
        self.book.remember_at(((box[0] + box[2]) / 2 * self._person_scale()[0], (box[1] + box[3]) / 2 * self._person_scale()[1]),
                              hair=a['hair'], glasses=a['glasses'] if a['glasses'] == 'yes' else '')
        if not text:
            return False, 'nothing certain about this face'
        return True, text, {'out_text': text, 'out_glasses': a['glasses'], 'out_hair': a['hair']}

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _wait_stop(self, ports, ctl):
        """Succeeds when the operator tells the robot to stop: the word "stop" (or one of the comma separated `words`) or a raised hand held up, and only while the
        operator being followed (follow/operator from FollowTarget) stands still. Anyone else's "stop", or one said while the operator walks, is ignored. Use in a
        Parallel next to FollowTarget. The hand sign is a raised hand, not counted fingers."""
        self.gate.words = [w for w in port_str(ports, 'words', 'stop').split(',') if w.strip()]
        self.gate.still_sec = port_float(ports, 'still_sec', 1.5, lo=0.5, hi=10.0)
        self.gate.voice_t = None
        end = time.monotonic() + port_float(ports, 'timeout_ms', 900000.0, lo=1000.0, hi=3600000.0) / 1000.0
        while True:
            if ctl.cancelled:
                return False, 'cancelled'
            why = self.gate.check(self.operator_id, time.monotonic())
            if why:
                return True, f'stop ({"heard" if why == "voice" else "hand held up"}) while the operator stood still'
            if time.monotonic() > end:
                return False, 'timeout'
            ctl.sleep(0.1)

    def _person_scale(self):
        """Factor from the camera frame's pixels to the person detector's pixels (1 when they are the same image)."""
        pm, im = self._fresh('persons'), self._fresh('image')
        return (pm.image_width / max(1, im.width), pm.image_height / max(1, im.height)) if pm is not None and im is not None else (1.0, 1.0)

    def _known(self, ports):
        want = int(port_float(ports, 'track_id')) if ports.get('track_id') else None
        return self.book.people.get(want) if want is not None else self.book.nearest(time.monotonic())

    def _find_person(self, ports, ctl):
        """The track id of the person remembered under a name (a face the gallery recognised), if they are in view now; then FollowTarget track_id
        (timeout only, no approach) turns the base to look at exactly that person."""
        name = port_str(ports, 'who')
        k = self.book.by_name(name)
        if k is None or time.monotonic() - k.seen > port_float(ports, 'max_age_sec', 2.0, lo=0.1, hi=600.0):
            return False, f'{name} is not in view'
        return True, f'{name} is person {k.id}', {'out_track_id': str(k.id)}

    def _describe_unique(self, ports, ctl):
        """The fewest remembered features nobody else in the room shares (colours of the top and bottoms, hair, glasses, sitting or standing)."""
        k = self._known(ports)
        if k is None:
            return False, 'no person known'
        text, unique = unique_description(self.book, k.id, int(port_float(ports, 'min_features', 2, lo=1, hi=5)),
                                          int(port_float(ports, 'max_features', 4, lo=1, hi=5)))
        if not text:
            return False, 'nothing certain about this person yet'
        return True, text + ('' if unique else ' (not unique)'), {'out_text': text, 'out_unique': 'true' if unique else 'false', 'out_track_id': str(k.id)}

    def _describe_location(self, ports, ctl):
        """Where a person is, from the objects around them ('sitting on the couch', 'next to the tv'); fails when nothing is near enough to say."""
        k, objs, pm = self._known(ports), self._fresh('objects'), self._fresh('persons')
        if k is None or k.box is None or objs is None or pm is None:
            return False, 'no person or no recent object detections'
        sx, sy = pm.image_width / max(1, objs.image_width), pm.image_height / max(1, objs.image_height)
        things = [(o.label, (o.x_min * sx, o.y_min * sy, o.x_max * sx, o.y_max * sy)) for o in objs.objects if o.confidence >= 0.4]
        text = location_phrase(k.box, things, k.features().get('posture'))
        return (True, text, {'out_text': text, 'out_track_id': str(k.id)}) if text else (False, 'nothing near enough to name')

    def _pointed_object(self, ports, ctl):
        """Which bag the nearest person is pointing at: the extended arm's ray against the bag boxes (backpack, handbag, suitcase)."""
        pm, objs = self._fresh('persons'), self._fresh('objects')
        if pm is None or objs is None:
            return False, 'no recent person or object detections'
        labels = tuple(w.strip() for w in port_str(ports, 'labels', ','.join(BAGS)).split(',') if w.strip())
        cands = [p for p in pm.persons if p.confidence >= 0.5 and len(p.keypoints_x) >= 11]
        if not cands:
            return False, 'nobody with a visible arm'
        p = max(cands, key=lambda p: (p.x_max - p.x_min) * (p.y_max - p.y_min))
        sx, sy = pm.image_width / max(1, objs.image_width), pm.image_height / max(1, objs.image_height)
        things = [(o.label, (o.x_min * sx, o.y_min * sy, o.x_max * sx, o.y_max * sy)) for o in objs.objects if o.confidence >= 0.3]
        hit = pointed_object(list(zip(p.keypoints_x, p.keypoints_y, p.keypoints_conf)), things, labels)
        if hit is None:
            return False, 'no bag on the line of the pointing arm'
        label, (x1, y1, x2, y2) = hit
        bearing = ((x1 + x2) / 2 - pm.image_width / 2) / (pm.image_width / 2)
        colour = ''
        img = self._fresh('image')
        if img is not None:
            from yolo11_common.base_node import image_to_bgr
            frame = image_to_bgr(img)
            fx, fy = frame.shape[1] / pm.image_width, frame.shape[0] / pm.image_height
            colour = dominant_colour(frame[max(0, int(y1 * fy)):int(y2 * fy), max(0, int(x1 * fx)):int(x2 * fx)])
        place = seat_place(f'{colour} {label}' if colour else label, bearing)        # as the robot sees it: 'red backpack on my left'
        return True, f'pointing at the {place}', {'out_label': label, 'out_colour': colour, 'out_place': place, 'out_bbox': f'{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}', 'out_bearing': f'{bearing:.2f}'}

    def _estimate_gender(self, ports, ctl):
        """Man or woman, he or she, and about how old, from the biggest face in view: three looks 0.3 s apart, voted; fails when the looks disagree or no
        face is close enough (never guesses). InsightFace genderage model, accuracy on this camera unmeasured."""
        from yolo11_common.base_node import image_to_bgr
        mc = port_float(ports, 'min_conf', 0.6, lo=0.0, hi=1.0)
        readings = []
        for k in range(3):
            faces, img = self._fresh('faces'), self._fresh('image')
            big = [f for f in (faces.faces if faces else []) if f.confidence >= mc and f.x_max - f.x_min >= self.get_parameter('min_face_px').value]
            if big and img is not None:
                f = max(big, key=lambda f: (f.x_max - f.x_min) * (f.y_max - f.y_min))
                frame = image_to_bgr(img)
                sx, sy = frame.shape[1] / max(1, faces.image_width), frame.shape[0] / max(1, faces.image_height)
                x1, y1, x2, y2 = face_square((f.x_min * sx, f.y_min * sy, f.x_max * sx, f.y_max * sy), frame.shape[1], frame.shape[0])
                try:
                    readings.append(self.genderage.read(frame[y1:y2, x1:x2]))
                except RuntimeError as e:
                    return False, str(e)
            if k < 2:
                ctl.sleep(0.3)
        if not readings:
            return False, 'no face close enough to judge'
        gender, age = vote(readings)
        if not gender:
            return False, 'the looks disagree or are unclear'
        return True, f'{gender}, about {age}', {'out_value': gender, 'out_pronoun': 'he' if gender == 'man' else 'she', 'out_age': str(age)}

    def _next_object(self, ports, ctl):
        """The next object the arm eye sees that this task handles, and where it goes (param/object_categories.yaml)."""
        task = port_str(ports, 'task')
        try:
            if self._cat is None:
                path = self.get_parameter('categories_file').value or os.path.join(
                    get_package_share_directory('framework_bringup'), 'param', 'object_categories.yaml')
                self._cat = load_table(path)
        except (ValueError, PackageNotFoundError) as e:
            return False, str(e)
        rows = self._cat.get(task)
        if not rows:
            return False, f'no objects configured for task {task!r} (known: {", ".join(sorted(self._cat))})'
        m = self._fresh('arm_objects')
        if m is None:
            return False, 'no recent arm-eye object detections (tools/arm/arm_eye.sh)'
        o, e = pick_next(m.objects, rows, self._last_obj, port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0))
        if o is None:
            return False, f'no {task} object in view'
        self._last_obj = o.label
        return True, f"{o.label} ({e['group']}) -> {e['nav']} / {e['put']}", {
            'out_label': o.label, 'out_nav': e['nav'], 'out_put': e['put'], 'out_group': e['group']}

    def _find_seat(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.4, lo=0.0, hi=1.0)
        objs, pers = self._fresh('objects'), self._fresh('persons')
        if objs is None:
            return False, 'no recent object detections'
        seats = [(o.label, (o.x_min, o.y_min, o.x_max, o.y_max), o.confidence) for o in objs.objects if o.label in ('chair', 'couch', 'bench')]
        spots = free_spots(seats, [(p.x_min, p.y_min, p.x_max, p.y_max) for p in (pers.persons if pers else [])], min_conf=mc)
        if not spots:
            return False, 'no free seat in view'
        label, cx, _, shared, _ = spots[0]
        w = max(1, objs.image_width)
        bearing = (cx - w / 2) / (w / 2)     # -1 left edge .. +1 right edge of the image: the free spot itself, not the middle of the couch
        place = ('free part of the ' if shared else '') + seat_place(label, bearing)
        radians = -math.atan(bearing * math.tan(math.radians(self.get_parameter('hfov_deg').value) / 2.0))      # + = turn left, as RotateBy takes it
        return True, f'free {place}', {'out_label': label, 'out_bearing': f'{bearing:.2f}', 'out_place': place, 'out_radians': f'{radians:.2f}'}

    def _get_pose(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        ok = any(h and gestures.has_pose(h[-1][1], mc) for _, h in self._tracks(ports))
        return ok, 'pose visible' if ok else 'no usable pose'

    def _waving(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        ok = any(gestures.waving(h, mc) for _, h in self._tracks(ports))
        return ok, 'waving' if ok else 'not waving'

    def _pointing(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        d = port_str(ports, 'direction', '')
        if d not in ('', 'left', 'right'):
            return False, "direction must be 'left' or 'right'"
        for _, h in self._tracks(ports):
            if h and (res := gestures.pointing(h[-1][1], mc, d)):
                return True, f'pointing {res}'
        return False, 'not pointing'


def main(args=None):
    run_executor(PoseExtension, args)


if __name__ == '__main__':
    main()
