import os
import time
from collections import defaultdict, deque

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from bt_common.executor_node import ExtensionExecutor, port_float, port_str, run_executor
from face_recognition_msgr.msg import FaceArray
from object_detection_msgr.msg import ObjectArray
from person_detection_msgr.msg import PersonArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from pose_extension import attributes, gestures
from pose_extension.categories import load_table, pick_next


def count_confident(items, min_conf, label=None):
    """How many detections reach min_conf (and carry `label`, when given)."""
    return sum(1 for i in items if i.confidence >= min_conf and (label is None or i.label == label))


SEAT_LABELS = ('chair', 'couch', 'bench')


def _overlap(a, b):
    """Fraction of box a (x_min, y_min, x_max, y_max) that box b covers."""
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    area = max(1e-9, (a[2] - a[0]) * (a[3] - a[1]))
    return max(0.0, w) * max(0.0, h) / area


def free_seats(objects, persons, labels=SEAT_LABELS, min_conf=0.4, max_covered=0.25):
    """Seat detections no person sits on: a seat box is taken when a person box covers more than `max_covered` of it.
    Returns the free seats (objects), biggest box first (bigger = closer, the best guess without depth)."""
    boxes = [(p.x_min, p.y_min, p.x_max, p.y_max) for p in persons]
    seats = [o for o in objects if o.label in labels and o.confidence >= min_conf
             and all(_overlap((o.x_min, o.y_min, o.x_max, o.y_max), b) <= max_covered for b in boxes)]
    return sorted(seats, key=lambda o: -(o.x_max - o.x_min) * (o.y_max - o.y_min))


def seat_place(label, bearing):
    """'chair on my left' from the image bearing (-1 left edge .. +1 right edge of the camera view); 'straight ahead' within +-0.25."""
    return f"{label} {'straight ahead' if abs(bearing) < 0.25 else 'on my left' if bearing < 0 else 'on my right'}"


def describe(persons, min_conf=0.5, track_id=None):
    """'a person wearing a red top' for the biggest (nearest) confident person that has a colour; '' if none."""
    cands = [p for p in persons if p.confidence >= min_conf and p.clothes_colour and (track_id is None or p.track_id == track_id)]
    if not cands:
        return ''
    best = max(cands, key=lambda p: (p.x_max - p.x_min) * (p.y_max - p.y_min))
    return f'a person wearing a {best.clothes_colour} top'


class PoseExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('pose_extension')
        for k, v in dict(persons_topic='persons', objects_topic='objects', window_sec=1.5, max_age_sec=0.7,
                         faces_topic='faces', image_topic='/camera/image_raw', face_parsing_model='',
                         min_face_px=attributes.MIN_FACE_PX, candidates_topic='/arm_eye/objects', categories_file='').items():
            self.declare_parameter(k, v)
        self.hist = defaultdict(lambda: deque(maxlen=120))   # track_id -> [(t, kps)]
        self.create_subscription(PersonArray, self.get_parameter('persons_topic').value,
                                 self._on_persons, 10)
        self.last = {}     # 'persons'/'objects' -> (monotonic time, message)
        self.create_subscription(ObjectArray, self.get_parameter('objects_topic').value,
                                 lambda m: self.last.__setitem__('objects', (time.monotonic(), m)), 10)
        self._cat, self._last_obj = None, ''
        self.create_subscription(ObjectArray, self.get_parameter('candidates_topic').value,
                                 lambda m: self.last.__setitem__('arm_objects', (time.monotonic(), m)), 10)
        self.parser = attributes.FaceParser(self.get_parameter('face_parsing_model').value)
        self.create_subscription(FaceArray, self.get_parameter('faces_topic').value,
                                 lambda m: self.last.__setitem__('faces', (time.monotonic(), m)), 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 lambda m: self.last.__setitem__('image', (time.monotonic(), m)), qos_profile_sensor_data)
        self.handlers = {'GetPersonPose': self._get_pose, 'IsPersonWaving': self._waving,
                         'IsPersonPointing': self._pointing, 'CountPeople': self._count_people,
                         'CountObjects': self._count_objects, 'FindFreeSeat': self._find_seat,
                         'DescribePerson': self._describe, 'DescribeAttributes': self._attributes, 'NextObject': self._next_object}

    def _on_persons(self, m):
        now = time.monotonic()
        self.last['persons'] = (now, m)
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
        n = count_confident(m.persons, mc)
        return True, f'{n} people', {'out_count': n}

    def _count_objects(self, ports, ctl):
        mc = port_float(ports, 'min_conf', 0.5, lo=0.0, hi=1.0)
        label = port_str(ports, 'label')
        m = self._fresh('objects')
        if m is None:
            return False, 'no recent object detections'
        n = count_confident(m.objects, mc, label)
        return True, f'{n} {label}', {'out_count': n}

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
        if not text:
            return False, 'nothing certain about this face'
        return True, text, {'out_text': text, 'out_glasses': a['glasses'], 'out_hair': a['hair']}

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
        seats = free_seats(objs.objects, pers.persons if pers else [], min_conf=mc)
        if not seats:
            return False, 'no free seat in view'
        best = seats[0]
        w = max(1, objs.image_width)
        bearing = ((best.x_min + best.x_max) / 2 - w / 2) / (w / 2)     # -1 left edge .. +1 right edge of the image
        place = seat_place(best.label, bearing)
        return True, f'free {place}', {'out_label': best.label, 'out_bearing': f'{bearing:.2f}', 'out_place': place}

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
