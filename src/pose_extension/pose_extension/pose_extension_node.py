import time
from collections import defaultdict, deque

from bt_common.executor_node import ExtensionExecutor, port_float, port_str, run_executor
from person_detection_msgr.msg import PersonArray

from pose_extension import gestures


class PoseExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('pose_extension')
        for k, v in dict(persons_topic='persons', window_sec=1.5, max_age_sec=0.7).items():
            self.declare_parameter(k, v)
        self.hist = defaultdict(lambda: deque(maxlen=120))   # track_id -> [(t, kps)]
        self.create_subscription(PersonArray, self.get_parameter('persons_topic').value,
                                 self._on_persons, 10)
        self.handlers = {'GetPersonPose': self._get_pose, 'IsPersonWaving': self._waving,
                         'IsPersonPointing': self._pointing}

    def _on_persons(self, m):
        now = time.monotonic()
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
