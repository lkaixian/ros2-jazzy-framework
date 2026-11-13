import json
import time
from collections import defaultdict, deque

import rclpy
from face_recognition_msgr.msg import FaceArray
from object_detection_msgr.msg import ObjectArray
from person_detection_msgr.msg import PersonArray
from rclpy.node import Node
from std_msgs.msg import String

from bt_orchestrator.scene import describe_person


class SceneSummarizer(Node):
    """Aggregates perception topics into one compact JSON string for the LLM."""

    def __init__(self):
        super().__init__('scene_summarizer_node')
        self.declare_parameter('window_sec', 2.0)
        self.declare_parameter('period_sec', 1.0)
        self.objects = {}   # label -> (best conf, last seen)
        self.persons = {}   # track_id -> last seen
        self.pose = defaultdict(lambda: deque(maxlen=60))   # track_id -> [(t, keypoints)]
        self.dist = {}      # track_id -> last distance_m
        self.faces = {}     # identity -> (best similarity, last seen)
        self.create_subscription(ObjectArray, 'objects', self._on_obj, 10)
        self.create_subscription(PersonArray, 'persons', self._on_person, 10)
        self.create_subscription(FaceArray, 'faces', self._on_face, 10)
        self.pub = self.create_publisher(String, 'scene/summary', 10)
        self.create_timer(self.get_parameter('period_sec').value, self._publish)

    def _on_obj(self, m):
        t = time.monotonic()
        for o in m.objects:
            self.objects[o.label] = (max(o.confidence, self.objects.get(o.label, (0,))[0]), t)

    def _on_person(self, m):
        t = time.monotonic()
        for p in m.persons:
            self.persons[p.track_id] = t
            self.dist[p.track_id] = p.distance_m
            if len(p.keypoints_x) == len(p.keypoints_y) == len(p.keypoints_conf) > 0:
                self.pose[p.track_id].append(
                    (t, list(zip(p.keypoints_x, p.keypoints_y, p.keypoints_conf))))

    def _on_face(self, m):
        t = time.monotonic()
        for f in m.faces:
            self.faces[f.identity] = (max(f.similarity,
                                          self.faces.get(f.identity, (-1,))[0]), t)

    def _publish(self):
        now, win = time.monotonic(), self.get_parameter('window_sec').value
        fresh = lambda t: now - t <= win  # noqa: E731
        scene = {
            'objects': sorted(k for k, (_, t) in self.objects.items() if fresh(t)),
            'num_persons': sum(1 for t in self.persons.values() if fresh(t)),
            # per person: distance and gestures, only what is actually known (see scene.py)
            'people': [describe_person(i, [(t, k) for t, k in self.pose[i] if fresh(t)],
                                       self.dist.get(i, 0.0))
                       for i, t in sorted(self.persons.items()) if fresh(t)],
            'known_faces': sorted(k for k, (_, t) in self.faces.items()
                                  if fresh(t) and k != 'unknown'),
        }
        self.pub.publish(String(data=json.dumps(scene)))


def main(args=None):
    rclpy.init(args=args)
    node = SceneSummarizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
