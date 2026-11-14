from yolo11_common.base_node import Yolo11Node, run

from person_detection.colour import clothes_colour
from person_detection_msgr.msg import Person, PersonArray


class PersonDetectionNode(Yolo11Node):
    """Detects (and optionally tracks) COCO class 0. A *-pose model adds keypoints."""

    def __init__(self):
        super().__init__('person_detection_node', 'yolo11n-pose.pt', task='pose')
        self.declare_parameter('track', True)
        self.track = self.get_parameter('track').value
        self.pub = self.create_publisher(PersonArray, 'persons', 10)

    def process(self, frame, header):
        kw = dict(conf=self.conf, device=self.device, classes=[0], verbose=False)
        result = (self.model.track(frame, persist=True, **kw)[0] if self.track
                  else self.model.predict(frame, **kw)[0])
        out = PersonArray()
        out.header = header
        out.image_height, out.image_width = frame.shape[:2]
        kpts = getattr(result, 'keypoints', None)
        for i, box in enumerate(result.boxes):
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            p = Person(track_id=int(box.id) if box.id is not None else -1,
                       confidence=float(box.conf),
                       x_min=x1, y_min=y1, x_max=x2, y_max=y2)
            if kpts is not None and kpts.xy is not None and len(kpts.xy) > i:
                xy = kpts.xy[i].cpu().numpy()
                cf = kpts.conf[i].cpu().numpy() if kpts.conf is not None else None
                p.keypoints_x = [float(v) for v in xy[:, 0]]
                p.keypoints_y = [float(v) for v in xy[:, 1]]
                p.keypoints_conf = [float(v) for v in cf] if cf is not None else []
            p.clothes_colour = clothes_colour(frame, (x1, y1, x2, y2), p.keypoints_x or None, p.keypoints_y or None,
                                              p.keypoints_conf or None)
            out.persons.append(p)
        self.pub.publish(out)
        return result.plot()


def main(args=None):
    run(PersonDetectionNode, args)


if __name__ == '__main__':
    main()
