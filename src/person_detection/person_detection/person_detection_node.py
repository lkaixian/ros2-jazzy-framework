import os
import time

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolo11_common.base_node import Yolo11Node, bgr_to_image, run

from person_detection.colour import clothes_colour, legs_box
from person_detection.parts import part_histograms
from person_detection.reid import UPPER, IdentityBook, Reid, board, is_cut, usable
from person_detection_msgr.msg import Person, PersonArray


class PersonDetectionNode(Yolo11Node):
    """Detects (and optionally tracks) COCO class 0. A *-pose model adds keypoints."""

    def __init__(self):
        super().__init__('person_detection_node', 'yolo26s-pose.pt', task='pose')
        self.declare_parameter('track', True)
        self.track = self.get_parameter('track').value
        self.declare_parameter('reid_model', '')   # path of the OSNet ONNX (models/reid/osnet_ain_x1_0.onnx); empty = no appearance vector
        path = self.get_parameter('reid_model').value
        self.reid = Reid(path) if path else None
        self.declare_parameter('reid_sim', 0.70)         # cosine similarity at which a new tracker id is taken for a person seen before
        self.declare_parameter('reid_keep_sec', 900.0)   # how long an unseen person's appearance is remembered
        self.declare_parameter('blur_ratio', 0.6)   # a frame sharper than this x recent sharpness is used for identity; blurrier ones only hold links
        self.sharp = []
        self.declare_parameter('colour_weight', 0.4)   # share of the colour of the torso/trousers (vs the network's vector) in the same-person score
        self.declare_parameter('dump_dir', '')         # save one crop per second per person here (dir/<id>/<ms>_c<0|1>.jpg) for tools/reid/compare.py; '' = off
        self.declare_parameter('reid_confirm_frames', 5)   # frames a new track must last, matching nobody, before it gets an id
        self.book = IdentityBook(self.get_parameter('reid_sim').value, self.get_parameter('reid_keep_sec').value,
                                 self.get_parameter('reid_confirm_frames').value, colour_weight=self.get_parameter('colour_weight').value)
        self.dumped = {}
        self.pub = self.create_publisher(PersonArray, 'persons', 10)
        self.declare_parameter('memory_topic', 'detections/reid_memory')   # picture of what the identity book remembers ('' = off)
        if self.reid is not None and self.get_parameter('memory_topic').value:
            self.mem_pub = self.create_publisher(Image, self.get_parameter('memory_topic').value, 1)
            self.create_timer(0.5, lambda: self.mem_pub.publish(
                bgr_to_image(board(self.book, time.monotonic()), Header(stamp=self.get_clock().now().to_msg()))))

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
            legs = legs_box((x1, y1, x2, y2), p.keypoints_x or None, p.keypoints_y or None, p.keypoints_conf or None)
            p.legs_colour = clothes_colour(frame, (x1, y1, x2, y2), region=legs) if legs else ''
            out.persons.append(p)
        blurry = False
        if self.reid is not None:
            g = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2GRAY)
            sharp = float(cv2.Laplacian(g, cv2.CV_32F).var())
            blurry = len(self.sharp) >= 15 and sharp < self.get_parameter('blur_ratio').value * float(np.percentile(self.sharp, 75))
            self.sharp = (self.sharp + [sharp])[-90:]
        plot_kw = {}
        if self.reid is not None and not blurry:
            crops = [frame[max(0, int(p.y_min)):int(p.y_max), max(0, int(p.x_min)):int(p.x_max)] for p in out.persons]
            parts = [part_histograms(frame, (p.x_min, p.y_min, p.x_max, p.y_max), p.keypoints_x, p.keypoints_y, p.keypoints_conf)
                     for p in out.persons]
            ok = [i for i, (c, q) in enumerate(zip(crops, out.persons))     # no shoulders = a fragment (feet, a thigh): nothing to recognise
                  if c.size and usable((q.x_min, q.y_min, q.x_max, q.y_max)) and 'torso' in parts[i]]
            size = (frame.shape[1], frame.shape[0])
            up = [i for i in ok if not is_cut((out.persons[i].x_min, out.persons[i].y_min, out.persons[i].x_max, out.persons[i].y_max), size)
                  and usable((0, 0, crops[i].shape[1], UPPER * crops[i].shape[0]))]       # whole bodies: also their torso view
            tops = [crops[i][:int(UPPER * crops[i].shape[0])] for i in up]
            vecs = self.reid.embed([crops[i] for i in ok] + tops)
            for i, e in zip(ok, vecs):
                out.persons[i].embedding = [float(v) for v in e]
            uppers = [None] * len(crops)
            for i, u, top in zip(up, vecs[len(ok):], tops):
                uppers[i] = (u, cv2.resize(top, (40, 80)))
            dets = [(p.track_id, np.asarray(p.embedding) if len(p.embedding) else None, (p.x_min, p.y_min, p.x_max, p.y_max))
                    for p in out.persons]
            thumbs = [cv2.resize(crops[i], (40, 80)) if i in ok else None for i in range(len(crops))]
            now = time.monotonic()
            ids = self.book.assign(dets, now, thumbs, size, uppers, parts)
            dump = self.get_parameter('dump_dir').value
            for i, sid in enumerate(ids if dump else ()):
                if sid is not None and i in ok and now - self.dumped.get(sid, -9.0) >= 1.0:
                    self.dumped[sid] = now
                    os.makedirs(os.path.join(dump, str(sid)), exist_ok=True)
                    cv2.imwrite(os.path.join(dump, str(sid), f'{int(now * 1000)}_c{is_cut(dets[i][2], size)}.jpg'), crops[i])
            for e in self.book.events:
                self.get_logger().info(e)
            self.book.events.clear()
            keep = [(p, sid) for p, sid in zip(out.persons, ids) if sid is not None]   # an unconfirmed track is not published
            out.persons = [p for p, _ in keep]
            for p, sid in keep:
                p.track_id = sid                  # the tracker's own number is replaced by the stable one
            plot_kw = dict(labels=False)              # the plot would print the tracker's ids
        elif blurry:
            # a swung or shaken camera smears the crops: no new looks, no new people, no broken links; the ids stay as they were
            ids = self.book.assign([(p.track_id, None, (p.x_min, p.y_min, p.x_max, p.y_max)) for p in out.persons], time.monotonic())
            keep = [(p, sid) for p, sid in zip(out.persons, ids) if sid is not None]
            out.persons = [p for p, _ in keep]
            for p, sid in keep:
                p.track_id = sid
            plot_kw = dict(labels=False)
        self.pub.publish(out)
        img = result.plot(**plot_kw)
        if blurry:
            cv2.putText(img, 'blurry: ids held', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        for p in out.persons if plot_kw else ():
            cv2.putText(img, f'id {p.track_id}', (int(p.x_min) + 4, int(p.y_min) + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return img


def main(args=None):
    run(PersonDetectionNode, args)


if __name__ == '__main__':
    main()
