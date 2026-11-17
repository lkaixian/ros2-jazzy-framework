import time

from face_recognition_msgr.msg import Face, FaceArray
from face_recognition_msgr.srv import Enroll
from yolo11_common.base_node import Yolo11Node, run

from face_recognition.gallery import Gallery, valid_name


class FaceRecognitionNode(Yolo11Node):
    """YOLO11 face detector -> crop -> YOLO11 embedding -> cosine match to gallery.

    Ultralytics does not ship face weights: pass a YOLO11 face model via `model`.
    Embeddings come from `embed_model` (default: a YOLO11 classifier) via model.embed().
    Service face/enroll stores the largest face of the last frame under a name.
    """

    def __init__(self):
        super().__init__('face_recognition_node', 'yolov11n-face.pt')
        self.declare_parameter('embed_model', 'yolo11n-cls.pt')
        self.declare_parameter('gallery_dir', '')
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('publish_embedding', True)    # BT ExtractFaceFeature needs it
        self.declare_parameter('enroll_max_age_sec', 2.0)
        from ultralytics import YOLO
        self.embedder = YOLO(self.get_parameter('embed_model').value)
        self.gallery = Gallery(self.get_parameter('gallery_dir').value)
        self.threshold = self.get_parameter('threshold').value
        self.pub_emb = self.get_parameter('publish_embedding').value
        self.last = None          # (monotonic time, embedding) of the largest face
        self.pub = self.create_publisher(FaceArray, 'faces', 10)
        self.create_service(Enroll, 'face/enroll', self._on_enroll)
        self.get_logger().info(f'Gallery: {len(self.gallery.names)} identities')

    def embed(self, crop):
        return self.embedder.embed(crop, verbose=False)[0].cpu().numpy()

    def _on_enroll(self, req, res):
        if not valid_name(req.name):
            res.ok, res.message = False, 'invalid name: use 1-40 of A-Z a-z 0-9 _ - (not "unknown")'
            return res
        if self.last is None or (time.monotonic() - self.last[0]
                                 > self.get_parameter('enroll_max_age_sec').value):
            res.ok, res.message = False, 'no face seen in the last frames'
            return res
        try:
            self.gallery.add(req.name, self.last[1])
        except (ValueError, OSError) as e:
            res.ok, res.message = False, str(e)
            return res
        res.ok, res.message = True, f'enrolled {req.name} ({len(self.gallery.names)} identities)'
        self.get_logger().info(res.message)
        return res

    def process(self, frame, header):
        import cv2
        result = self.model.predict(frame, conf=self.conf, device=self.device,
                                    verbose=False)[0]
        out = FaceArray()
        out.header = header
        h, w = frame.shape[:2]
        out.image_height, out.image_width = h, w
        annotated = result.plot(labels=False)      # our own identity label below; the built-in one sits on top of it
        biggest, biggest_emb = 0.0, None
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            f = Face(confidence=float(box.conf), x_min=x1, y_min=y1, x_max=x2, y_max=y2,
                     identity='unknown')
            crop = frame[max(0, int(y1)):int(y2), max(0, int(x1)):int(x2)]
            if crop.size:
                emb = self.embed(crop)
                f.identity, f.similarity = self.gallery.match(emb, self.threshold)
                if self.pub_emb:
                    f.embedding = [float(v) for v in emb]
                if (x2 - x1) * (y2 - y1) > biggest:
                    biggest, biggest_emb = (x2 - x1) * (y2 - y1), emb
                label = f'{f.identity} {f.similarity:.2f}'
                pos = (max(0, int(x1)), min(h - 6, int(y2) + 20))     # below the box: never hidden by it
                col = (0, 255, 0) if f.identity != 'unknown' else (0, 200, 255)
                cv2.putText(annotated, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
                cv2.putText(annotated, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
            out.faces.append(f)
        if biggest_emb is not None:
            self.last = (time.monotonic(), biggest_emb)
        self.pub.publish(out)
        return annotated


def main(args=None):
    run(FaceRecognitionNode, args)


if __name__ == '__main__':
    main()
