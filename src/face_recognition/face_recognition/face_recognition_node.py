from face_recognition_msgr.msg import Face, FaceArray
from yolo11_common.base_node import Yolo11Node, run

from face_recognition.gallery import Gallery


class FaceRecognitionNode(Yolo11Node):
    """YOLO11 face detector -> crop -> YOLO11 embedding -> cosine match to gallery.

    Ultralytics does not ship face weights: pass a YOLO11 face model via `model`.
    Embeddings come from `embed_model` (default: a YOLO11 classifier) via model.embed().
    """

    def __init__(self):
        super().__init__('face_recognition_node', 'yolov11n-face.pt')
        self.declare_parameter('embed_model', 'yolo11n-cls.pt')
        self.declare_parameter('gallery_dir', '')
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('publish_embedding', False)
        from ultralytics import YOLO
        self.embedder = YOLO(self.get_parameter('embed_model').value)
        self.gallery = Gallery(self.get_parameter('gallery_dir').value)
        self.threshold = self.get_parameter('threshold').value
        self.pub_emb = self.get_parameter('publish_embedding').value
        self.pub = self.create_publisher(FaceArray, 'faces', 10)
        self.get_logger().info(f'Gallery: {len(self.gallery.names)} identities')

    def embed(self, crop):
        return self.embedder.embed(crop, verbose=False)[0].cpu().numpy()

    def process(self, frame, header):
        import cv2
        result = self.model.predict(frame, conf=self.conf, device=self.device,
                                    verbose=False)[0]
        out = FaceArray()
        out.header = header
        h, w = frame.shape[:2]
        out.image_height, out.image_width = h, w
        annotated = result.plot()
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
                cv2.putText(annotated, f'{f.identity} {f.similarity:.2f}',
                            (int(x1), max(12, int(y1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            out.faces.append(f)
        self.pub.publish(out)
        return annotated


def main(args=None):
    run(FaceRecognitionNode, args)


if __name__ == '__main__':
    main()
