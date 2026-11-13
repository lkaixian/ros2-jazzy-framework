from yolo11_common.base_node import Yolo11Node, run

from object_detection_msgr.msg import Object, ObjectArray


class ObjectDetectionNode(Yolo11Node):
    def __init__(self):
        super().__init__('object_detection_node', 'yolo11n.pt')
        self.pub = self.create_publisher(ObjectArray, 'objects', 10)

    def process(self, frame, header):
        result = self.model.predict(frame, conf=self.conf, device=self.device,
                                    verbose=False)[0]
        out = ObjectArray()
        out.header = header
        out.image_height, out.image_width = frame.shape[:2]
        for i, box in enumerate(result.boxes):
            cls = int(box.cls)
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            out.objects.append(Object(
                class_id=cls, id=i, label=result.names[cls], confidence=float(box.conf),
                x_min=x1, y_min=y1, x_max=x2, y_max=y2))
        self.pub.publish(out)
        return result.plot()


def main(args=None):
    run(ObjectDetectionNode, args)


if __name__ == '__main__':
    main()
