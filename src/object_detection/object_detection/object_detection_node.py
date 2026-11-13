import json

from object_detection_msgr.msg import Object, ObjectArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from yolo11_common.base_node import run, Yolo11Node


class ObjectDetectionNode(Yolo11Node):
    def __init__(self):
        super().__init__('object_detection_node', 'yolo11n.pt')
        self.pub = self.create_publisher(ObjectArray, 'objects', 10)
        # A fine-tuned model has its own classes: announce them (latched) so the validator and
        # the LLM only use labels this detector can actually produce.
        self.classes = String(data=json.dumps(
            {'classes': [str(n) for _, n in sorted(self.model.names.items())]}))
        self.cls_pub = self.create_publisher(String, 'object_detection/classes', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cls_pub.publish(self.classes)
        self.create_timer(5.0, lambda: self.cls_pub.publish(self.classes))
        self.get_logger().info(f'{len(self.model.names)} classes: '
                               f'{", ".join(str(n) for n in self.model.names.values())}')

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
