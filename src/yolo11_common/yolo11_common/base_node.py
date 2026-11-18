"""Base node: image source (topic or webcam) + YOLO11 model loading."""
import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def image_to_bgr(msg: Image):
    """Decode a sensor_msgs/Image (bgr8/rgb8/mono8) into a BGR ndarray."""
    import numpy as np
    if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    ch = 1 if msg.encoding == 'mono8' else 3
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    arr = arr.reshape(msg.height, msg.width, ch)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return arr


def bgr_to_image(frame, header) -> Image:
    img = Image()
    img.header = header
    img.height, img.width = frame.shape[:2]
    img.encoding = 'bgr8'
    img.step = img.width * 3
    img.data = frame.tobytes()
    return img


def load_yolo(model: str, task: str, use_openvino: bool, logger):
    """Load a YOLO11 model; optionally export once to OpenVINO and reuse it."""
    from ultralytics import YOLO  # lazy: keeps the package importable without it
    if use_openvino:
        stem = os.path.splitext(model)[0]
        ov_dir = f'{stem}_openvino_model'
        if not os.path.isdir(ov_dir):
            logger.info(f'Exporting {model} to OpenVINO -> {ov_dir}')
            YOLO(model).export(format='openvino')
        return YOLO(ov_dir, task=task)
    return YOLO(model, task=task)


class Yolo11Node(Node):
    """Subclass and implement process(frame, header) -> annotated frame or None."""

    def __init__(self, name, default_model, task='detect'):
        super().__init__(name)
        self.declare_parameter('model', default_model)
        self.declare_parameter('openvino', False)
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('device', '')
        self.declare_parameter('image_topic', '')   # empty -> open the webcam
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('show', False)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('image_out', 'detections/image')   # annotated frames; one topic per node
        self.declare_parameter('raw_topic', '')   # webcam only: also publish the raw frames here for other nodes

        p = self.get_parameter
        self.conf = p('conf').value
        self.device = p('device').value or None
        self.show = p('show').value
        self.frame_id = p('frame_id').value
        self.model = load_yolo(p('model').value, task, p('openvino').value,
                               self.get_logger())
        self.image_pub = self.create_publisher(Image, p('image_out').value, 10)
        self.raw_pub = (self.create_publisher(Image, p('raw_topic').value, 1)
                        if p('raw_topic').value else None)

        topic = p('image_topic').value
        self.cap = None
        if topic:
            self.create_subscription(Image, topic, self._on_image, 1)
        else:
            self.cap = cv2.VideoCapture(p('camera_index').value)
            if not self.cap.isOpened():
                raise RuntimeError('Could not open webcam')
            self.create_timer(1.0 / p('rate_hz').value, self._on_timer)

    def _on_image(self, msg):
        try:
            self._handle(image_to_bgr(msg), msg.header)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def _on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=5.0)
            return
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        if self.raw_pub is not None and self.raw_pub.get_subscription_count() > 0:
            self.raw_pub.publish(bgr_to_image(frame, header))
        self._handle(frame, header)

    def _handle(self, frame, header):
        annotated = self.process(frame, header)
        if annotated is None:
            return
        if (self.get_parameter('publish_image').value
                and self.image_pub.get_subscription_count() > 0):
            self.image_pub.publish(bgr_to_image(annotated, header))
        if self.show:
            cv2.imshow(self.get_name(), annotated)
            cv2.waitKey(1)

    def process(self, frame, header):
        raise NotImplementedError

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def run(node_cls, args=None):
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
