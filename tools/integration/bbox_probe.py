import json, sys, time, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_detection_msgr.msg import ObjectArray
from yolo11_common.base_node import bgr_to_image, image_to_bgr
IMG, OUT = sys.argv[1], sys.argv[2]
class P(Node):
    def __init__(self):
        super().__init__('bbox_probe')
        self.frame = cv2.imread(IMG)
        self.pub = self.create_publisher(Image, 'cam/image_raw', 1)
        self.got_objs = None; self.got_img = False
        self.create_subscription(ObjectArray, 'objects', self.on_obj, 10)
        self.create_subscription(Image, 'detections/image', self.on_img, 10)
        self.create_timer(0.3, self.tick)
    def tick(self):
        m = bgr_to_image(self.frame, Image().header); m.header.frame_id = 'camera'; self.pub.publish(m)
    def on_obj(self, m):
        if self.got_objs is None:
            self.got_objs = {'w': m.image_width, 'h': m.image_height, 'objects': [
                dict(label=o.label, conf=round(o.confidence, 3), box=[round(o.x_min, 1), round(o.y_min, 1), round(o.x_max, 1), round(o.y_max, 1)]) for o in m.objects]}
    def on_img(self, m):
        if not self.got_img:
            cv2.imwrite(OUT, image_to_bgr(m)); self.got_img = True
rclpy.init(); n = P(); end = time.time() + 40
while rclpy.ok() and time.time() < end and not (n.got_objs is not None and n.got_img):
    rclpy.spin_once(n, timeout_sec=0.1)
print(json.dumps(n.got_objs)); print("annotated image saved:", n.got_img)
