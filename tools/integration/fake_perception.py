import sys, rclpy
from rclpy.node import Node
from face_recognition_msgr.msg import Face, FaceArray
from face_recognition_msgr.srv import Enroll
from person_detection_msgr.msg import Person, PersonArray
IDENT = sys.argv[1] if len(sys.argv) > 1 else 'alice'
class P(Node):
    def __init__(self):
        super().__init__('fake_perception')
        self.fp = self.create_publisher(FaceArray, 'faces', 10)
        self.pp = self.create_publisher(PersonArray, 'persons', 10)
        self.enrolled = []
        self.create_service(Enroll, 'face/enroll', self.enroll)
        self.create_timer(0.1, self.tick)
    def tick(self):
        pa = PersonArray(); pa.image_width, pa.image_height = 640, 480
        pa.persons.append(Person(track_id=1, confidence=0.9, x_min=100., y_min=50., x_max=300., y_max=450.))
        self.pp.publish(pa)
        fa = FaceArray(); fa.image_width, fa.image_height = 640, 480
        fa.faces.append(Face(confidence=0.95, x_min=150., y_min=60., x_max=250., y_max=170., identity=IDENT, similarity=0.88, embedding=[0.1, 0.2, 0.3]))
        self.fp.publish(fa)
    def enroll(self, req, res):
        self.enrolled.append(req.name); res.ok, res.message = True, f'stub enrolled {req.name}'
        self.get_logger().info(f'ENROLLED {req.name}'); return res
rclpy.init(); n = P()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
