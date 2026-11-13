import copy
import time

import rclpy
from person_detection_msgr.msg import PersonArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from person_detection.depth_ranging import depth_from_image, person_distance


class PersonDepthNode(Node):
    """persons + aligned depth image -> persons_ranged (same message, distance_m filled in).

    Point downstream consumers at the ranged topic (e.g. pid_extension persons_topic). Depth is
    taken from the most recent depth frame no older than max_age_sec (receipt time), so a
    stale or missing camera leaves distance_m at 0 ("unknown") instead of a wrong number.
    """

    def __init__(self):
        super().__init__('person_depth_node')
        for k, v in {'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                     'persons_in': 'persons', 'persons_out': 'persons_ranged',
                     'region_frac': 0.4, 'min_m': 0.2, 'max_m': 8.0, 'max_age_sec': 0.25}.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.depth = None            # (array, scale, monotonic receipt time)
        self.create_subscription(Image, p('depth_topic'), self._on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(PersonArray, p('persons_out'), 10)
        self.create_subscription(PersonArray, p('persons_in'), self._on_persons, 10)

    def _on_depth(self, m):
        try:
            arr, scale = depth_from_image(m.encoding, m.height, m.width, m.step, m.data,
                                          bool(m.is_bigendian))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        self.depth = (arr, scale, time.monotonic())

    def _on_persons(self, m):
        out = copy.deepcopy(m)
        fresh = self.depth is not None and \
            time.monotonic() - self.depth[2] <= self.get_parameter('max_age_sec').value
        for q in out.persons:
            q.distance_m = 0.0
            if fresh:
                arr, scale, _ = self.depth
                q.distance_m = person_distance(
                    arr, (q.x_min, q.y_min, q.x_max, q.y_max), (m.image_width, m.image_height),
                    scale, self.get_parameter('region_frac').value,
                    self.get_parameter('min_m').value, self.get_parameter('max_m').value)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
