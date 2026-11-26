"""RealSense depth image -> a thin PointCloud2 for the Nav2 costmaps and the collision monitor.

The 2D lidar sees one slice at ~14 cm; the robot is ~0.9 m tall. This gives Nav2 what is in front of the camera at the
robot's whole height. Every `stride`-th pixel only (640x480 / 4 -> ~19k points), so three subscribers stay cheap.
The cloud stays in the camera's optical frame: Nav2 applies base -> camera_link (param/navigation.yaml) and the height
filter (min/max_obstacle_height), so a wrong camera pitch is a TF error, not a hidden one here.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32


def depth_to_points(depth, k, stride=4, min_m=0.3, max_m=3.0):
    """depth: HxW metres (0/nan = no reading); k = (fx, fy, cx, cy). -> N x 3 float32, optical axes (x right, y down, z forward)."""
    fx, fy, cx, cy = k
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(np.float32)
    ok = np.isfinite(z) & (z >= min_m) & (z <= max_m)
    z, u, v = z[ok], u[ok], v[ok]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1).astype(np.float32)


class DepthCloudNode(Node):
    def __init__(self):
        super().__init__('depth_cloud_node')
        for k, v in dict(depth_topic='/camera/camera/depth/image_rect_raw',
                         info_topic='/camera/camera/depth/camera_info', output_topic='depth_cloud',
                         stride=4, min_range=0.3, max_range=3.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.k = None
        self.pub = self.create_publisher(PointCloud2, p('output_topic'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('info_topic'), self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p('depth_topic'), self._on_depth, qos_profile_sensor_data)

    def _on_info(self, m):
        self.k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        if self.k is None:
            return
        if m.encoding == '16UC1':
            depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width] * 0.001
        elif m.encoding == '32FC1':
            depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width]
        else:
            self.get_logger().error(f'unsupported depth encoding {m.encoding}', throttle_duration_sec=10.0)
            return
        pts = depth_to_points(depth, self.k, self.get_parameter('stride').value,
                              self.get_parameter('min_range').value, self.get_parameter('max_range').value)
        self.pub.publish(create_cloud_xyz32(m.header, pts))


def main(args=None):
    rclpy.init(args=args)
    node = DepthCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
