"""Republish /scan as /scan_filtered with every beam that lands inside the robot's own body set to inf.

The robot's standoffs, supports and cables return at 0.10-0.23 m from the base centre. Nav2's collision_monitor (no minimum
range on its scan source) counted them as an obstacle inside the footprint and kept cutting the speed; the costmaps use it too.
The beam position is tested in the robot frame (TF base <- laser), so the mount values are not duplicated here. Without that TF
the scan is passed through unchanged (the simulator keeps working). Blind zone: anything closer than `mask_radius` to the base
centre is ignored by Nav2; the bumper is the backstop.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def mask_ranges(ranges, angle_min, angle_inc, lx, ly, lyaw, radius):
    """ranges in the laser frame; (lx, ly, lyaw) = laser pose in the base frame. -> (ranges with the body beams = inf, count masked)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r)
    rr = np.where(ok, r, 0.0)
    px, py = rr * np.cos(a), rr * np.sin(a)
    c, s = math.cos(lyaw), math.sin(lyaw)
    bx, by = lx + px * c - py * s, ly + px * s + py * c
    inside = ok & (bx * bx + by * by < radius * radius)
    out = r.copy()
    out[inside] = np.inf
    return out, int(inside.sum())


class ScanMaskNode(Node):
    def __init__(self):
        super().__init__('scan_mask_node')
        for k, v in dict(input_topic='scan', output_topic='scan_filtered', base_frame='base', mask_radius=0.27,
                         tf_wait_sec=5.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.base, self.radius, self.wait = p('base_frame'), float(p('mask_radius')), float(p('tf_wait_sec'))
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pose, self.t0, self.warned = None, time.monotonic(), False
        self.pub = self.create_publisher(LaserScan, p('output_topic'), qos_profile_sensor_data)
        self.create_subscription(LaserScan, p('input_topic'), self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'masking beams within {self.radius} m of {self.base}: {p("input_topic")} -> {p("output_topic")}')

    def _lookup(self, frame):
        try:
            t = self.buf.lookup_transform(self.base, frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001  not available (yet): tf2 raises several types
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def _on_scan(self, m):
        if self.pose is None:
            self.pose = self._lookup(m.header.frame_id)
        if self.pose is None:
            if time.monotonic() - self.t0 < self.wait:
                return                                   # still waiting for the static TF
            if not self.warned:
                self.get_logger().warn(f'no TF {self.base} <- {m.header.frame_id}: passing the scan through unmasked')
                self.warned = True
            self.pub.publish(m)
            return
        out, _ = mask_ranges(m.ranges, m.angle_min, m.angle_increment, *self.pose, self.radius)
        o = LaserScan()
        o.header, o.angle_min, o.angle_max, o.angle_increment = m.header, m.angle_min, m.angle_max, m.angle_increment
        o.time_increment, o.scan_time, o.range_min, o.range_max = m.time_increment, m.scan_time, m.range_min, m.range_max
        o.ranges, o.intensities = out.astype(np.float32).tolist(), m.intensities
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
