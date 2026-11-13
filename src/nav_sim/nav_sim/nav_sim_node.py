"""nav_sim_node: simulated Kobuki + RPLIDAR on a map, for testing Nav2/SLAM without a robot.

Subscribes: <cmd_vel_topic> (Twist)
Publishes:  /scan (LaserScan, frame laser), /odom (Odometry), TF odom -> base,
            ground truth on /nav_sim/ground_truth (PoseStamped, map frame)
The map->odom transform is NOT published: AMCL or a SLAM node does that, exactly as on the robot.
The base -> laser static transform comes from navigation.launch.py (static_tf in navigation.yaml),
so it must match the lidar_* parameters here.
"""
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from nav_sim.kinematics import Robot
from nav_sim.occupancy import OccupancyMap


def default_map():
    return os.path.join(get_package_share_directory('framework_bringup'), 'maps', 'test_room.yaml')


class NavSim(Node):
    def __init__(self):
        super().__init__('nav_sim_node')
        defaults = dict(map_yaml='', cmd_vel_topic='cmd_vel', odom_frame='odom', base_frame='base',
                        laser_frame='laser', start_x=0.5, start_y=0.0, start_yaw=0.0,
                        robot_radius=0.17, lidar_x=0.0, lidar_y=0.0, lidar_z=0.25, lidar_yaw=0.0,
                        lidar_range_min=0.15, lidar_range_max=12.0, lidar_beams=360,
                        scan_rate_hz=10.0, scan_noise_std=0.01, physics_hz=50.0,
                        cmd_timeout_sec=0.6, max_lin_acc=1.5, max_ang_acc=4.0,
                        odom_slip=0.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.map = OccupancyMap.from_yaml(p('map_yaml') or default_map())
        self.radius = p('robot_radius')
        if self.map.collides(p('start_x'), p('start_y'), self.radius):
            raise ValueError(f"start pose ({p('start_x')}, {p('start_y')}) is inside an obstacle "
                             f'or outside the free space of the map')
        self.robot = Robot(p('start_x'), p('start_y'), p('start_yaw'), p('max_lin_acc'),
                           p('max_ang_acc'), p('cmd_timeout_sec'))
        self.robot.set_start()
        self.rng = np.random.default_rng(0)
        n = int(p('lidar_beams'))
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        self.tf = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.gt_pub = self.create_publisher(PoseStamped, 'nav_sim/ground_truth', 10)
        self.create_subscription(Twist, p('cmd_vel_topic'), self._on_cmd, 10)
        self.dt = 1.0 / p('physics_hz')
        self.last = time.monotonic()
        self.bumps = 0
        self.create_timer(self.dt, self._physics)
        self.create_timer(1.0 / p('scan_rate_hz'), self._scan)
        self.get_logger().info(
            f"map {self.map.w}x{self.map.h} cells @ {self.map.res} m, start "
            f"({p('start_x')}, {p('start_y')}, {p('start_yaw')}), cmd on {p('cmd_vel_topic')}")

    def _on_cmd(self, m):
        self.robot.command(m.linear.x, m.angular.z)

    def _physics(self):
        now = time.monotonic()
        dt, self.last = now - self.last, now
        if self.robot.step(dt, lambda x, y: self.map.collides(x, y, self.radius)):
            self.bumps += 1
            self.get_logger().warn('collision: translation blocked', throttle_duration_sec=2.0)
        stamp = self.get_clock().now().to_msg()
        r = self.robot
        slip = self.get_parameter('odom_slip').value
        t = TransformStamped()
        t.header.stamp, t.header.frame_id = stamp, self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value
        t.transform.translation.x, t.transform.translation.y = r.odom_x, r.odom_y
        yaw = r.odom_yaw * (1.0 + slip)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header, o.child_frame_id = t.header, t.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y = r.odom_x, r.odom_y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = r.v, r.w
        self.odom_pub.publish(o)
        g = PoseStamped()
        g.header.stamp, g.header.frame_id = stamp, 'map'
        g.pose.position.x, g.pose.position.y = r.x, r.y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(r.yaw / 2), math.cos(r.yaw / 2)
        self.gt_pub.publish(g)

    def _scan(self):
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        r = self.robot
        ly = p('lidar_yaw')
        lx = r.x + p('lidar_x') * math.cos(r.yaw) - p('lidar_y') * math.sin(r.yaw)
        lyp = r.y + p('lidar_x') * math.sin(r.yaw) + p('lidar_y') * math.cos(r.yaw)
        rng = self.map.raycast(lx, lyp, r.yaw + ly, self.angles, p('lidar_range_min'),
                               p('lidar_range_max'))
        noise = p('scan_noise_std')
        if noise > 0:
            rng = rng + self.rng.normal(0.0, noise, rng.shape)
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = p('laser_frame')
        m.angle_min, m.angle_max = float(self.angles[0]), float(self.angles[-1])
        m.angle_increment = float(self.angles[1] - self.angles[0])
        m.time_increment = 0.0
        m.scan_time = 1.0 / p('scan_rate_hz')
        m.range_min, m.range_max = p('lidar_range_min'), p('lidar_range_max')
        m.ranges = [float(x) if np.isfinite(x) else float('inf') for x in rng]
        self.scan_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = NavSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
