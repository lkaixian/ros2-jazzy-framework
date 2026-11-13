import math, sys, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

class Sim(Node):
    def __init__(self):
        super().__init__('sim_robot')
        self.x = self.y = self.yaw = 0.0; self.v = self.w = 0.0
        self.max_v = self.max_w = 0.0; self.last = time.monotonic()
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.2, self.dump)
    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z
        self.max_v = max(self.max_v, abs(self.v)); self.max_w = max(self.max_w, abs(self.w))
    def tick(self):
        now = time.monotonic(); dt = now - self.last; self.last = now
        self.x += self.v * math.cos(self.yaw) * dt; self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        o = Odometry(); o.header.frame_id = 'odom'; o.header.stamp = self.get_clock().now().to_msg()
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pub.publish(o)
    def dump(self):
        open(sys.argv[1], 'w').write(f'{self.x:.3f} {self.y:.3f} {self.yaw:.3f} v={self.v:.3f} w={self.w:.3f} maxv={self.max_v:.3f} maxw={self.max_w:.3f}\n')
rclpy.init(); n = Sim()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
