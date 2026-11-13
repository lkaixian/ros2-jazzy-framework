"""Unicycle model with acceleration limits and the Kobuki's command timeout. No ROS imports."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def approach(current, target, max_delta):
    """Move `current` toward `target` by at most `max_delta`."""
    d = target - current
    return target if abs(d) <= max_delta else current + math.copysign(max_delta, d)


class Robot:
    """Pose in the map frame plus commanded/actual velocity.

    cmd_timeout: like the Kobuki driver, a command older than this is treated as zero.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, max_lin_acc=1.5, max_ang_acc=4.0, cmd_timeout=0.6):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0                 # actual velocities
        self.cmd_v = self.cmd_w = 0.0
        self.cmd_age = float('inf')
        self.max_lin_acc, self.max_ang_acc = max_lin_acc, max_ang_acc
        self.cmd_timeout = cmd_timeout
        self.odom_x = self.odom_y = self.odom_yaw = 0.0   # integrated in the odom frame

    def command(self, v, w):
        self.cmd_v, self.cmd_w, self.cmd_age = v, w, 0.0

    def step(self, dt, collides=lambda x, y: False):
        """Advance dt seconds. Returns True if a collision stopped the motion."""
        self.cmd_age += dt
        tv, tw = (self.cmd_v, self.cmd_w) if self.cmd_age <= self.cmd_timeout else (0.0, 0.0)
        self.v = approach(self.v, tv, self.max_lin_acc * dt)
        self.w = approach(self.w, tw, self.max_ang_acc * dt)
        yaw = wrap(self.yaw + self.w * dt)
        mid = self.yaw + 0.5 * self.w * dt
        nx, ny = self.x + self.v * math.cos(mid) * dt, self.y + self.v * math.sin(mid) * dt
        blocked = collides(nx, ny)
        if blocked:
            self.v = 0.0                      # bump: stop translating, keep turning
            nx, ny = self.x, self.y
        # odometry integrates the same increments in the odom frame (frame origin = start pose)
        dx, dy = nx - self.x, ny - self.y
        c, s = math.cos(-self._start_yaw()), math.sin(-self._start_yaw())
        self.odom_x += c * dx - s * dy
        self.odom_y += s * dx + c * dy
        self.odom_yaw = wrap(self.odom_yaw + wrap(yaw - self.yaw))
        self.x, self.y, self.yaw = nx, ny, yaw
        return blocked

    _start = None

    def set_start(self):
        """Call once after setting the initial pose: odom frame = start pose."""
        self._start = self.yaw
        self.odom_x = self.odom_y = self.odom_yaw = 0.0

    def _start_yaw(self):
        return self._start if self._start is not None else 0.0
