"""The path the followed person walked, as breadcrumbs in the odom frame; steer to a crumb a little ahead instead of straight at the person. ROS-free.

Driving straight at the person cuts corners (through a barrier, a doorframe, a crowd the person went around). Following their path keeps the robot where
they have already shown there is room. shortcut: odometry only (drifts over tens of metres), crumbs from depth-ranged detections, no smoothing.
"""
import math


class Trail:
    def __init__(self, spacing=0.25, max_len=400):
        self.spacing, self.max_len = spacing, max_len
        self.crumbs = []

    def add(self, pos):
        """pos = (x, y) of the person; kept only when at least `spacing` m from the previous crumb."""
        if not self.crumbs or math.hypot(pos[0] - self.crumbs[-1][0], pos[1] - self.crumbs[-1][1]) >= self.spacing:
            self.crumbs.append((float(pos[0]), float(pos[1])))
            del self.crumbs[:-self.max_len]

    def target(self, robot_xy, lookahead=0.8):
        """The first crumb at least `lookahead` m from the robot among those from the robot's own place on the trail onward (the crumbs before the one
        nearest to it are behind it and dropped); the newest crumb when all are closer; None for an empty trail.
        shortcut: nearest crumb = the robot's place, wrong only if the person walked a loop that passes the robot."""
        if not self.crumbs:
            return None
        near = min(range(len(self.crumbs)), key=lambda i: math.hypot(self.crumbs[i][0] - robot_xy[0], self.crumbs[i][1] - robot_xy[1]))
        del self.crumbs[:near]
        for c in self.crumbs:
            if math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]) >= lookahead:
                return c
        return self.crumbs[-1]


def trail_error(pose, wp, hfov_deg, limit=1.5):
    """The image-offset error (-1 left edge .. +1 right edge, as FollowTarget's turn controller takes it) that a point `wp` in the odom frame would
    have for a camera at pose = (x, y, yaw): positive = the point is to the right, so the robot must turn right."""
    ang = math.atan2(wp[1] - pose[1], wp[0] - pose[0]) - pose[2]
    ang = math.atan2(math.sin(ang), math.cos(ang))
    return max(-limit, min(limit, -math.tan(ang) / math.tan(math.radians(hfov_deg) / 2.0))) if abs(ang) < math.radians(80) else math.copysign(limit, -ang)
