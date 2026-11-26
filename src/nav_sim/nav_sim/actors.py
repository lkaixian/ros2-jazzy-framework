"""People and overhead obstacles for nav_sim: scripted walkers, what a camera would see of them, and a depth cloud of boxes the lidar
slice misses. Pure numpy/yaml, no ROS. World coordinates are metres in the map frame.

Scenario file (nav_sim `scenario:=<file.yaml>`):
  actors:                       # people: circles that walk a path, block the robot, show in /scan and in the camera
    - {name: leader, id: 1, colour: red, radius: 0.22, speed: 0.25, start_after: 2.0, path: [[2, 1], [5, 1], [5, 4]]}
  overhead:                     # boxes (x, y = centre, w along x, d along y, z0..z1 above the floor) the 2D lidar cannot see
    - {x: 3.0, y: 1.0, w: 0.8, d: 0.6, z0: 0.55, z1: 0.8}
An actor with `loop: true` walks the path back and forth; otherwise it stops at the end. It never waits for the robot.
"""
import math

import numpy as np
import yaml

CAMERA = dict(x=0.16, y=-0.08, z=0.755, hfov_deg=69.0, width=640, height=480)    # RealSense mount, notes/ROBOT_GEOMETRY.md
PERSON_HEIGHT = 1.7


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Actor:
    """A person walking a polyline at constant speed, in the map frame."""

    def __init__(self, name, track_id, path, speed=0.3, radius=0.22, colour='', start_after=0.0, loop=False, height=PERSON_HEIGHT):
        if len(path) < 1:
            raise ValueError(f'actor {name}: empty path')
        self.name, self.track_id, self.radius, self.colour, self.height = name, int(track_id), float(radius), colour, float(height)
        self.path = [(float(x), float(y)) for x, y in path]
        self.speed, self.start_after, self.loop = float(speed), float(start_after), bool(loop)
        self.x, self.y = self.path[0]
        self.yaw, self.t, self._leg, self._dir = 0.0, 0.0, 0, 1

    def step(self, dt):
        self.t += dt
        if self.t < self.start_after or len(self.path) < 2:
            return
        left = self.speed * dt
        while left > 1e-9:
            tx, ty = self.path[self._leg + self._dir] if 0 <= self._leg + self._dir < len(self.path) else (None, None)
            if tx is None:
                return
            d = math.hypot(tx - self.x, ty - self.y)
            if d > 1e-9:
                self.yaw = math.atan2(ty - self.y, tx - self.x)
            if d <= left:
                self.x, self.y, left = tx, ty, left - d
                self._leg += self._dir
                nxt = self._leg + self._dir
                if not 0 <= nxt < len(self.path):
                    if not self.loop:
                        return
                    self._dir = -self._dir
            else:
                self.x, self.y, left = self.x + (tx - self.x) / d * left, self.y + (ty - self.y) / d * left, 0.0


def load_scenario(path):
    """(actors, overhead boxes) from a scenario YAML; an empty/None path gives none. Raises ValueError with a readable message."""
    if not path:
        return [], []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    actors = []
    for i, a in enumerate(data.get('actors') or []):
        try:
            actors.append(Actor(a.get('name', f'actor{i}'), a.get('id', i + 1), a['path'], a.get('speed', 0.3), a.get('radius', 0.22),
                                a.get('colour', ''), a.get('start_after', 0.0), a.get('loop', False), a.get('height', PERSON_HEIGHT)))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'{path}: actor #{i}: {e!r}') from None
    boxes = []
    for i, b in enumerate(data.get('overhead') or []):
        try:
            boxes.append({k: float(b[k]) for k in ('x', 'y', 'w', 'd', 'z0', 'z1')})
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f'{path}: overhead #{i}: {e!r}') from None
    return actors, boxes


def ray_circles(ox, oy, angles, circles, rmax):
    """Distance along each ray (world angles) to the nearest circle (cx, cy, r); np.inf where none within rmax."""
    out = np.full(len(angles), np.inf)
    dx, dy = np.cos(angles), np.sin(angles)
    for cx, cy, r in circles:
        fx, fy = cx - ox, cy - oy
        t = fx * dx + fy * dy                          # distance to the closest approach
        h2 = fx * fx + fy * fy - t * t                 # squared distance of the centre from the ray
        ok = (t > 0) & (h2 < r * r)
        hit = t - np.sqrt(np.maximum(r * r - h2, 0.0))
        out = np.where(ok & (hit < out) & (hit <= rmax), np.maximum(hit, 0.0), out)
    return out


def box_touches(x, y, radius, boxes, height=0.90):
    """True if a robot circle at (x, y) overlaps the footprint of an overhead box whose underside is below the robot's top (`height`)."""
    for b in boxes:
        if b['z0'] >= height:
            continue
        nx = min(max(x, b['x'] - b['w'] / 2), b['x'] + b['w'] / 2)
        ny = min(max(y, b['y'] - b['d'] / 2), b['y'] + b['d'] / 2)
        if math.hypot(x - nx, y - ny) < radius:
            return True
    return False


def camera_pose(rx, ry, ryaw, cam=CAMERA):
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + cam['x'] * c - cam['y'] * s, ry + cam['x'] * s + cam['y'] * c, ryaw


def detections(rx, ry, ryaw, actors, omap, cam=CAMERA, max_range=6.0, min_range=0.4):
    """What a person detector with depth would report: a list of dicts {track_id, colour, x_min, y_min, x_max, y_max, distance_m}
    (pixels, 640x480 by default). A person is dropped when outside the field of view, nearer than min_range or farther than max_range,
    behind a wall, or hidden behind a nearer person. Boxes are clipped to the image like a real detector's."""
    cx, cy, cyaw = camera_pose(rx, ry, ryaw, cam)
    W, H = cam['width'], cam['height']
    fx = (W / 2.0) / math.tan(math.radians(cam['hfov_deg']) / 2.0)
    seen = []
    for a in actors:
        dx, dy = a.x - cx, a.y - cy
        fwd, left = dx * math.cos(cyaw) + dy * math.sin(cyaw), -dx * math.sin(cyaw) + dy * math.cos(cyaw)
        if not (min_range <= fwd <= max_range):
            continue
        u = W / 2.0 - fx * left / fwd                  # image x grows to the right, robot y to the left
        half = fx * a.radius / fwd
        if u + half < 0 or u - half > W:
            continue
        if omap is not None:
            wall = float(omap.raycast(cx, cy, 0.0, np.array([math.atan2(dy, dx)]), 0.05, math.hypot(dx, dy))[0])
            if math.isfinite(wall) and wall < math.hypot(dx, dy) - a.radius:
                continue
        top, bot = H / 2.0 - fx * (a.height - cam['z']) / fwd, H / 2.0 + fx * cam['z'] / fwd
        seen.append(dict(track_id=a.track_id, colour=a.colour, distance_m=float(fwd), _bearing=math.atan2(left, fwd), _r=a.radius,
                         x_min=float(np.clip(u - half, 0, W)), x_max=float(np.clip(u + half, 0, W)),
                         y_min=float(np.clip(top, 0, H)), y_max=float(np.clip(bot, 0, H))))
    seen.sort(key=lambda d: d['distance_m'])
    keep = []
    for d in seen:                                     # hidden = its centre line lies inside a nearer person's angular width
        if not any(abs(d['_bearing'] - k['_bearing']) < math.asin(min(1.0, k['_r'] / max(k['distance_m'], k['_r'] + 1e-6)))
                   for k in keep):
            keep.append(d)
    return [{k: v for k, v in d.items() if not k.startswith('_')} for d in keep]


def overhead_cloud(rx, ry, ryaw, boxes, cam=CAMERA, vfov_deg=42.0, max_range=3.0, min_range=0.3, step=0.05, floor=True):
    """Depth points (N x 3, camera_link axes: x forward, y left, z up, metres) of the box surfaces and, with `floor`, of the floor,
    inside the camera's field of view and range. Meant for the sim's /depth_cloud (frame camera_link)."""
    cx, cy, cyaw = camera_pose(rx, ry, ryaw, cam)
    pts = []
    for b in boxes:
        xs = np.arange(b['x'] - b['w'] / 2, b['x'] + b['w'] / 2 + 1e-9, step)
        ys = np.arange(b['y'] - b['d'] / 2, b['y'] + b['d'] / 2 + 1e-9, step)
        zs = np.arange(b['z0'], b['z1'] + 1e-9, step)
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        edge = np.zeros(X.shape, bool)
        edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
        for z in zs:                                   # side walls at each height, plus the top and bottom faces
            m = edge if z not in (zs[0], zs[-1]) else np.ones(X.shape, bool)
            pts.append(np.column_stack([X[m], Y[m], np.full(m.sum(), z)]))
    if floor:
        r = np.arange(min_range, max_range, 0.1)
        a = np.radians(np.arange(-cam['hfov_deg'] / 2, cam['hfov_deg'] / 2, 3.0))
        R, A = np.meshgrid(r, a)
        pts.append(np.column_stack([cx + (R * np.cos(A + cyaw)).ravel(), cy + (R * np.sin(A + cyaw)).ravel(), np.zeros(R.size)]))
    if not pts:
        return np.zeros((0, 3), np.float32)
    w = np.vstack(pts)
    dx, dy, dz = w[:, 0] - cx, w[:, 1] - cy, w[:, 2] - cam['z']
    fwd, left = dx * math.cos(cyaw) + dy * math.sin(cyaw), -dx * math.sin(cyaw) + dy * math.cos(cyaw)
    ok = ((fwd >= min_range) & (np.hypot(fwd, left) <= max_range) & (fwd > 1e-6)
          & (np.abs(np.arctan2(left, fwd)) <= math.radians(cam['hfov_deg'] / 2))
          & (np.abs(np.arctan2(dz, fwd)) <= math.radians(vfov_deg / 2)))
    return np.column_stack([fwd[ok], left[ok], dz[ok]]).astype(np.float32)
