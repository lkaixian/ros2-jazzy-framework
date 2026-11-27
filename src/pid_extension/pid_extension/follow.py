"""Person-following logic for FollowTarget, with no ROS imports: who to follow, where they were, and what to do when they vanish.

Odom-frame poses are (x, y, yaw). Bearing is positive to the robot's left (the camera sees a person on its right at x > centre).
shortcut: reactive pursuit, no map or path planning: a person who walked round a corner is looked for at the place they were last
seen (drive there, then sweep), not routed to. Upgrade to Nav2 goals at the last-seen point only if sim and real runs show it failing.
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def bearing_from_image(ex, hfov_deg):
    """Bearing (rad, + left) of a pixel at normalised horizontal offset ex (-1 left edge .. +1 right edge of the image)."""
    return -math.atan(ex * math.tan(math.radians(hfov_deg) / 2.0))


class Memory:
    """Where the followed person was last seen, in the odom frame."""

    def __init__(self):
        self.track_id = None
        self.colour = ''
        self.pos = None            # (x, y) metres, only when a depth range was available
        self.world_bearing = None  # heading from the robot to them when last seen, rad (always known once seen)
        self.rel_bearing = 0.0     # the same relative to the robot's heading then: which side they left on
        self.seen_t = None

    def update(self, person, pose, now, hfov_deg, width):
        ex = ((person.x_min + person.x_max) / 2.0 - width / 2.0) / (width / 2.0)
        b = bearing_from_image(ex, hfov_deg)
        self.track_id = person.track_id if person.track_id >= 0 else self.track_id
        self.colour = person.clothes_colour or self.colour
        self.rel_bearing, self.world_bearing, self.seen_t = b, wrap(pose[2] + b), now
        if person.distance_m > 0:
            self.pos = (pose[0] + person.distance_m * math.cos(self.world_bearing),
                        pose[1] + person.distance_m * math.sin(self.world_bearing))
        return ex, b


def select(persons, mem, pose, hfov_deg, width, tid=None, lock=False, reid_radius=1.2, reid_bearing=0.5):
    """The person to follow, or None.

    tid given or locked (mem.track_id): that track id. If it is absent (the tracker gave the same person a new id after an
    occlusion) take the one that fits where they were: the same clothes colour when both are known, within reid_radius of the
    last depth position, or within reid_bearing of the last bearing when there was no depth. Otherwise the biggest box.
    A different clothes colour is never adopted; two equally good candidates pick the nearer to the last position.
    """
    want = tid if tid is not None else (mem.track_id if lock else None)
    if want is None:
        return max(persons, key=lambda q: (q.y_max - q.y_min) * (q.x_max - q.x_min), default=None)
    for q in persons:
        if q.track_id == want:
            return q
    best, best_d = None, None
    for q in persons:
        if q.track_id < 0 or (q.clothes_colour and mem.colour and q.clothes_colour != mem.colour):
            continue
        ex = ((q.x_min + q.x_max) / 2.0 - width / 2.0) / (width / 2.0)
        wb = wrap(pose[2] + bearing_from_image(ex, hfov_deg))
        if mem.pos is not None and q.distance_m > 0:
            d = math.hypot(pose[0] + q.distance_m * math.cos(wb) - mem.pos[0], pose[1] + q.distance_m * math.sin(wb) - mem.pos[1])
            ok = d <= reid_radius
        elif mem.world_bearing is not None:
            d = abs(wrap(wb - mem.world_bearing))
            ok = d <= reid_bearing
        else:
            continue
        if ok and (best_d is None or d < best_d):
            best, best_d = q, d
    return best


class LostPolicy:
    """What to do once the person is gone: drive to where they were last seen, then sweep toward the side they left on, then give up.

    step(...) -> (v, w, phase) with phase 'goto' | 'search' | 'wait' | 'fail'. `wait` (stand still) is used when the odom pose is
    unknown or search is off: it fails after wait_sec.
    """

    def __init__(self, goto_sec=6.0, search_sec=12.0, sweep_rad=1.6, wait_sec=1.0, arrive_m=0.6, v=0.15, w=0.5):
        self.goto_sec, self.search_sec, self.sweep_rad, self.wait_sec = goto_sec, search_sec, sweep_rad, wait_sec
        self.arrive_m, self.v, self.w = arrive_m, v, w
        self.reset()

    def reset(self):
        self.t0 = None             # when the person was lost
        self.search_t0 = None
        self.targets = None        # absolute yaw targets of the sweep, odom frame
        self.leg = 0

    def step(self, mem, pose, now, can_drive=True, can_turn=True):
        """can_drive / can_turn: False when the lidar says there is no room to go forward / turn (then that motion is skipped)."""
        if pose is None or mem.world_bearing is None:
            self.t0 = self.t0 or now
            return 0.0, 0.0, ('wait' if now - self.t0 <= self.wait_sec else 'fail')
        self.t0 = self.t0 or now
        t = now - self.t0
        if mem.pos is not None and t < self.goto_sec:
            d = math.hypot(mem.pos[0] - pose[0], mem.pos[1] - pose[1])
            if d > self.arrive_m:
                err = wrap(math.atan2(mem.pos[1] - pose[1], mem.pos[0] - pose[0]) - pose[2])
                w = max(-self.w, min(self.w, 1.5 * err))
                return (self.v if abs(err) < 0.5 and can_drive else 0.0), w, 'goto'
        if self.search_t0 is None:
            self.search_t0 = now
            side = 1.0 if mem.rel_bearing >= 0 else -1.0          # they left to the left -> look left first
            self.targets = [wrap(pose[2] + side * self.sweep_rad), wrap(pose[2] - side * self.sweep_rad)]   # the second leg turns on through the rear
        if now - self.search_t0 > self.search_sec or self.leg >= len(self.targets):
            return 0.0, 0.0, 'fail'
        err = wrap(self.targets[self.leg] - pose[2])
        if abs(err) < 0.1:
            self.leg += 1
            return 0.0, 0.0, 'search'
        return 0.0, (math.copysign(self.w, err) if can_turn else 0.0), 'search'
