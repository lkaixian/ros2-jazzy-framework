"""Pure logic for the navigation BT nodes (no ROS imports): places, geometry, map names, results."""
import math
import os
import re

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,60}$')
GOAL_STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}   # action_msgs/GoalStatus


class PlacesError(ValueError):
    pass


def valid_name(name):
    return bool(NAME_RE.match(name))


def yaw_to_quat(yaw):
    """Planar rotation -> (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def bearing(frm, to):
    """Heading (rad) of the straight line frm -> to, both (x, y)."""
    return math.atan2(to[1] - frm[1], to[0] - frm[0])


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def goal_yaw(current, target, yaw=None, min_travel=0.05):
    """Final heading for a goal. Given -> as is. Omitted -> the direction of travel from the
    current pose (the robot ends facing where it was heading); keeps the current heading if the
    goal is practically where the robot already is; 0 if the pose is unknown.

    current: (x, y, yaw) or None; target: (x, y).
    """
    if yaw is not None:
        return yaw
    if current is None:
        return 0.0
    if distance(current, target) < min_travel:
        return current[2]
    return bearing(current, target)


def load_places(path):
    """{name: (x, y, yaw)} from a places YAML; raises PlacesError with a readable message."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        raise PlacesError(f'cannot read places file {path}: {e}') from None
    raw = data.get('places')
    if not isinstance(raw, dict):
        raise PlacesError(f"{path}: expected a top-level 'places:' mapping")
    out = {}
    for name, p in raw.items():
        if not isinstance(name, str) or not valid_name(name):
            raise PlacesError(f'place name {name!r}: use 1-60 of A-Z a-z 0-9 _ -')
        try:
            x, y, yaw = float(p['x']), float(p['y']), float(p.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            raise PlacesError(f"place {name!r}: needs numeric x and y (yaw optional)") from None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise PlacesError(f'place {name!r}: coordinates must be finite')
        out[name] = (x, y, yaw)
    return out


def places_listing(places):
    return ', '.join(sorted(places)) if places else '(none configured)'


def map_base_path(map_dir, file):
    """<map_dir>/<file> for the map saver (no extension); `file` must be a plain name so a plan
    cannot write outside map_dir."""
    if not valid_name(file):
        raise ValueError(f'invalid map file name {file!r}: use 1-60 of A-Z a-z 0-9 _ -')
    return os.path.join(os.path.expanduser(map_dir), file)


def nav_result(status, error_code=0, error_msg=''):
    """(ok, message) for a finished NavigateToPose goal."""
    name = GOAL_STATUS.get(status, f'status {status}')
    if status == 4:
        return True, 'arrived'
    if status == 5:
        return False, 'cancelled'
    detail = error_msg or (f'error code {error_code}' if error_code else 'no reason given')
    return False, f'navigation {name.lower()}: {detail}'
