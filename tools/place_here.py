#!/usr/bin/env python3
"""Save where the robot stands now as a named place: python3 tools/place_here.py door   (needs the navigation stack: map->base transform).
Writes/replaces the line in src/framework_bringup/param/places.yaml (comments and other lines stay) and prints it."""
import math
import os
import re
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

PLACES = __file__.rsplit('/tools/', 1)[0] + '/src/framework_bringup/param/places.yaml'


def main(name):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        sys.exit('name: letters, digits, _ and - only')
    rclpy.init()
    node = rclpy.create_node('place_here')
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)    # the listener must spin on its own: nothing here spins the node
    try:
        t = buf.lookup_transform('map', 'base', Time(), Duration(seconds=5.0))
    except Exception as e:  # noqa: BLE001
        sys.exit(f'no map->base transform ({e}): is tools/map_session.sh --nav running and the initial pose right?')
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    line = f'  {name}: {{x: {t.transform.translation.x:.2f}, y: {t.transform.translation.y:.2f}, yaw: {yaw:.2f}}}'
    text = open(PLACES).read()
    pat = re.compile(rf'^  {re.escape(name)}:.*$', re.M)
    text = pat.sub(line, text, count=1) if pat.search(text) else text.rstrip('\n') + '\n' + line + '\n'
    open(PLACES, 'w').write(text)
    print(line.strip(), '->', PLACES, flush=True)
    os._exit(0)     # the listener's spin thread would keep the process alive


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) == 2 else sys.exit(__doc__))
