#!/usr/bin/env python3
"""Camera height, pitch and roll from the floor in a depth image: the z / pitch / roll of base_to_camera in param/navigation.yaml.

  source env.sh; ros2 launch framework_bringup sensors.launch.py   # RealSense up, robot on flat floor, floor in view
  python3 tools/camera/floor_plane.py [--depth TOPIC] [--info TOPIC] [--frames 10]

Prints the median of N frames. Gives nothing for x, y or yaw (a plane cannot): use a tape and the wall test against /scan.
"""
import argparse
import math
import sys

import numpy as np


def fit_floor(pts, iters=300, tol=0.02, seed=0):
    """Plane n.p = d through the points (N x 3, camera_link axes: x forward, y left, z up). -> (n, height, inlier fraction).
    RANSAC, then a least-squares refit on the inliers. n points up, so the floor is at n.p = -height."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs(pts @ n - n @ a) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        raise ValueError('no plane found')
    q = pts[best]
    m = q.mean(axis=0)
    n = np.linalg.svd(q - m)[2][-1]
    if n[2] < 0:
        n = -n
    return n, -float(n @ m), float(best.mean())


def pitch_roll(n):
    """Camera pitch (nose down +) and roll from the floor normal in camera_link axes: n = (-sin p, cos p sin r, cos p cos r)."""
    return math.asin(max(-1.0, min(1.0, -n[0]))), math.atan2(n[1], n[2])


def depth_points(depth, scale, k, step=8, lo=0.3, hi=4.0):
    """Lower half of the depth image -> camera_link points (optical x right, y down, z forward -> x fwd, y left, z up)."""
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    v, u = np.mgrid[int(cy) + 1:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[v, u].astype(np.float64) * scale
    ok = np.isfinite(z) & (z >= lo) & (z <= hi)
    u, v, z = u[ok], v[ok], z[ok]
    return np.stack([z, -(u - cx) * z / fx, -(v - cy) * z / fy], axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--depth', default='/camera/camera/depth/image_rect_raw')
    ap.add_argument('--info', default='/camera/camera/depth/camera_info')
    ap.add_argument('--frames', type=int, default=10)
    a = ap.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from person_detection.depth_ranging import depth_from_image
    rclpy.init()
    node = rclpy.create_node('floor_plane')
    k, rows = [], []
    node.create_subscription(CameraInfo, a.info, lambda m: k.append(list(m.k)) if not k else None, qos_profile_sensor_data)

    def on_depth(m):
        if not k or len(rows) >= a.frames:
            return
        arr, scale = depth_from_image(m.encoding, m.height, m.width, m.step, m.data, bool(m.is_bigendian))
        try:
            n, height, frac = fit_floor(depth_points(arr, scale, k[0]))
        except ValueError:
            return
        rows.append((height, *pitch_roll(n), frac))
    node.create_subscription(Image, a.depth, on_depth, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + 15e9
    while rclpy.ok() and len(rows) < a.frames and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.try_shutdown()
    if not rows:
        sys.exit(f'no usable frames on {a.depth} / {a.info} in 15 s (camera running? floor in the lower half of the view?)')
    h, p, r, f = np.array(rows).T
    low = '   LOW: is the floor in view and flat?' if np.median(f) < 0.4 else ''
    print(f'{len(rows)} frames   floor inliers {np.median(f):.0%}{low}')
    print(f'z (height above floor)  {np.median(h):.3f} m   spread {np.ptp(h) * 1000:.1f} mm')
    print(f'pitch (nose down +)     {np.median(p):+.4f} rad = {math.degrees(np.median(p)):+.1f} deg')
    print(f'roll                    {np.median(r):+.4f} rad = {math.degrees(np.median(r)):+.1f} deg')
    print('-> base_to_camera z/pitch/roll; assumes base z=0 is the floor and the depth sensor sits at camera_link')


if __name__ == '__main__':
    main()
