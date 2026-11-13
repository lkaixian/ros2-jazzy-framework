"""Generate a clean test map: two rooms joined by a door, with a few box obstacles.

  ros2 run nav_sim make_test_map --out-dir src/framework_bringup/maps
Writes test_room.pgm + test_room.yaml (12 x 8 m, 0.05 m/cell, origin (-1, -1)).
"""
import argparse
import os

import cv2
import numpy as np

RES = 0.05
ORIGIN = (-1.0, -1.0)
W_M, H_M = 12.0, 8.0


def make():
    """Return an HxW uint8 image: 254 free, 0 occupied (row 0 = top = max y)."""
    w, h = int(W_M / RES), int(H_M / RES)
    img = np.full((h, w), 254, dtype=np.uint8)

    def rect(x0, y0, x1, y1):            # world metres, y up
        c0, c1 = int((x0 - ORIGIN[0]) / RES), int((x1 - ORIGIN[0]) / RES)
        r0, r1 = h - int((y1 - ORIGIN[1]) / RES), h - int((y0 - ORIGIN[1]) / RES)
        img[max(r0, 0):r1, max(c0, 0):c1] = 0

    t = 0.1                                       # wall thickness
    rect(-1.0, -1.0, 11.0, -1.0 + t)              # outer walls
    rect(-1.0, 7.0 - t, 11.0, 7.0)
    rect(-1.0, -1.0, -1.0 + t, 7.0)
    rect(11.0 - t, -1.0, 11.0, 7.0)
    rect(5.0, -1.0, 5.0 + t, 2.2)                 # interior wall with a 1.4 m door at y 2.2..3.6
    rect(5.0, 3.6, 5.0 + t, 7.0)
    rect(1.5, 1.0, 2.1, 1.6)                      # boxes
    rect(2.5, 4.2, 3.1, 4.8)
    rect(7.5, 1.5, 8.1, 2.1)
    rect(8.5, 4.5, 9.1, 5.1)
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out-dir', default='.')
    a = ap.parse_args(argv)
    os.makedirs(a.out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(a.out_dir, 'test_room.pgm'), make())
    with open(os.path.join(a.out_dir, 'test_room.yaml'), 'w') as f:
        f.write(f"image: test_room.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n")
    print('wrote', os.path.join(a.out_dir, 'test_room.yaml'))


if __name__ == '__main__':
    main()
