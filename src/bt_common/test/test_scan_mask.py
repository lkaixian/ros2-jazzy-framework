import math

import numpy as np

from bt_common.scan_mask import mask_ranges

# the real mount: lidar 0.15 m forward of the base centre, facing backwards (yaw pi)
LX, LY, LYAW = 0.15, 0.0, math.pi
INC = math.radians(90)          # four beams: laser angles 0, 90, 180, 270 deg


def run(ranges, radius=0.27):
    return mask_ranges(ranges, 0.0, INC, LX, LY, LYAW, radius)


def test_beam_on_the_robot_body_is_masked_and_a_far_beam_is_kept():
    # laser angle 0 = robot REAR: a return 0.10 m away is at base x = 0.15 - 0.10 = 0.05 -> inside the body
    # laser angle 180 = robot FRONT: 0.50 m away is at base x = 0.65 -> an obstacle
    out, n = run([0.10, 3.0, 0.50, 3.0])
    assert n == 1 and np.isinf(out[0]) and out[2] == 0.5 and out[1] == 3.0 and out[3] == 3.0


def test_distance_is_measured_from_the_base_centre_not_from_the_lidar():
    # 0.30 m from the lidar but straight toward the rear: base x = 0.15 - 0.30 = -0.15 -> still inside the 0.27 m radius
    out, n = run([0.30, 9.0, 9.0, 9.0])
    assert n == 1 and np.isinf(out[0])
    # the same 0.30 m toward the FRONT is 0.45 m from the centre: kept
    out, n = run([9.0, 9.0, 0.30, 9.0])
    assert n == 0 and out[2] == 0.30


def test_nan_and_inf_pass_through_untouched():
    out, n = run([float('nan'), float('inf'), 0.05, 5.0])
    assert n == 1 and math.isnan(out[0]) and math.isinf(out[1]) and np.isinf(out[2]) and out[3] == 5.0
