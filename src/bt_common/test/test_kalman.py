import math

import numpy as np
import pytest

from bt_common.kalman import Kalman2D


def walk(kf, t0, t1, v=(0.8, 0.2), noise=0.05, seed=0, start=(0.0, 0.0)):
    rng = np.random.default_rng(seed)
    t = t0
    while t <= t1 + 1e-9:
        kf.update((start[0] + v[0] * t + rng.normal(0, noise), start[1] + v[1] * t + rng.normal(0, noise)), t, sigma=noise)
        t += 0.1
    return kf


def test_the_filter_learns_the_velocity_and_is_steadier_than_the_measurements():
    kf = walk(Kalman2D((0.0, 0.0), 0.0), 0.1, 6.0)
    assert kf.velocity[0] == pytest.approx(0.8, abs=0.15) and kf.velocity[1] == pytest.approx(0.2, abs=0.15)
    x, y = kf.position()
    assert math.hypot(x - 0.8 * 6.0, y - 0.2 * 6.0) < 0.12


def test_it_predicts_through_a_gap_and_the_uncertainty_grows_and_is_capped():
    kf = walk(Kalman2D((0.0, 0.0), 0.0), 0.1, 5.0)
    px, py = kf.position(6.5)                                          # 1.5 s after the last sighting
    assert math.hypot(px - 0.8 * 6.5, py - 0.2 * 6.5) < 0.35
    assert kf.sigma(6.5) > kf.sigma(5.0)
    assert kf.position(60.0) == kf.position(kf.seen + 2.0)             # never extrapolated beyond the horizon: held there


def test_a_measurement_that_cannot_be_the_same_person_is_rejected_and_a_long_gap_restarts_the_track():
    kf = walk(Kalman2D((0.0, 0.0), 0.0), 0.1, 4.0)
    assert kf.update((3.2 + 5.0, 0.8 + 4.0), 4.1, sigma=0.05) is False and kf.rejected == 1      # 6 m away 0.1 s later: another person
    assert kf.update((0.8 * 4.2, 0.2 * 4.2), 4.2, sigma=0.05) is True and kf.rejected == 0
    assert kf.update((10.0, 10.0), 9.0, sigma=0.05) is True                                       # 4.8 s without a sighting: start again there
    assert kf.position() == pytest.approx((10.0, 10.0), abs=1e-6)


def test_time_going_backwards_or_standing_still_does_not_break_it():
    kf = Kalman2D((1.0, 1.0), 5.0)
    kf.predict(4.0)
    kf.predict(5.0)
    assert kf.position() == (1.0, 1.0) and kf.update((1.0, 1.0), 5.0)
