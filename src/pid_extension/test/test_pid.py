import pytest

from pid_extension.pid import PID


def test_proportional_and_clamp():
    p = PID(2.0, out_limit=1.0)
    assert p.update(0.2, 0.0, 0.1) == 0.4
    assert p.update(5.0, 0.0, 0.1) == 1.0 and p.update(-5.0, 0.0, 0.1) == -1.0


def test_integral_antiwindup():
    p = PID(0.0, ki=1.0, out_limit=0.5)
    for _ in range(1000):
        p.update(10.0, 0.0, 0.1)
    assert p.i <= 0.5 + 1e-9
    assert p.update(-10.0, 0.0, 0.1) < 0.5         # recovers immediately, no huge wind-up


def test_derivative_on_measurement_no_setpoint_kick():
    p = PID(0.0, kd=1.0, out_limit=100.0)
    p.update(0.0, 0.0, 0.1)
    assert p.update(50.0, 0.0, 0.1) == 0.0         # setpoint jump does not kick D


def test_derivative_damps_motion():
    p = PID(0.0, kd=1.0, out_limit=100.0)
    p.update(0.0, 0.0, 0.1)
    assert p.update(0.0, 0.1, 0.1) < 0             # measurement rising -> negative output


def test_explicit_error_for_angle_wrap():
    assert PID(1.0).update(0, 0, 0.1, err=-0.3) == -0.3


def test_closed_loop_first_order_plant_converges():
    p, x = PID(2.0, ki=0.2, kd=0.0, out_limit=5.0), 0.0
    for _ in range(400):
        x += p.update(1.0, x, 0.05) * 0.05
    assert abs(x - 1.0) < 0.02


def test_set_gains_resets_integral():
    p = PID(1.0, ki=1.0)
    p.update(1, 0, 1.0)
    p.set_gains(kp=2.0)
    assert p.i == 0.0 and p.kp == 2.0


def test_range_speed_forward_only_clamped_and_never_inside_target():
    from pid_extension.pid import MIN_STANDOFF_M, range_speed
    assert MIN_STANDOFF_M >= 0.5
    assert range_speed(3.0, 1.0, 0.5, 0.15) == 0.15          # far: clamped to v_max
    assert range_speed(1.1, 1.0, 0.5, 0.15) == pytest.approx(0.05)
    assert range_speed(1.0, 1.0, 0.5, 0.15) == 0.0           # at target: stop
    assert range_speed(0.4, 1.0, 0.5, 0.15) == 0.0           # too close: never reverses
