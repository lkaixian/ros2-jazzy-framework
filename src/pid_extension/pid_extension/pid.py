"""Small PID with output clamp, integral anti-windup and derivative-on-measurement."""

MIN_STANDOFF_M = 0.5     # FollowTarget never approaches a person closer than this


def range_speed(distance_m, target_m, kp, v_max):
    """Forward speed to close the gap to a person: proportional, forward only, clamped.

    Never negative (the robot stops rather than backs into someone) and 0 at/inside target.
    """
    if distance_m <= target_m:
        return 0.0
    return min(v_max, kp * (distance_m - target_m))


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=1.0, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = out_limit if i_limit is None else i_limit
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_meas = None

    def set_gains(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        self.i = 0.0

    def update(self, setpoint, measurement, dt, err=None):
        """err may be given when the plain difference is wrong (e.g. angle wrap)."""
        e = (setpoint - measurement) if err is None else err
        self.i = max(-self.i_limit, min(self.i_limit, self.i + self.ki * e * dt))
        d = 0.0
        if self.prev_meas is not None and dt > 0:
            d = -(measurement - self.prev_meas) / dt   # derivative on measurement
        self.prev_meas = measurement
        out = self.kp * e + self.i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, out))
