"""A constant-velocity Kalman filter for a person walking on the floor (x, y in metres, any fixed frame). ROS-free.

Depth-ranged detections jump by a few centimetres and drop out when someone is hidden for a moment. The filter smooths the position, gives a velocity, throws away a
measurement that cannot be the same person, and predicts where they are for the next second or two so the follower can drive toward that instead of the last
sighting and the arm can still point at them.
shortcut: straight-line motion with random acceleration, one filter per person, no data association (the caller picks which detection belongs to it).
"""
import numpy as np


class Kalman2D:
    def __init__(self, pos, t, accel_sigma=0.8, pos_sigma=0.15, vel_sigma=0.8, gate=13.8, reinit_sec=3.0):
        self.x = np.array([pos[0], pos[1], 0.0, 0.0], float)
        self.P = np.diag([pos_sigma ** 2, pos_sigma ** 2, vel_sigma ** 2, vel_sigma ** 2])
        self.t, self.q, self.gate, self.reinit = float(t), accel_sigma ** 2, gate, reinit_sec
        self.seen = float(t)               # time of the last accepted measurement
        self.rejected = 0

    def _fq(self, dt):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        return F, self.q * g @ g.T

    def predict(self, t):
        """Move the state forward to time t (backwards is ignored)."""
        dt = float(t) - self.t
        if dt <= 0:
            return
        F, Q = self._fq(dt)
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + Q, float(t)

    def update(self, pos, t, sigma=0.1):
        """Fold in a measured position (x, y) with standard deviation `sigma` m. Returns True when it was accepted: a measurement whose Mahalanobis distance is beyond `gate`
        (chi-square, 2 dof: 13.8 = 99.9 %) is someone else or a glitch and is skipped; after `reinit_sec` without an accepted one the filter restarts at the new position."""
        z = np.asarray(pos, float)
        if float(t) - self.seen > self.reinit:
            self.__init__(z, t, np.sqrt(self.q), 0.15, 0.8, self.gate, self.reinit)
            return True
        self.predict(t)
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + np.eye(2) * sigma ** 2
        y = z - H @ self.x
        if float(y @ np.linalg.solve(S, y)) > self.gate:
            self.rejected += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.seen, self.rejected = float(t), 0
        return True

    def position(self, t=None, horizon=2.0):
        """Where the person is at time t (default: the filter's own time), extrapolated at most `horizon` s past the last accepted measurement, then held there."""
        t = self.t if t is None else min(float(t), self.seen + horizon)
        dt = t - self.t
        return (float(self.x[0] + self.x[2] * dt), float(self.x[1] + self.x[3] * dt))

    @property
    def velocity(self):
        return (float(self.x[2]), float(self.x[3]))

    def sigma(self, t=None):
        """One-sigma position uncertainty (m) at time t: it grows while nothing is measured."""
        t = self.t if t is None else float(t)
        F, Q = self._fq(max(0.0, t - self.t))
        P = F @ self.P @ F.T + Q
        return float(np.sqrt((P[0, 0] + P[1, 1]) / 2))
