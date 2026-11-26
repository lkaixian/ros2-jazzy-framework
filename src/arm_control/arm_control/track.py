"""One pinned pick target (id 1) as a constant-velocity Kalman filter on a point in the arm base frame, plus where to look when it is lost.

No ROS imports. The measurements are already positions in the arm frame (the pixel -> point conversion, which is the non-linear part, happens
before), so the filter is linear: a plain Kalman filter, not an extended one. It does three jobs:
  * average the 2 cm depth noise over several looks;
  * keep the same object when two of the same label are in view (nearest to the prediction, inside a gate);
  * coast through a few missed looks on the predicted position, and say where to point the camera when it stays lost.
"""
import math

import numpy as np

CHI_GATE = 3.5          # Mahalanobis distance of the innovation (3 dof) beyond which a detection is another object
GATE_FLOOR_M = 0.04     # ...but never gate tighter than this: a settled filter must not reject normal jitter


class Track:
    def __init__(self, p, t, r_sd=0.02, accel_sd=0.3, pos_sd=0.03, vel_sd=0.5, ident=1):
        self.id, self.t = ident, t
        self.x = np.concatenate([np.asarray(p, float), np.zeros(3)])
        self.P = np.diag([pos_sd ** 2] * 3 + [vel_sd ** 2] * 3)
        self.R, self.q = np.eye(3) * r_sd ** 2, accel_sd ** 2
        self.seen, self.missed = 1, 0

    @property
    def pos(self):
        return self.x[:3].copy()

    @property
    def vel(self):
        return self.x[3:].copy()

    @property
    def sigma(self):
        """1-sigma position uncertainty (m)."""
        return float(math.sqrt(np.trace(self.P[:3, :3]) / 3.0))

    def predict(self, t):
        dt = max(0.0, t - self.t)
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * self.q * dt ** 4 / 4
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * self.q * dt ** 3 / 2
        Q[3:, 3:] = np.eye(3) * self.q * dt ** 2
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + Q, t
        return self.pos

    def _gate(self, z):
        d = np.asarray(z, float) - self.x[:3]
        S = self.P[:3, :3] + self.R
        return float(d @ np.linalg.solve(S, d)) ** 0.5 <= CHI_GATE or float(np.linalg.norm(d)) <= GATE_FLOOR_M

    def step(self, t, points):
        """One look at time t with the detections of this label (arm-frame points). Returns 'seen' or 'coasting'."""
        self.predict(t)
        near = min(points, key=lambda z: float(np.linalg.norm(np.asarray(z, float) - self.x[:3])), default=None)
        if near is None or not self._gate(near):
            self.missed += 1
            return 'coasting'
        z = np.asarray(near, float)
        S = self.P[:3, :3] + self.R
        K = self.P[:, :3] @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x[:3])
        self.P = (np.eye(6) - np.hstack([K, np.zeros((6, 3))])) @ self.P
        self.seen, self.missed = self.seen + 1, 0
        return 'seen'


def scan_yaws(pred, ws_min, ws_max, yaw_lo, yaw_hi, fov, n=5):
    """Joint-1 angles to try, nearest the predicted position first, then alternating either side by one camera field of view.

    The prediction is clipped into the workable box first (it can only be found where the arm can work), and every yaw stays inside the
    taught range. [] when the box holds no bearing the arm can look along.
    """
    p = np.clip(np.asarray(pred, float), ws_min, ws_max)
    if math.hypot(p[0], p[1]) < 0.05:
        return []
    b = math.atan2(p[1], p[0])
    out = []
    for k in range(n):
        for s in ((0,) if k == 0 else (1, -1)):
            y = b + s * k * fov
            if yaw_lo <= y <= yaw_hi and not any(abs(y - o) < 1e-6 for o in out):
                out.append(y)
    return out
