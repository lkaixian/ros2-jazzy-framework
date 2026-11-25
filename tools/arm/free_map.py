"""Collision-free reachable region of the arm, IDEAL CASE: the union of the space swept by the WHOLE arm (link skeleton + a margin) while it
was moved by hand with torque off, without touching anything. It says where the arm fits, NOT where it may safely be driven: gravity load,
motion, cable pull and overload are not considered (see torque.py). Everything else is unknown, not "a wall". Cumulative across runs.

Frame = arm base (omx_fk): x forward, y left, z up, metres. Pure maths, no hardware.
"""
import itertools

import numpy as np

LINKS = ('link2', 'link3', 'link4', 'link5', 'end_effector_link')
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
# ASCII views: (value at first row/col, value at last, count). Rows are 4 cm and columns 2 cm because characters are twice as tall as wide.
VIEWS = (('TOP view (front up, left on the left)', lambda p: (p[0], p[1]), (0.40, -0.16, 14), (0.40, -0.40, 40)),
         ('SIDE view (front on the right)', lambda p: (p[2], p[0]), (0.45, -0.30, 19), (-0.16, 0.40, 28)))


def skeleton(arm, q, per=8):
    """Points along base -> shoulder -> elbow -> wrist -> gripper for joint angles q (arm base frame), shape (n, 3)."""
    joints = [np.zeros(3)] + [arm.xyz(link, q) for link in LINKS]
    return np.array([a + (b - a) * t for a, b in zip(joints, joints[1:]) for t in np.linspace(0, 1, per)])


class FreeMap:
    def __init__(self, voxel=0.02, margin=1):
        """margin: voxels added around the skeleton (arm thickness + OAK), 1 voxel = 2 cm by default."""
        self.v, self.m = voxel, margin
        self.core, self.cells, self.lo, self.hi = set(), set(), {}, {}
        self.n = 0
        self._offsets = list(itertools.product(range(-margin, margin + 1), repeat=3))

    def add(self, arm, q):
        """Add one sample; returns how many new voxels it opened."""
        before = len(self.cells)
        for c in map(tuple, np.floor(skeleton(arm, q) / self.v).astype(int)):
            if c not in self.core:
                self.core.add(c)
                self.cells.update((c[0] + a, c[1] + b, c[2] + d) for a, b, d in self._offsets)
        for n in JOINTS:
            self.lo[n], self.hi[n] = min(self.lo.get(n, q[n]), q[n]), max(self.hi.get(n, q[n]), q[n])
        self.n += 1
        return len(self.cells) - before

    def stats(self):
        c = np.array(list(self.cells)) if self.cells else np.zeros((0, 3), int)
        cm2 = (self.v * 100) ** 2
        return {'volume_l': len(c) * self.v ** 3 * 1000, 'top_cm2': len({(a, b) for a, b, _ in c.tolist()}) * cm2,
                'side_cm2': len({(a, d) for a, _, d in c.tolist()}) * cm2, 'voxels': len(c), 'samples': self.n,
                'extent': None if not len(c) else {k: (c[:, i].min() * self.v, (c[:, i].max() + 1) * self.v)
                                                   for i, k in enumerate('xyz')}}

    def save(self, path):
        np.savez_compressed(path, core=np.array(sorted(self.core), dtype=np.int32).reshape(-1, 3), voxel=self.v, margin=self.m,
                            n=self.n, lo=[self.lo.get(j, 0.0) for j in JOINTS], hi=[self.hi.get(j, 0.0) for j in JOINTS])

    @classmethod
    def load(cls, path):
        d = np.load(path)
        m = cls(float(d['voxel']), int(d['margin']))
        for c in map(tuple, d['core'].tolist()):
            m.core.add(c)
            m.cells.update((c[0] + a, c[1] + b, c[2] + e) for a, b, e in m._offsets)
        m.n = int(d['n'])
        if m.n:
            m.lo, m.hi = dict(zip(JOINTS, map(float, d['lo']))), dict(zip(JOINTS, map(float, d['hi'])))
        return m

    def render(self, cur=None):
        """Top and side ASCII views side by side, as lines. '#' swept, '.' unknown, '@' the gripper now, '+' the arm base."""
        centres = (np.array(list(self.cells)) + 0.5) * self.v if self.cells else np.zeros((0, 3))
        panels = []
        for title, pick, (r0, r1, nr), (c0, c1, nc) in VIEWS:
            grid = [['.'] * nc for _ in range(nr)]
            for pts, ch in ((centres, '#'), ([(0.0, 0.0, 0.0)], '+'), ([] if cur is None else [cur], '@')):
                for p in pts:
                    rv, cv = pick(p)
                    r, c = int((rv - r0) / ((r1 - r0) / nr)), int((cv - c0) / ((c1 - c0) / nc))
                    if 0 <= r < nr and 0 <= c < nc:
                        grid[r][c] = ch
            panels.append([title.ljust(nc)] + [''.join(row) for row in grid])
        n = max(len(x) for x in panels)
        return [(panels[0][i] if i < len(panels[0]) else ' ' * 40) + '   ' + (panels[1][i] if i < len(panels[1]) else '')
                for i in range(n)]
