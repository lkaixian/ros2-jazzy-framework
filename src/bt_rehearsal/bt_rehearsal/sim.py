"""Simulator interface and a ROS-free 2D kinematic implementation over nav_sim's occupancy map.

`KinematicNavSim` interprets a small BT subset (Sequence/Fallback/Retry/Repeat/Parallel/Sleep and the
navigation nodes) and checks reachability on the inflated grid. It does NOT run BehaviorTree.CPP or
Nav2: a plan that passes here is sim-verified at the kinematic level only. Nodes it cannot model fail
loudly (`unsupported node`) rather than pass silently.
"""
import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from typing import Optional, Protocol

from bt_rehearsal.isolation import Observable, Privileged

CONTROL = {'Sequence', 'ReactiveSequence', 'Parallel'}
FALLBACK = {'Fallback', 'ReactiveFallback'}
INSTANT_OK = {'Sleep', 'Speak', 'AlwaysSuccess', 'Say'}   # no physical effect to model: succeed
MAX_STEPS = 500


@dataclass(frozen=True)
class Task:
    text: str
    goal_place: Optional[str] = None     # scoring only: where the robot must end up


class Simulator(Protocol):
    def reset(self) -> None: ...
    def scene(self) -> dict: ...                      # perception-style summary (deployable)
    def run(self, xml: str) -> tuple: ...             # -> (Observable, Privileged)
    def score(self, task: Task, obs: Observable, priv: Privileged) -> bool: ...


class KinematicNavSim:
    """places: {name: (x, y)} in the map frame; scene_summary: what perception would report."""

    def __init__(self, occ, places, start=(0.5, 0.5, 0.0), radius=0.17, speed=0.3,
                 scene_summary=None, goal_tol=0.3):
        self.occ, self.places, self.start = occ, dict(places), start
        self.radius, self.speed, self.goal_tol = radius, speed, goal_tol
        self._scene = scene_summary or {'objects': [], 'num_persons': 0}
        self.reset()

    def reset(self):
        self.pose = list(self.start)
        self.steps = 0
        self.travelled = 0.0

    def scene(self):
        return dict(self._scene)

    # ---- grid reachability ----
    def _cell(self, x, y):
        r, c = self.occ.to_cell(x, y)
        return int(r), int(c)

    def _path_len(self, goal):
        """BFS over cells whose robot footprint is free; returns metres or None."""
        (r0, c0), (r1, c1) = self._cell(*self.pose[:2]), self._cell(*goal)
        if self.occ.collides(goal[0], goal[1], self.radius):
            return None
        step = max(1, int(round(self.radius / self.occ.res)))
        # coarse grid (cell = radius) keeps the BFS small; fine enough for room-scale maps

        def key(r, c):
            return r // step, c // step
        seen, q = {key(r0, c0)}, deque([(r0, c0, 0.0)])
        while q:
            r, c, d = q.popleft()
            if key(r, c) == key(r1, c1):
                return d * self.occ.res
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr * step, c + dc * step
                k = key(nr, nc)
                if k in seen or not self.occ.inside(nr, nc):
                    continue
                x = self.occ.ox + (nc + 0.5) * self.occ.res
                y = self.occ.oy + (self.occ.h - 1 - nr + 0.5) * self.occ.res
                if self.occ.collides(x, y, self.radius):
                    continue
                seen.add(k)
                q.append((nr, nc, d + step))
        return None

    # ---- BT interpretation: each node returns (ok, failed_node, message) ----
    def _nav(self, tag, a):
        if tag == 'NavigateToPlace':
            name = a.get('place', '')
            if name not in self.places:
                return False, tag, f"unknown place '{name}'"
            goal = self.places[name]
        else:
            try:
                goal = (float(a['x']), float(a['y']))
            except (KeyError, ValueError):
                return False, tag, 'bad pose'
        d = self._path_len(goal)
        if d is None:
            return False, tag, 'no path to goal'
        limit = a.get('timeout_ms')
        if limit and d / self.speed * 1000 > float(limit):
            return False, tag, 'timeout'
        self.pose[0], self.pose[1] = goal
        self.travelled += d
        return True, '', ''

    def _tick(self, el):
        self.steps += 1
        if self.steps > MAX_STEPS:
            return False, el.tag, 'step limit'
        t, kids = el.tag, list(el)
        if t == 'BehaviorTree':
            return self._tick(kids[0])
        if t in CONTROL:
            for k in kids:
                ok, n, m = self._tick(k)
                if not ok:
                    return False, n, m
            return True, '', ''
        if t in FALLBACK:
            last = (False, t, 'no child succeeded')
            for k in kids:
                ok, n, m = self._tick(k)
                if ok:
                    return True, '', ''
                last = (False, n, m)
            return last
        if t == 'RetryUntilSuccessful':
            last = (False, t, '')
            for _ in range(int(el.get('num_attempts', 1))):
                last = self._tick(kids[0])
                if last[0]:
                    return last
            return last
        if t == 'Repeat':
            for _ in range(int(el.get('num_cycles', 1))):
                r = self._tick(kids[0])
                if not r[0]:
                    return r
            return True, '', ''
        if t in ('NavigateToPlace', 'NavigateToPose'):
            return self._nav(t, el.attrib)
        if t in INSTANT_OK:
            return True, '', ''
        return False, t, 'unsupported node (not modelled by the kinematic sim)'

    def run(self, xml):
        self.reset()
        root = ET.fromstring(xml)
        main = root.get('main_tree_to_execute')
        tree = next(t for t in root if t.tag == 'BehaviorTree' and t.get('ID') == main)
        subs = {t.get('ID'): t for t in root if t.tag == 'BehaviorTree'}
        for sub in tree.iter('SubTree'):          # kinematic sim does not model sub-trees
            if sub.get('ID') in subs:
                ok, n, m = False, 'SubTree', 'sub-trees are not modelled by the kinematic sim'
                break
        else:
            ok, n, m = self._tick(tree)
        obs = Observable(self.scene(), (), {'result': 'SUCCESS' if ok else 'FAILURE',
                                            'failed_node': n, 'message': m, 'steps': self.steps})
        return obs, Privileged(tuple(self.pose), 0, {'travelled': self.travelled})

    def score(self, task, obs, priv):
        """The only place ground truth is read."""
        if obs.outcome.get('result') != 'SUCCESS':
            return False
        if task.goal_place is None:
            return True
        gx, gy = self.places[task.goal_place]
        return math.hypot(priv.pose[0] - gx, priv.pose[1] - gy) <= self.goal_tol
