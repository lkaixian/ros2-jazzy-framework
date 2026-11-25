#!/usr/bin/env python3
"""Navigation for the arm: plan a joint-space route through poses the arm has PHYSICALLY been in (hand-guided, torque off, no contact).

The taught streams (~/arm_stream*.csv, ~/arm_range*.csv) are a roadmap: consecutive taught poses and near-crossings between runs are
connected, and Dijkstra finds the shortest taught route between where the arm is now and where it should go (usually the rest pose).
A route is only ever made of motion that already happened, so it needs no body model. It cannot go anywhere that was not taught.

    python3 tools/arm/route.py --from 1.19,2.07,-1.38,-0.04 --to rest        # plan only, moves nothing
"""
import argparse
import csv
import glob
import heapq
import os
import sys

import numpy as np
from free_map import JOINTS

SPACING = 0.04     # rad: a taught pose becomes a node only when it is this far from the previous node of its stream
LINK = 0.10        # rad: nodes closer than this are connected (consecutive poses, and crossings between runs)
SNAP = 0.20        # rad: the start and the goal must be this close to a taught pose
SIMPLIFY = 0.03    # rad: simplification tolerance of the returned route


def taught_files():
    return sorted(glob.glob(os.path.expanduser('~/arm_stream*.csv')) + glob.glob(os.path.expanduser('~/arm_range*.csv')))


def load_roadmap(files=None):
    """(nodes (n, 4), edges [(i, j)]) from hand-guided streams: consecutive nodes of one stream, plus near neighbours across streams."""
    nodes, edges = [], []
    for path in files if files is not None else taught_files():
        prev = None
        for r in csv.DictReader(open(path)):
            q = np.array([float(r[j]) for j in JOINTS])
            if prev is not None and np.linalg.norm(q - nodes[prev]) < SPACING:
                continue
            nodes.append(q)
            if prev is not None and np.linalg.norm(q - nodes[prev]) <= 3 * LINK:     # a bigger jump is a gap in the recording
                edges.append((prev, len(nodes) - 1))
            prev = len(nodes) - 1
    n = np.array(nodes).reshape(-1, len(JOINTS))
    if len(n):
        d = np.linalg.norm(n[:, None, :] - n[None, :, :], axis=2)
        edges += [(int(i), int(j)) for i, j in zip(*np.where(np.triu(d <= LINK, 1)))]
    return n, edges


def plan(nodes, edges, q_from, q_to, snap=SNAP):
    """(waypoints [4-vectors] or None, why). q_from / q_to: dicts or arrays of the four joint angles."""
    a, b = (np.array([q[j] for j in JOINTS]) if isinstance(q, dict) else np.asarray(q, float) for q in (q_from, q_to))
    if not len(nodes):
        return None, 'no taught poses found (~/arm_stream*.csv, ~/arm_range*.csv)'
    sa, sb = np.linalg.norm(nodes - a, axis=1), np.linalg.norm(nodes - b, axis=1)
    i, j = int(sa.argmin()), int(sb.argmin())
    if sa[i] > snap:
        return None, f'the arm is {sa[i]:.2f} rad from the nearest taught pose (limit {snap}): move it by hand onto a taught route'
    if sb[j] > snap:
        return None, f'the target is {sb[j]:.2f} rad from the nearest taught pose (limit {snap})'
    adj = {}
    for u, v in edges:
        w = float(np.linalg.norm(nodes[u] - nodes[v]))
        adj.setdefault(u, []).append((v, w))
        adj.setdefault(v, []).append((u, w))
    dist, prev, heap = {i: 0.0}, {}, [(0.0, i)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == j:
            break
        if d > dist.get(u, np.inf):
            continue
        for v, w in adj.get(u, ()):
            if d + w < dist.get(v, np.inf):
                dist[v], prev[v] = d + w, u
                heapq.heappush(heap, (d + w, v))
    if j not in dist:
        return None, 'no taught route connects them (the taught streams are not joined near the start or the goal)'
    path = [j]
    while path[-1] != i:
        path.append(prev[path[-1]])
    pts = [a] + [nodes[k] for k in reversed(path)] + [b]
    return simplify(pts), f'{len(path)} taught poses, {sum(np.linalg.norm(p - q) for p, q in zip(pts, pts[1:])):.2f} rad of joint travel'


def simplify(pts, tol=SIMPLIFY):
    """Ramer-Douglas-Peucker in joint space: drop poses that lie within `tol` of the line between their neighbours."""
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    t = np.clip(np.dot(np.array(pts) - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
    d = np.linalg.norm(np.array(pts) - (a + t[:, None] * ab), axis=1)
    k = int(d.argmax())
    if d[k] <= tol:
        return [a, b]
    return simplify(pts[:k + 1], tol)[:-1] + simplify(pts[k:], tol)


def main():
    from arm_tool import RATE, load_rest
    from path_check import parse_pose
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--from', dest='src', required=True)
    ap.add_argument('--to', default='rest')
    a = ap.parse_args()
    rest = load_rest()
    nodes, edges = load_roadmap()
    wps, why = plan(nodes, edges, parse_pose(a.src, rest), parse_pose(a.to, rest))
    print(f'roadmap: {len(nodes)} taught poses, {len(edges)} links')
    if wps is None:
        sys.exit('NO ROUTE: ' + why)
    print(f'route: {why}; {len(wps)} waypoints after simplifying; about {sum(np.abs(q - p).max() / RATE for p, q in zip(wps, wps[1:])):.0f} s at {RATE} rad/s')
    for w in wps:
        print('  ' + ' '.join(f'{v:+.2f}' for v in w))


if __name__ == '__main__':
    main()
