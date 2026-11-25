import math

import numpy as np
import pytest

import arm_control.pick as pick
from arm_control.ik import EE, JOINTS, _task, map_limits
from arm_control.motion import Refused
from arm_control.omx_fk import Arm

arm = Arm()
CUR = dict(zip(JOINTS, (0.2, -0.6, 0.5, 0.1)))
WIDE = {j: (-3.0, 3.0) for j in JOINTS}


def at(monkeypatch, xyz):
    monkeypatch.setattr(pick, 'upright_pixel_to_arm', lambda *a, **k: xyz)


def test_reachable_target_gives_approach_and_grasp_that_reach_it(monkeypatch):
    t = _task(arm, (0.1, -0.7, 0.8, 0.0))
    at(monkeypatch, t[:3] - np.array([math.cos(0.1), math.sin(0.1), 0.0]) * 0.03)     # offset pushes it back onto t
    r = pick.grasp_poses(arm, CUR, 640, 360, 0.2, 1280, 720, {**WIDE, 'joint1': (-0.5, 0.5)}, (-1, -1, -1), (1, 1, 1))
    got = _task(arm, [r['grasp'][j] for j in JOINTS])
    assert np.linalg.norm(got[:3] - r['point']) < 0.004
    pre = _task(arm, [r['pre'][j] for j in JOINTS])
    assert np.linalg.norm(pre[:3] - got[:3]) == pytest.approx(0.06, abs=0.01)


def test_refusals(monkeypatch):
    at(monkeypatch, np.array([0.01, 0.0, 0.2]))
    with pytest.raises(Refused, match='above the arm base'):
        pick.grasp_poses(arm, CUR, 0, 0, 0.2, 1280, 720, WIDE, (-1, -1, -1), (1, 1, 1))
    at(monkeypatch, np.array([0.3, 0.0, 0.2]))
    with pytest.raises(Refused, match='outside the configured workspace'):
        pick.grasp_poses(arm, CUR, 0, 0, 0.2, 1280, 720, WIDE, (-1, -1, -1), (0.2, 1, 1))
    at(monkeypatch, np.array([3.0, 0.0, 0.2]))
    with pytest.raises(Refused, match='no arm pose'):
        pick.grasp_poses(arm, CUR, 0, 0, 0.2, 1280, 720, WIDE, (-9, -9, -9), (9, 9, 9))


def test_place_uses_the_same_checks(monkeypatch):
    t = _task(arm, (0.1, -0.7, 0.8, 0.0))
    r = pick.approach_poses(arm, CUR, t[:3], {**WIDE, 'joint1': (-0.5, 0.5)}, (-1, -1, -1), (1, 1, 1))
    assert np.linalg.norm(np.array(_task(arm, [r['grasp'][j] for j in JOINTS])[:3]) - t[:3]) < 0.004
    with pytest.raises(Refused, match='outside the configured workspace'):
        pick.approach_poses(arm, CUR, (0.3, 0.0, 0.2), WIDE, (-1, -1, -1), (0.2, 1, 1))
