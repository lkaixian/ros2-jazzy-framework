import os

import numpy as np
import pytest

from arm_control.ik import JOINTS, LIMITS, _task, solve
from arm_control.omx_fk import XACRO, Arm

# CI does not install open_manipulator_description; a present-but-broken xacro still fails
HAVE_URDF = os.path.exists(XACRO)
pytestmark = pytest.mark.skipif(not HAVE_URDF, reason='open_manipulator_description is not installed')
arm = Arm() if HAVE_URDF else None


@pytest.mark.parametrize('q', [(0.3, -0.5, 0.4, 0.2), (-0.8, 0.2, -0.3, -0.4), (0.0, -1.0, 1.0, 0.0)])
def test_fk_ik_round_trip(q):
    t = _task(arm, q)
    sol = solve(arm, t[:3], pitch=t[3])
    assert sol is not None
    got = _task(arm, [sol[j] for j in JOINTS])
    assert np.linalg.norm(got[:3] - t[:3]) < 0.003 and abs(got[3] - t[3]) < 0.02
    assert all(LIMITS[j][0] <= sol[j] <= LIMITS[j][1] for j in JOINTS)


def test_unreachable_is_none():
    assert solve(arm, (2.0, 0.0, 0.3)) is None


def test_solution_stays_inside_the_taught_range():
    from types import SimpleNamespace
    from arm_control.ik import map_limits
    fmap = SimpleNamespace(lo=dict(zip(JOINTS, (-0.5, -1.2, 0.0, -0.3))), hi=dict(zip(JOINTS, (0.5, 0.0, 1.4, 0.9))))
    lim = map_limits(fmap)
    t = _task(arm, (0.3, -0.5, 0.4, 0.2))
    sol = solve(arm, t[:3], pitch=t[3], limits=lim)
    assert sol is not None and all(lim[j][0] <= sol[j] <= lim[j][1] for j in JOINTS)
    narrow = {**lim, 'joint2': (0.3, 0.4)}                 # the reachable pose needs joint2 near -0.5: now excluded
    assert solve(arm, t[:3], pitch=t[3], limits=narrow) is None


def test_a_map_without_ranges_is_refused():
    from types import SimpleNamespace
    from arm_control.ik import map_limits
    with pytest.raises(ValueError):
        map_limits(SimpleNamespace(lo={}, hi={}))
