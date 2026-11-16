import os

import pytest

from bt_rehearsal.isolation import IsolationError, Observable, Privileged, require_observable
from bt_rehearsal.sim import KinematicNavSim, Task

MAPS = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'maps', 'test_room.yaml')
PLACES = {'start': (0.5, 0.5), 'left_room': (3.0, 3.0), 'door': (5.05, 2.9), 'right_room': (9.5, 5.5),
          'wall': (5.0, 0.5)}


def tree(body):
    return f'<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">{body}</BehaviorTree></root>'


@pytest.fixture(scope='module')
def sim():
    from nav_sim.occupancy import OccupancyMap
    return KinematicNavSim(OccupancyMap.from_yaml(MAPS), PLACES)


def test_navigates_through_door(sim):
    obs, priv = sim.run(tree('<Sequence><NavigateToPlace place="door"/><NavigateToPlace place="right_room"/></Sequence>'))
    assert obs.outcome['result'] == 'SUCCESS'
    assert sim.score(Task('go', 'right_room'), obs, priv)


def test_unknown_place_fails_with_node(sim):
    obs, _ = sim.run(tree('<NavigateToPlace place="kitchen"/>'))
    assert obs.outcome['failed_node'] == 'NavigateToPlace' and 'kitchen' in obs.outcome['message']


def test_wrong_goal_scores_false(sim):
    obs, priv = sim.run(tree('<NavigateToPlace place="left_room"/>'))
    assert obs.outcome['result'] == 'SUCCESS' and not sim.score(Task('go', 'right_room'), obs, priv)


def test_fallback_and_unsupported(sim):
    obs, _ = sim.run(tree('<Fallback><NavigateToPlace place="kitchen"/><Sleep msec="1"/></Fallback>'))
    assert obs.outcome['result'] == 'SUCCESS'
    obs, _ = sim.run(tree('<DetectObject label="cup"/>'))
    assert 'unsupported' in obs.outcome['message']


def test_isolation_types():
    with pytest.raises(IsolationError):
        Observable({'objects': [], 'pose': [1, 2]})
    with pytest.raises(IsolationError):
        Observable({}, outcome={'goal_distance': 0.2})
    with pytest.raises(IsolationError):
        require_observable(Privileged())
    assert not hasattr(Privileged(), 'context_text')
