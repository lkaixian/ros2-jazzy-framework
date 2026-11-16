import json
import os

import pytest

from bt_rehearsal.isolation import Observable
from bt_rehearsal.llm import LlmPlanner, Reflector, template_lesson
from bt_rehearsal.loop import rehearse, to_memory
from bt_rehearsal.sim import KinematicNavSim, Task

MAPS = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'maps', 'test_room.yaml')
PLACES = {'start': (0.5, 0.5), 'door': (5.05, 2.9), 'right_room': (9.5, 5.5)}


def xml_for(place):
    return (f'<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">'
            f'<NavigateToPlace place="{place}"/></BehaviorTree></root>')


class Scripted:
    """Planner returning scripted specs and recording the feedback it was given."""
    def __init__(self, places):
        self.places, self.seen = list(places), []

    def propose(self, task, feedback=()):
        self.seen.append(list(feedback))
        return json.dumps({'place': self.places[min(len(self.seen), len(self.places)) - 1]})


@pytest.fixture
def sim():
    from nav_sim.occupancy import OccupancyMap
    return KinematicNavSim(OccupancyMap.from_yaml(MAPS), PLACES, scene_summary={'objects': ['cup'], 'num_persons': 0})


def build(spec):
    return xml_for(json.loads(spec)['place'])


def validate(xml):
    return [] if 'kitchen' not in xml else ["port 'place'='kitchen' is not a known value"]


def test_succeeds_after_feedback(sim):
    pl = Scripted(['kitchen', 'nowhere', 'right_room'])
    r = rehearse(Task('go to the right room', 'right_room'), pl, build, validate, sim, Reflector())
    assert r.success and r.attempts == 3 and len(r.lessons) == 2
    assert 'validator' in r.lessons[0].lower() and 'nowhere' in r.lessons[1]
    assert pl.seen[0] == [] and pl.seen[2] == r.lessons and r.validated_attempts == 2


def test_budget_exhausted(sim):
    r = rehearse(Task('x', 'right_room'), Scripted(['kitchen']), build, validate, sim, Reflector(), max_attempts=2)
    assert r.status == 'exhausted' and r.attempts == 2


def test_timeout(sim):
    t = iter([0, 0, 999])
    r = rehearse(Task('x', 'right_room'), Scripted(['kitchen']), build, validate, sim, max_attempts=5,
                 timeout_s=10, clock=lambda: next(t))
    assert r.status == 'timeout'


def test_wrong_destination_is_failure(sim):
    r = rehearse(Task('x', 'right_room'), Scripted(['door']), build, validate, sim, Reflector(), max_attempts=1)
    assert not r.success and 'did not reach' in r.lessons[0]


def test_no_reflection_feeds_raw_context(sim):
    pl = Scripted(['kitchen', 'right_room'])
    rehearse(Task('x', 'right_room'), pl, build, validate, sim, None)
    assert pl.seen[1][0].startswith('Scene:')


def test_planner_unreachable_degrades(sim):
    def boom(*_):
        raise ConnectionRefusedError('down')
    r = rehearse(Task('x'), LlmPlanner(boom, ''), build, validate, sim)
    assert r.status == 'planner_unavailable'


def test_reflector_falls_back_and_lints(sim):
    obs = Observable({'objects': []}, ('bad port',))
    assert Reflector(lambda *_: 'robot stood at 1.5, 2.5').lesson('t', obs) == template_lesson(obs)

    def down(*_):
        raise OSError
    assert Reflector(down).lesson('t', obs) == template_lesson(obs)
    assert Reflector(lambda *_: 'Check the place name.\nextra').lesson('t', obs) == template_lesson(obs) + '. Check the place name.'


def test_to_memory(sim):
    t = Task('go to the right room', 'right_room')
    r = rehearse(t, Scripted(['right_room']), build, validate, sim, Reflector())
    m = to_memory(t, r, 'kobuki', '0.1.0', 'rules')
    m.check()
    assert m.scene == {'objects': ['cup'], 'num_persons': 0}
    with pytest.raises(ValueError):
        to_memory(t, rehearse(t, Scripted(['kitchen']), build, validate, sim, max_attempts=1), 'k', '', '')


def test_reflector_gets_ports_and_places(sim):
    seen = []

    class Spy(Reflector):
        def lesson(self, task, obs, detail=''):
            seen.append(detail)
            return super().lesson(task, obs, detail)
    rehearse(Task('x', 'right_room'), Scripted(['nowhere', 'right_room']), build, validate, sim, Spy(), hint='Valid places: door.')
    assert "NavigateToPlace with ports {'place': 'nowhere'}" in seen[0] and 'Valid places: door.' in seen[0]
