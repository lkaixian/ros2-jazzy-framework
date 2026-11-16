"""Rehearse -> reflect -> retry. The planner only ever sees Observable-derived text."""
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from bt_memory.schema import TaskMemory
from bt_rehearsal.isolation import Observable, require_observable
from bt_rehearsal.llm import PlannerUnavailable


@dataclass
class Result:
    status: str                  # success | exhausted | timeout | planner_unavailable
    attempts: int = 0
    xml: str = ''
    lessons: list = field(default_factory=list)
    outcome: dict = field(default_factory=dict)
    scene: dict = field(default_factory=dict)
    validated_first_try: bool = False
    validated_attempts: int = 0     # attempts whose plan passed the validator

    @property
    def success(self):
        return self.status == 'success'


def rehearse(task, planner, build, validate, sim, reflector=None, max_attempts=4, timeout_s=120.0,
             clock=time.monotonic, hint=''):
    """task: bt_rehearsal.sim.Task. build(spec_json) -> xml. validate(xml) -> [errors]. reflector=None
    disables lessons (feedback is then only the raw Observable context)."""
    t0, feedback, lessons, res = clock(), [], [], Result('exhausted')
    sim.reset()
    scene = sim.scene()
    for n in range(1, max_attempts + 1):
        if clock() - t0 > timeout_s:
            res.status = 'timeout'
            break
        res.attempts = n
        try:
            spec = planner.propose(task.text, list(feedback))
            xml = build(spec)
            errs = tuple(validate(xml))
        except PlannerUnavailable:
            res.status = 'planner_unavailable'
            break
        except Exception as e:  # noqa: BLE001  a bad spec is feedback, not a crash
            xml, errs = '', (f'plan could not be built: {type(e).__name__}: {e}',)
        if errs:
            obs = Observable(scene, errs)
        else:
            res.validated_attempts += 1
            res.validated_first_try |= n == 1
            obs, priv = sim.run(xml)
            if sim.score(task, obs, priv):          # the ONLY use of ground truth
                res.status, res.xml, res.outcome, res.scene = 'success', xml, dict(obs.outcome), scene
                res.lessons = lessons
                return res
        require_observable(obs)
        if reflector:
            lessons.append(reflector.lesson(task.text, obs, _detail(xml, obs, hint)))
            feedback.append(lessons[-1])
        else:
            feedback.append(obs.context_text())
    res.lessons = lessons
    return res


def _detail(xml, obs, hint):
    """What the planner itself wrote at the failing node, plus static config (valid place names): both
    observable on a real robot. Gives a small reflector something concrete to name."""
    node, parts = obs.outcome.get('failed_node'), []
    if node and xml:
        try:
            el = next((e for e in ET.fromstring(xml).iter() if e.tag == node), None)
        except ET.ParseError:
            el = None
        if el is not None:
            parts.append(f'The plan used {node} with ports {dict(el.attrib)}.')
    return ' '.join(parts + ([hint] if hint else []))


def to_memory(task, res, embodiment, framework_version, model_version):
    if not res.success:
        raise ValueError(f'rehearsal did not succeed: {res.status}')
    return TaskMemory(task=task.text, scene=res.scene, xml=res.xml, lessons=list(res.lessons),
                      outcome={'success': True, 'attempts': res.attempts, 'steps': res.outcome.get('steps', 0)},
                      embodiment=embodiment, framework_version=framework_version, model_version=model_version)
