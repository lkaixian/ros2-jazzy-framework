"""Data isolation: what the planner may see versus what only the scorer may see.

Two unrelated types, no common base. A simulator returns both; only `Observable` can be turned into
planner context (`context_text`). `Privileged` has no text rendering at all, and `Observable` accepts
only allow-listed keys, so a ground-truth value cannot ride along in a dict.
"""
import json
from dataclasses import dataclass, field

# What a real robot can report at deployment: perception summary, validator output, executor feedback.
SCENE_KEYS = ('objects', 'num_persons', 'people', 'known_faces')
OUTCOME_KEYS = ('result', 'failed_node', 'message', 'steps')


class IsolationError(ValueError):
    pass


def _only(d, allowed, what):
    extra = set(d) - set(allowed)
    if extra:
        raise IsolationError(f'{what} has keys a real robot cannot supply: {sorted(extra)}')
    return dict(d)


@dataclass(frozen=True)
class Observable:
    scene: dict                                       # scene/summary JSON (perception)
    validator_errors: tuple = ()                      # bt_validator output
    outcome: dict = field(default_factory=dict)       # executor-style: result, failed_node, message, steps

    def __post_init__(self):
        _only(self.scene, SCENE_KEYS, 'scene')
        _only(self.outcome, OUTCOME_KEYS, 'outcome')

    def context_text(self):
        parts = [f'Scene: {json.dumps(self.scene, sort_keys=True)}']
        if self.validator_errors:
            parts.append('Validator errors:\n- ' + '\n- '.join(self.validator_errors))
        if self.outcome:
            o = self.outcome
            parts.append(f"Run result: {o.get('result', '?')}"
                         + (f" at node {o['failed_node']}" if o.get('failed_node') else '')
                         + (f" ({o['message']})" if o.get('message') else ''))
        return '\n'.join(parts)


@dataclass(frozen=True)
class Privileged:
    """Simulator ground truth. Used by the scorer only. Deliberately has no text/dict export."""
    pose: tuple = (0.0, 0.0, 0.0)
    collisions: int = 0
    extra: dict = field(default_factory=dict)


def require_observable(x):
    """Gate at the planner/reflector boundary."""
    if not isinstance(x, Observable):
        raise IsolationError(f'planner context must be Observable, got {type(x).__name__}')
    return x
