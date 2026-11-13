"""Rule-based task_spec -> BT XML, used when no fine-tuned SLM is configured.

task_spec JSON:
  {"goal": str, "retry": N?, "steps": [STEP, ...]}
  STEP = {"node": "DetectObject", "ports": {"label": "cup"}}
       | {"group": "Sequence|Fallback|ReactiveSequence|ReactiveFallback",
          "retry": N?, "steps": [STEP, ...]}
"""
import json
import re
from xml.sax.saxutils import quoteattr

GROUPS = ('Sequence', 'Fallback', 'ReactiveSequence', 'ReactiveFallback')


def _slug(s):
    return re.sub(r'\W+', '_', s).strip('_') or 'Main'


def _keep(step, known_ids):
    """On retry, drop unknown leaves (and groups left empty) so the validator can pass."""
    if 'group' in step:
        step = dict(step, steps=[s for s in (_keep(c, known_ids) for c in step.get('steps', []))
                                 if s])
        return step if step['steps'] else None
    return step if step.get('node') in known_ids else None


def _render(step, pad):
    sp = ' ' * pad
    if 'group' in step:
        inner = [_render(c, pad + 2) for c in step.get('steps', [])]
        body = f'{sp}<{step["group"]}>\n' + '\n'.join(inner) + f'\n{sp}</{step["group"]}>'
        return _retry(step, body, pad)
    attrs = ''.join(f' {k}={quoteattr(str(v))}' for k, v in step.get('ports', {}).items())
    return f'{sp}<{step["node"]}{attrs}/>'


def _retry(spec, body, pad):
    if not spec.get('retry'):
        return body
    sp = ' ' * pad
    indented = '\n'.join('  ' + ln for ln in body.split('\n'))
    return (f'{sp}<RetryUntilSuccessful num_attempts="{int(spec["retry"])}">\n'
            f'{indented}\n{sp}</RetryUntilSuccessful>')


def build_xml(task_spec, known_ids=None, errors=()):
    """known_ids: when given and `errors` is non-empty (retry), unknown steps are dropped."""
    spec = json.loads(task_spec)
    steps = spec.get('steps', [])
    if errors and known_ids is not None:
        steps = [s for s in (_keep(c, known_ids) for c in steps) if s]
    tid = _slug(spec.get('goal', 'Main'))
    top = {'group': 'Sequence', 'steps': steps, 'retry': spec.get('retry')}
    body = _render(top, 4)
    return (f'<root BTCPP_format="4" main_tree_to_execute="{tid}">\n'
            f'  <BehaviorTree ID="{tid}">\n{body}\n  </BehaviorTree>\n</root>')
