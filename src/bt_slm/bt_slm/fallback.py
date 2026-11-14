"""Rule-based task_spec -> BT XML, used when no fine-tuned SLM is configured.

task_spec JSON:
  {"goal": str, "retry": N?, "steps": [STEP, ...]}
  STEP = {"node": "DetectObject", "ports": {"label": "cup"}}
       | {"group": "Sequence|Fallback|ReactiveSequence|ReactiveFallback|Parallel",
          "retry": N?, "repeat": N?, "success_count": N? (Parallel), "failure_count": N? (Parallel), "steps": [STEP, ...]}
  (macro and foreach steps are expanded by bt_common.macros before this module sees the spec)
"""
import json
import re
from xml.sax.saxutils import quoteattr

GROUPS = ('Sequence', 'Fallback', 'ReactiveSequence', 'ReactiveFallback', 'Parallel')


def _slug(s):
    return re.sub(r'\W+', '_', s).strip('_') or 'Main'


def _unknown(step, known_ids):
    """Ids of the leaves a retry cannot keep: a plan with steps removed is not the plan that was asked for (T-22)."""
    if 'group' in step:
        return [u for c in step.get('steps', []) for u in _unknown(c, known_ids)]
    return [] if step.get('node') in known_ids else [str(step.get('node'))]


def _render(step, pad):
    sp = ' ' * pad
    if 'group' in step:
        inner = [_render(c, pad + 2) for c in step.get('steps', [])]
        attrs = ''.join(f' {k}="{int(step[k])}"' for k in ('success_count', 'failure_count')
                        if step['group'] == 'Parallel' and step.get(k) is not None)
        body = f'{sp}<{step["group"]}{attrs}>\n' + '\n'.join(inner) + f'\n{sp}</{step["group"]}>'
        return _retry(step, body, pad)
    attrs = ''.join(f' {k}={quoteattr(str(v))}' for k, v in step.get('ports', {}).items())
    return f'{sp}<{step["node"]}{attrs}/>'


def _wrap(body, pad, tag, attr, n):
    sp = ' ' * pad
    indented = '\n'.join('  ' + ln for ln in body.split('\n'))
    return f'{sp}<{tag} {attr}="{int(n)}">\n{indented}\n{sp}</{tag}>'


def _retry(spec, body, pad):
    """`repeat: N` runs the group N times in a row (Repeat); `retry: N` retries it on failure (outermost)."""
    if spec.get('repeat'):
        body = _wrap(body, pad, 'Repeat', 'num_cycles', spec['repeat'])
    if spec.get('retry'):
        body = _wrap(body, pad, 'RetryUntilSuccessful', 'num_attempts', spec['retry'])
    return body


def build_xml(task_spec, known_ids=None, errors=()):
    """known_ids: when given and `errors` is non-empty (retry), unknown steps are dropped."""
    spec = json.loads(task_spec)
    steps = spec.get('steps', [])
    if errors and known_ids is not None:
        bad = [u for c in steps for u in _unknown(c, known_ids)]
        if bad:
            raise ValueError(f'plan uses unknown nodes {sorted(set(bad))}; refusing to drop them')
    tid = _slug(spec.get('goal', 'Main'))
    top = {'group': 'Sequence', 'steps': steps, 'retry': spec.get('retry')}
    body = _render(top, 4)
    return (f'<root BTCPP_format="4" main_tree_to_execute="{tid}">\n'
            f'  <BehaviorTree ID="{tid}">\n{body}\n  </BehaviorTree>\n</root>')
