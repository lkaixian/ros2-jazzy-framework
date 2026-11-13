"""Rule-based task_spec -> BT XML, used when no fine-tuned SLM is configured.

task_spec JSON: {"goal": str, "steps": [{"node": "DetectObject", "ports": {"label": "cup"}}]}
Optional "retry": N wraps the sequence in RetryUntilSuccessful.
"""
import json
import re
from xml.sax.saxutils import quoteattr


def _slug(s):
    return re.sub(r'\W+', '_', s).strip('_') or 'Main'


def build_xml(task_spec, known_ids=None, errors=()):
    """known_ids: when given and `errors` is non-empty (retry), unknown steps are dropped."""
    spec = json.loads(task_spec)
    steps = spec.get('steps', [])
    if errors and known_ids is not None:
        steps = [s for s in steps if s.get('node') in known_ids]
    tid = _slug(spec.get('goal', 'Main'))
    lines = []
    for s in steps:
        attrs = ''.join(f' {k}={quoteattr(str(v))}' for k, v in s.get('ports', {}).items())
        lines.append(f'      <{s["node"]}{attrs}/>')
    body = '    <Sequence>\n' + '\n'.join(lines) + '\n    </Sequence>'
    if spec.get('retry'):
        body = (f'    <RetryUntilSuccessful num_attempts="{int(spec["retry"])}">\n'
                f'{body}\n    </RetryUntilSuccessful>')
    return (f'<root BTCPP_format="4" main_tree_to_execute="{tid}">\n'
            f'  <BehaviorTree ID="{tid}">\n{body}\n  </BehaviorTree>\n</root>')
