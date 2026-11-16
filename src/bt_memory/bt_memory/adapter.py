"""Cross-embodiment adapter: remap a memory's action names and ports through the TARGET robot's
capability file, then re-validate. An action the target does not have is a loud error, never a drop.

Capability file (YAML):
    embodiment: my_arm_bot
    actions:                        # source node id -> how the target spells it
      NavigateToPlace: {name: GoToPlace, ports: {place: target}, defaults: {speed: "0.2"}}
      Speak: {name: Speak}
"""
import xml.etree.ElementTree as ET
from dataclasses import replace

import yaml


# BehaviorTree.CPP built-ins present on every embodiment (never remapped).
PASSTHROUGH = {'root', 'BehaviorTree', 'TreeNodesModel', 'SubTree', 'Sequence', 'ReactiveSequence', 'Fallback',
               'ReactiveFallback', 'Parallel', 'RetryUntilSuccessful', 'Repeat', 'Inverter', 'ForceSuccess',
               'ForceFailure', 'Timeout', 'AlwaysSuccess', 'AlwaysFailure', 'Sleep', 'KeepRunningUntilFailure'}


class AdaptError(ValueError):
    pass


def load_capabilities(path):
    with open(path) as f:
        cap = yaml.safe_load(f) or {}
    if not isinstance(cap.get('embodiment'), str) or not isinstance(cap.get('actions'), dict):
        raise AdaptError(f'{path}: needs `embodiment: <id>` and an `actions:` mapping')
    for src, a in cap['actions'].items():
        if not isinstance(a, dict) or not a.get('name'):
            raise AdaptError(f"{path}: action '{src}' needs a `name`")
    return cap


def adapt_xml(xml, cap):
    root, missing = ET.fromstring(xml), set()
    for el in root.iter():
        if el.tag in PASSTHROUGH:
            continue
        a = cap['actions'].get(el.tag)
        if a is None:
            missing.add(el.tag)
            continue
        ports = a.get('ports') or {}
        el.attrib = {ports.get(k, k): v for k, v in el.attrib.items()}
        for k, v in (a.get('defaults') or {}).items():
            el.attrib.setdefault(k, str(v))
        el.tag = a['name']
    if missing:
        raise AdaptError(f"embodiment '{cap['embodiment']}' has no action for: {', '.join(sorted(missing))}")
    return ET.tostring(root, encoding='unicode')


def adapt_memory(mem, cap, validate):
    """validate(xml) -> [errors] against the TARGET catalog. Returns a new memory; raises AdaptError."""
    if mem.embodiment == cap['embodiment']:
        raise AdaptError('memory is already for this embodiment')
    xml = adapt_xml(mem.xml, cap)
    errs = validate(xml)
    if errs:
        raise AdaptError('adapted plan fails validation on the target: ' + '; '.join(errs[:5]))
    return replace(mem, xml=xml, embodiment=cap['embodiment'], adapted_from=mem.embodiment, task_norm=mem.task_norm)
