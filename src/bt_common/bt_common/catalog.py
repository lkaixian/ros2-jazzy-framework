"""Catalog of allowed BT nodes: core built-ins plus extension manifests."""
import os
import re
from dataclasses import dataclass, field
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory

CATEGORIES = ('Action', 'Condition', 'Control', 'Decorator')


@dataclass
class NodeSpec:
    id: str
    category: str
    required: List[str] = field(default_factory=list)
    optional: List[str] = field(default_factory=list)
    description: str = ''
    # sync = finishes inside one tick; async = returns RUNNING over several ticks (a DDS action).
    # '' for Control/Decorator nodes, which have no mode of their own.
    mode: str = ''


RESERVED_PORTS = ('name', 'ID')
_PORT_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def check_port_names(nid, ports):
    """BehaviorTree.CPP forbids `name`/`ID` and non-alphabetic starts as port names."""
    for p in ports:
        if p in RESERVED_PORTS or not _PORT_RE.match(p):
            raise ValueError(f"node {nid}: port name {p!r} is not allowed by BehaviorTree.CPP "
                             f"(reserved: {', '.join(RESERVED_PORTS)}; must start with a letter)")


MODES = ('sync', 'async')


def default_mode(category, extension):
    """Mirrors bt_executor: every extension node is an ExtNode (StatefulActionNode, one DDS action
    goal, async) whatever its category; core Conditions are SimpleConditions (sync); core Actions
    default to async (Sleep) and the instant ones say `mode: sync` in core_nodes.yaml."""
    if category not in ('Action', 'Condition'):
        return ''
    return 'async' if extension or category == 'Action' else 'sync'


def _from_dict(d, extension=False):
    out = {}
    for nid, s in (d.get('nodes') or {}).items():
        check_port_names(nid, list(s.get('required', [])) + list(s.get('optional', [])))
        mode = s.get('mode') or default_mode(s['category'], extension)
        if mode and (mode not in MODES or s['category'] not in ('Action', 'Condition')):
            raise ValueError(f"node {nid}: mode {mode!r} must be one of {MODES} and only applies "
                             f"to Action/Condition nodes")
        out[nid] = NodeSpec(nid, s['category'], list(s.get('required', [])),
                            list(s.get('optional', [])), s.get('description', ''), mode)
    return out


def discover_catalog(extra_configs=()):
    """Core nodes plus every installed *_extension manifest (and any extra YAML files)."""
    from ament_index_python.packages import get_packages_with_prefixes
    cat = Catalog()
    paths = list(extra_configs)
    for pkg, prefix in sorted(get_packages_with_prefixes().items()):
        cfg = os.path.join(prefix, 'share', pkg, 'config', pkg + '.yaml')
        if pkg.endswith('_extension') and os.path.exists(cfg):
            paths.append(cfg)
    for path in paths:
        with open(path) as f:
            cat.add_manifest_dict(yaml.safe_load(f))
    return cat


def format_listing(specs, with_desc=False):
    """Compact one-line-per-node listing. The SLM prompt/training data use with_desc=False
    (it must stay byte-identical); the LLM prompt adds descriptions."""
    lines = []
    for n in specs:
        ports = ','.join(n.required + [p + '?' for p in n.optional])
        # the mode tag is LLM-only: the SLM prompt/training data must stay byte-identical
        tag = f'{n.category},{n.mode}' if with_desc and n.mode else n.category
        line = f'{n.id}[{tag}]({ports})'
        if with_desc and n.description:
            line += f' - {n.description}'
        lines.append(line)
    return '\n'.join(lines)


class Catalog:
    def __init__(self, core_file=None):
        core_file = core_file or os.path.join(
            get_package_share_directory('bt_common'), 'config', 'core_nodes.yaml')
        with open(core_file) as f:
            self.core = _from_dict(yaml.safe_load(f))
        self.extensions = {}   # extension name -> {id: NodeSpec}

    def add_manifest_msg(self, msg):
        self.extensions[msg.extension] = {
            n.id: NodeSpec(n.id, n.category, list(n.required_ports), list(n.optional_ports),
                           n.description, default_mode(n.category, True)) for n in msg.nodes}

    def add_manifest_dict(self, d):
        self.extensions[d['extension']] = _from_dict(d, extension=True)

    def all(self):
        out = dict(self.core)
        for nodes in self.extensions.values():
            out.update(nodes)
        return out

    def get(self, nid):
        return self.all().get(nid)

    def prompt_listing(self):
        """Compact one-line-per-node listing for LLM/SLM prompts."""
        lines = []
        for n in self.all().values():
            ports = ','.join(n.required + [p + '?' for p in n.optional])
            lines.append(f'{n.id}[{n.category}]({ports})')
        return '\n'.join(lines)
