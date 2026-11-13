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


RESERVED_PORTS = ('name', 'ID')
_PORT_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def check_port_names(nid, ports):
    """BehaviorTree.CPP forbids `name`/`ID` and non-alphabetic starts as port names."""
    for p in ports:
        if p in RESERVED_PORTS or not _PORT_RE.match(p):
            raise ValueError(f"node {nid}: port name {p!r} is not allowed by BehaviorTree.CPP "
                             f"(reserved: {', '.join(RESERVED_PORTS)}; must start with a letter)")


def _from_dict(d):
    out = {}
    for nid, s in (d.get('nodes') or {}).items():
        check_port_names(nid, list(s.get('required', [])) + list(s.get('optional', [])))
        out[nid] = NodeSpec(nid, s['category'], list(s.get('required', [])),
                            list(s.get('optional', [])), s.get('description', ''))
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


def format_listing(specs):
    """Compact one-line-per-node listing used identically in prompts and training data."""
    lines = []
    for n in specs:
        ports = ','.join(n.required + [p + '?' for p in n.optional])
        lines.append(f'{n.id}[{n.category}]({ports})')
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
            n.id: NodeSpec(n.id, n.category, list(n.required_ports),
                           list(n.optional_ports), n.description) for n in msg.nodes}

    def add_manifest_dict(self, d):
        self.extensions[d['extension']] = _from_dict(d)

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
