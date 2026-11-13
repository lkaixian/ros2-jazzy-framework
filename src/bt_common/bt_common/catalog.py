"""Catalog of allowed BT nodes: core built-ins plus extension manifests."""
import os
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


def _from_dict(d):
    out = {}
    for nid, s in (d.get('nodes') or {}).items():
        out[nid] = NodeSpec(nid, s['category'], list(s.get('required', [])),
                            list(s.get('optional', [])), s.get('description', ''))
    return out


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
