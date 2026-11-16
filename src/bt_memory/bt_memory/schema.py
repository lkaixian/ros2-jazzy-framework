"""Task Memory record and its on-disk store. Pure Python, no ROS.

A memory is a proposal, never a trusted plan: it holds nothing that needs a weight update, and
whoever loads it must re-validate `xml` against the CURRENT scene and catalog.
"""
import hashlib
import json
import os
import re
import tempfile
from dataclasses import asdict, dataclass, field, fields

SCHEMA_VERSION = 1
# Lessons and scenes are text the planner will read: keep simulator facts out (the paper's "memory linter").
_COORDS = re.compile(r'-?\d+\.\d+\s*[,;]\s*-?\d+\.\d+')
_FORBIDDEN = ('ground_truth', 'ground truth', 'privileged', 'nav_sim/', 'seed')


class MemoryError_(ValueError):
    """Bad or incompatible memory record."""


def norm_task(text):
    """Lower-case, punctuation-free, single-spaced task text: the matching key."""
    return ' '.join(re.sub(r'[^\w\s]', ' ', text.lower()).split())


def lint(text):
    """Return a list of problems with a lesson/scene string (empty = clean)."""
    low = text.lower()
    bad = [f"contains '{w}'" for w in _FORBIDDEN if w in low]
    if _COORDS.search(text):
        bad.append('contains a coordinate pair')
    return bad


@dataclass
class TaskMemory:
    task: str
    scene: dict            # scene/summary at rehearsal time: only what perception reports
    xml: str               # validated BT XML
    lessons: list = field(default_factory=list)
    outcome: dict = field(default_factory=dict)    # {success, attempts, failed_steps}: no ground truth
    embodiment: str = 'kobuki'
    framework_version: str = ''
    model_version: str = ''
    schema_version: int = SCHEMA_VERSION
    task_norm: str = ''
    adapted_from: str = ''       # source embodiment when produced by the adapter

    def __post_init__(self):
        self.task_norm = self.task_norm or norm_task(self.task)

    @property
    def id(self):
        return hashlib.sha256(f'{self.embodiment}|{self.task_norm}|{self.xml}'.encode()).hexdigest()[:12]

    def check(self):
        """Raise if the record may not be stored."""
        if not self.task.strip() or not self.xml.strip():
            raise MemoryError_('task and xml are required')
        if not self.outcome.get('success'):
            raise MemoryError_('only successful rehearsals are stored')
        problems = [f'lesson {i}: {p}' for i, t in enumerate(self.lessons) for p in lint(t)]
        problems += [f'scene: {p}' for p in lint(json.dumps(self.scene))]
        if problems:
            raise MemoryError_('; '.join(problems))

    def to_json(self):
        return json.dumps(asdict(self), indent=2, sort_keys=True, ensure_ascii=False)

    @classmethod
    def from_dict(cls, d):
        v = d.get('schema_version')
        if v != SCHEMA_VERSION:
            raise MemoryError_(f'incompatible memory schema_version {v!r} (this build reads {SCHEMA_VERSION})')
        unknown = set(d) - {f.name for f in fields(cls)}
        if unknown:
            raise MemoryError_(f'unknown fields {sorted(unknown)}')
        try:
            return cls(**d)
        except TypeError as e:
            raise MemoryError_(str(e)) from e


class MemoryStore:
    """One `<id>.json` per memory in a directory."""

    def __init__(self, root):
        self.root = root

    def save(self, mem):
        mem.check()
        os.makedirs(self.root, exist_ok=True)
        path = os.path.join(self.root, f'{mem.id}.json')
        fd, tmp = tempfile.mkstemp(dir=self.root, suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            f.write(mem.to_json())
        os.replace(tmp, path)          # a crash never leaves a half-written memory
        return path

    def load(self, path):
        try:
            with open(path) as f:
                return TaskMemory.from_dict(json.load(f))
        except json.JSONDecodeError as e:
            raise MemoryError_(f'{path}: not JSON ({e})') from e

    def all(self):
        """(memories, problems): an incompatible or corrupt file is reported, not fatal."""
        mems, problems = [], []
        for name in sorted(os.listdir(self.root)) if os.path.isdir(self.root) else []:
            if name.endswith('.json'):
                try:
                    mems.append(self.load(os.path.join(self.root, name)))
                except (MemoryError_, OSError) as e:
                    problems.append(f'{name}: {e}')
        return mems, problems
