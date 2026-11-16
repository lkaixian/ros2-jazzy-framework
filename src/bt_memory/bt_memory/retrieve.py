"""Embedding-free retrieval: normalized task text plus scene-object overlap.

`Retriever` is the seam for embeddings later: implement `rank(task, scene, memories)` returning
[(score, memory)] and pass it where `TextSceneRetriever` is used. A hit is a PROPOSAL: callers must
re-validate `memory.xml` against the current catalog before using it.
"""
import xml.etree.ElementTree as ET
from typing import Protocol

from bt_memory.schema import norm_task


class Retriever(Protocol):
    def rank(self, task, scene, memories): ...


def _jaccard(a, b):
    return len(a & b) / len(a | b) if a | b else 0.0


class TextSceneRetriever:
    def __init__(self, scene_weight=0.3):
        self.w = scene_weight

    def score(self, task, scene, m):
        key = norm_task(task)
        if key == m.task_norm:
            return 1.0
        t = _jaccard(set(key.split()), set(m.task_norm.split()))
        a, b = set(scene.get('objects', [])), set(m.scene.get('objects', []))
        return t if not (a or b) else (1 - self.w) * t + self.w * _jaccard(a, b)

    def rank(self, task, scene, memories):
        return sorted(((self.score(task, scene, m), m) for m in memories), key=lambda h: -h[0])


def retrieve(store, task, scene, embodiment, k=3, min_score=0.5, retriever=None):
    """Top-k [(score, memory)] for this embodiment with score >= min_score. Corrupt files are skipped."""
    mems, _ = store.all()
    ranked = (retriever or TextSceneRetriever()).rank(task, scene, [m for m in mems if m.embodiment == embodiment])
    return [h for h in ranked if h[0] >= min_score][:k]


def summarize_plan(xml):
    """Leaf nodes in order as `Node(port=value, ...)`: compact enough for a 4 GB-class context."""
    out = []
    for el in ET.fromstring(xml).iter():
        if not len(el) and el.tag not in ('root', 'BehaviorTree'):
            out.append(f"{el.tag}({', '.join(f'{k}={v}' for k, v in el.attrib.items() if k != 'name')})")
    return ' -> '.join(out)


def format_examples(hits):
    return [f"Task: {m.task}\nPlan: {summarize_plan(m.xml)}"
            + (f"\nLessons: {' '.join(m.lessons)}" if m.lessons else '') for _, m in hits]


def first_candidate(hits, validate):
    """First hit whose XML validates NOW (validate(xml) -> [errors]); None if none does."""
    for _, m in hits:
        if not validate(m.xml):
            return m
    return None


def unknown_places(xml, places):
    """Errors for NavigateToPlace ports naming a place the CURRENT map does not have. The validator does not know
    places, so without this a memory made on an old map would still look valid after the map changed.
    Blackboard references ({x}) cannot be checked statically and are skipped."""
    return [f"place '{e.get('place')}' is not in the current places file"
            for e in ET.fromstring(xml).iter('NavigateToPlace')
            if e.get('place') and not e.get('place').startswith('{') and e.get('place') not in places]
