"""WICL retrieval at deployment. Pure logic, no ROS. Off unless the orchestrator's memory_enabled is true.

A retrieved memory is a proposal: `candidate` is only returned if its XML passes the validator NOW.
"""
import os

from bt_memory.retrieve import first_candidate, format_examples, retrieve
from bt_memory.schema import MemoryStore

DEFAULT_DIR = os.path.expanduser('~/.local/share/robocup_fw/bt_memory')


class MemoryHook:
    def __init__(self, store_dir='', embodiment='kobuki', k=3, min_score=0.5, use_candidate=True, retriever=None):
        self.store = MemoryStore(store_dir or DEFAULT_DIR)
        self.embodiment, self.k, self.min_score = embodiment, k, min_score
        self.use_candidate, self.retriever = use_candidate, retriever

    def on_task(self, text, scene, validate):
        """-> (candidate_memory | None, planner_text). validate(xml) -> [errors] against the current catalog.
        planner_text = the task plus few-shot examples (the plain task when nothing matched)."""
        hits = retrieve(self.store, text, scene, self.embodiment, self.k, self.min_score, self.retriever)
        cand = first_candidate(hits, validate) if self.use_candidate else None
        if not hits:
            return None, text
        return cand, text + '\n\nSimilar tasks done before (examples, may not fit this scene):\n' + \
            '\n\n'.join(format_examples(hits))
