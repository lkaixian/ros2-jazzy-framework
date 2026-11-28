"""Online target classifier for FollowTarget, after Ye et al., "Person Re-Identification for Robot Person Following with Online
Continual Learning" (arXiv 2309.11727), without the online fine-tuning of the feature network.

While the target is confidently seen it learns, in closed form (ridge regression), to tell the target (label 1) from the other people
in the same frame (label 0) on their appearance vectors. When the target is lost, a candidate is accepted only after scoring above
`reid` for `frames` consecutive calls. The paper's reid is 0.7; on two real people (tools/reid/target_check.py) the target scored
0.60-0.83 and the other person at most 0.18, so 0.5 sits well inside that gap. No ROS imports.

Differences from the paper (not in it, added here): the classifier trains on the short-term memory plus a few samples drawn from a
small long-term reservoir, so older views (front, side, back) are not forgotten; there is no per-part model and no feature fine-tuning.
shortcut: frozen features, so a long change of lighting or clothes is not learned. Upgrade to fine-tuning the extractor online
(the paper's main gain) if the features drift on real runs.
"""
import numpy as np


class TargetModel:
    def __init__(self, short=32, long=128, lam=1.0, sw=0.35, reid=0.5, frames=5, min_pos=8, min_neg=16, seed=0):
        self.short, self.long, self.lam, self.sw, self.reid, self.frames = short, long, lam, sw, reid, frames
        self.min_pos, self.min_neg = min_pos, min_neg
        self.pos, self.neg = [], []          # short-term: the latest vectors of the target and of the others
        self.lpos, self.lneg, self.seen = [], [], [0, 0]    # long-term reservoirs and how many were ever offered
        self.w = None
        self.rng = np.random.default_rng(seed)
        self._prev, self._cur = {}, {}

    @property
    def ready(self):
        return self.w is not None

    def score(self, e):
        return float(np.append(e, 1.0) @ self.w)

    def _remember(self, k, e):
        mem = (self.lpos, self.lneg)[k]
        self.seen[k] += 1
        if len(mem) < self.long:
            mem.append(e)
        else:
            j = int(self.rng.integers(0, self.seen[k]))      # reservoir sampling: every sample ever seen is equally likely to stay
            if j < self.long:
                mem[j] = e

    def _fit(self):
        if len(self.pos) < self.min_pos or len(self.neg) < self.min_neg:
            return
        lp = [self.lpos[i] for i in self.rng.permutation(len(self.lpos))[:self.short]]
        ln = [self.lneg[i] for i in self.rng.permutation(len(self.lneg))[:self.short]]
        P, N = self.pos + lp, self.neg + ln
        A = np.hstack([np.vstack(P + N), np.ones((len(P) + len(N), 1))])
        y = np.r_[np.ones(len(P)), np.zeros(len(N))]
        # dual form: a few dozen samples, 513 features; solves in well under a millisecond
        self.w = A.T @ np.linalg.solve(A @ A.T + self.lam * np.eye(len(A)), y)

    def update(self, target, others):
        """One frame in which the followed person and the others in view are known. -> True when it was learned from."""
        e = np.asarray(target, float)
        if self.ready and self.score(e) <= self.sw:
            return False             # the id may have switched to someone else: learn nothing from this frame
        self.pos = (self.pos + [e])[-self.short:]
        self._remember(0, e)
        for o in others:
            o = np.asarray(o, float)
            self.neg = (self.neg + [o])[-self.short:]
            self._remember(1, o)
        self._fit()
        return True

    def begin(self):
        """Start of a re-identification round: consecutive-frame counts carry over only for candidates present both times."""
        self._prev, self._cur = self._cur, {}

    def vote(self, key, ok):
        self._cur[key] = self._prev.get(key, 0) + 1 if ok else 0
        return self._cur[key]
