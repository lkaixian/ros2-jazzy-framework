"""Face gallery: one .npy embedding per identity in a directory."""
import os

import numpy as np


def _unit(v):
    v = np.asarray(v, dtype=np.float32).ravel()
    return v / (np.linalg.norm(v) + 1e-9)


class Gallery:
    def __init__(self, path):
        self.path = path
        self.names, self.vecs = [], []
        if path and os.path.isdir(path):
            for f in sorted(os.listdir(path)):
                if f.endswith('.npy'):
                    self.names.append(f[:-4])
                    self.vecs.append(_unit(np.load(os.path.join(path, f))))

    def match(self, emb, threshold):
        """Return (identity, cosine similarity)."""
        if not self.vecs:
            return 'unknown', 0.0
        sims = np.stack(self.vecs) @ _unit(emb)
        i = int(np.argmax(sims))
        return (self.names[i] if sims[i] >= threshold else 'unknown'), float(sims[i])

    def add(self, name, emb):
        os.makedirs(self.path, exist_ok=True)
        np.save(os.path.join(self.path, f'{name}.npy'), _unit(emb))
