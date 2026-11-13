"""Face gallery: one .npy embedding per identity in a directory."""
import os
import re

import numpy as np

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


def valid_name(name):
    """Names become file names, so only a conservative character set is accepted."""
    return bool(NAME_RE.match(name)) and name.lower() != 'unknown'


def _unit(v):
    v = np.asarray(v, dtype=np.float32).ravel()
    return v / (np.linalg.norm(v) + 1e-9)


class Gallery:
    def __init__(self, path):
        self.path = path
        self.names, self.vecs = [], []
        if path and os.path.isdir(path):
            for f in sorted(os.listdir(path)):
                if f.endswith('.npy') and valid_name(f[:-4]):
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
        """Persist and make the identity matchable immediately. Re-enrolling replaces."""
        if not valid_name(name):
            raise ValueError(f'invalid name {name!r}: use 1-40 of A-Z a-z 0-9 _ -')
        if not self.path:
            raise ValueError('gallery_dir parameter is empty; nowhere to store faces')
        os.makedirs(self.path, exist_ok=True)
        v = _unit(emb)
        np.save(os.path.join(self.path, f'{name}.npy'), v)
        if name in self.names:
            self.vecs[self.names.index(name)] = v
        else:
            self.names.append(name)
            self.vecs.append(v)
