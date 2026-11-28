"""The venue's name and drink lists (param/vocab.yaml), for snapping what was heard to what is plausible. No ROS imports besides ament lookup."""
import os

import yaml


def load_vocab(path=''):
    """{'names': [...], 'drinks': [...], 'strict_names': bool}; the framework_bringup file when no path is given; empty lists if unreadable.
    (stt_pipeline.vocab.load_vocab reads the same file for the Whisper prompt: the two packages do not depend on each other.)"""
    empty = {'names': [], 'drinks': [], 'strict_names': False, 'snap_names': True, 'snap_drinks': True}
    try:
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('framework_bringup'), 'param', 'vocab.yaml')
        d = yaml.safe_load(open(path)) or {}
        return {'names': [str(n) for n in d.get('names') or []], 'drinks': [str(n) for n in d.get('drinks') or []],
                'strict_names': bool(d.get('strict_names', False)), 'snap_names': bool(d.get('snap_names', True)), 'snap_drinks': bool(d.get('snap_drinks', True))}
    except Exception:  # noqa: BLE001 - lists are optional: no file, plain extraction
        return empty
