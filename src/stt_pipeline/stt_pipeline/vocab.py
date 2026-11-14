"""Vocabulary help for speech recognition: place names as hints, and snapping near-miss words to them (no ROS imports)."""
import difflib
import os

import yaml


def place_phrases(path=''):
    """['far corner', 'left room', ...] from a places YAML; the framework_bringup one when no path is given; [] if unreadable."""
    try:
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('framework_bringup'), 'param', 'places.yaml')
        return sorted(n.replace('_', ' ') for n in yaml.safe_load(open(path))['places'])
    except Exception:  # noqa: BLE001 - hints are optional: no file, no hints
        return []


def hint_prompt(phrases):
    """A Whisper `initial_prompt`: the words it should expect, as a sentence it can continue."""
    return f'Go to the {", ".join(phrases)}.' if phrases else ''


def snap(text, phrases, cutoff=0.8):
    """Replace 1-3 word windows that nearly match a phrase ('for corner' -> 'far corner'); exact words and short words stay.

    shortcut: character similarity only, no phonetics; raise `cutoff` if it rewrites real words, lower it for a noisier microphone.
    """
    words = text.split()
    out, i = [], 0
    while i < len(words):
        for n in (3, 2, 1):
            window = ' '.join(words[i:i + n])
            core = window.strip('.,!?').lower()
            if len(words[i:i + n]) < n or len(core) < 4:
                continue
            best = difflib.get_close_matches(core, phrases, n=1, cutoff=cutoff)
            if best:
                out.append(best[0] if best[0] != core else window)
                i += n
                break
        else:
            out.append(words[i])
            i += 1
    return ' '.join(out)
