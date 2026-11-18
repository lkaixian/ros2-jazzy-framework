"""Pull a name or a favourite drink out of a spoken sentence (no LLM, no model)."""
import re

_NAME = [r"\b(?:my name is|my name's|name is)\s+([a-z][a-z'-]*)", r"\bcall me\s+([a-z][a-z'-]*)",
         r"\bi am\s+([a-z][a-z'-]*)", r"\bi'?m\s+([a-z][a-z'-]*)", r"\bit(?:'?s| is)\s+([a-z][a-z'-]*)",
         r"\bthis is\s+([a-z][a-z'-]*)"]
_DRINK = [r"\b(?:my )?favou?rite drink is\s+(.+)", r"\bi (?:would|'d) like\s+(.+)",
          r"\bi (?:like|love|prefer|drink)\s+(.+)", r"\bi want\s+(.+)", r"\bbring me\s+(.+)"]
_FILLER = {'a', 'an', 'the', 'some', 'um', 'uh', 'please', 'hello', 'hi', 'hey', 'well', 'just', 'so', 'there',
           'you', 'thanks', 'thank', 'okay', 'ok', 'yes', 'yeah', 'sure'}


def _clean(text):
    return re.sub(r"[^a-z' \-]", ' ', text.lower())


def extract_name(text):
    """'My name is Alice.' -> 'alice'; a lone word ('Bob') is taken as the name; '' if nothing usable."""
    t = _clean(text)
    for pat in _NAME:
        m = re.search(pat, t)
        if m and m.group(1) not in _FILLER:
            return re.sub(r"[^a-z]", '', m.group(1))
    words = [w for w in t.split() if w not in _FILLER]
    return re.sub(r"[^a-z]", '', words[0]) if len(words) == 1 else ''


def extract_drink(text):
    """'I like orange juice please.' -> 'orange juice'; '' if no drink phrase; a short bare phrase counts as the drink."""
    t = ' '.join(w for w in _clean(text).split())
    for pat in _DRINK:
        m = re.search(pat, t)
        if m:
            words = [w for w in m.group(1).split() if w not in ('please', 'thanks', 'thank', 'you')]
            while words and words[0] in _FILLER:
                words.pop(0)
            return ' '.join(words[:4])
    words = [w for w in t.split() if w not in _FILLER]
    return ' '.join(words) if 0 < len(words) <= 3 else ''


_YES = {'yes', 'yeah', 'yep', 'yup', 'yea', 'sure', 'correct', 'right', 'ok', 'okay', 'affirmative', 'definitely'}
_NO = {'no', 'nope', 'nah', 'wrong', 'incorrect', 'not', "don't", 'negative'}


def is_yes(text):
    """True for an affirmative answer ('Yes, that is right'); False for a no, an unclear answer, or both ('yes... no')."""
    words = set(_clean(text).split())
    return bool(words & _YES) and not words & _NO
