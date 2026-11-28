"""Pull a name or a favourite drink out of a spoken sentence (no LLM, no model)."""
import re

_NAME = [r"\b(?:my name is|my name's|name is)\s+([a-z][a-z'-]*)", r"\bcall me\s+([a-z][a-z'-]*)",
         r"\bi am\s+([a-z][a-z'-]*)", r"\bi'?m\s+([a-z][a-z'-]*)", r"\bit(?:'?s| is)\s+([a-z][a-z'-]*)",
         r"\bthis is\s+([a-z][a-z'-]*)"]
_DRINK = [r"\b(?:my )?favou?rite drink is\s+(.+)", r"\bi (?:would|'d) like\s+(.+)",
          r"\bi (?:really |just |usually |always |also |would )?(?:like|love|prefer|drink)\s+(.+)", r"\bi want\s+(.+)", r"\bbring me\s+(.+)"]
_FILLER = {'a', 'an', 'the', 'some', 'um', 'uh', 'please', 'hello', 'hi', 'hey', 'well', 'just', 'so', 'there',
           'you', 'thanks', 'thank', 'okay', 'ok', 'yes', 'yeah', 'sure'}


def _clean(text):
    return re.sub(r"[^a-z' \-]", ' ', text.lower())


def soundex(word):
    """Four-character Soundex key ('jane' and 'jain' -> J500), the usual cheap way to match a misheard name."""
    w = re.sub(r'[^a-z]', '', word.lower())
    if not w:
        return ''
    code = {c: d for d, cs in {'1': 'bfpv', '2': 'cgjkqsxz', '3': 'dt', '4': 'l', '5': 'mn', '6': 'r'}.items() for c in cs}
    out, last = w[0].upper(), code.get(w[0], '')
    for c in w[1:]:
        d = code.get(c, '')
        if d and d != last:
            out += d
        if c not in 'hw':
            last = d
    return (out + '000')[:4]


def snap_word(heard, choices, cutoff=0.8):
    """The list entry a misheard word most likely is ('jain' -> 'jane'), or '' when none is close: the characters match well (cutoff), or
    the Soundex keys match and at least 60% of the characters do. shortcut: spelling + Soundex only; no real phonetic model."""
    import difflib
    heard, best, best_r = heard.lower(), '', 0.0
    for c in choices:
        r = difflib.SequenceMatcher(None, heard, c.lower()).ratio()
        if (r >= cutoff or (r >= 0.6 and soundex(heard) == soundex(c))) and r > best_r:
            best, best_r = c, r
    return best


def extract_name(text, names=(), strict=False, snap=True):
    """'My name is Alice.' -> 'alice'; a lone word ('Bob') is taken as the name; '' if nothing usable.

    With a `names` list (the venue's): a heard word that is on it wins wherever it is in the sentence, a near miss is snapped to it
    ('jain' -> 'jane'), and with strict=True a name that is not close to any list entry is NOT accepted (the caller asks again: a wrong name
    costs points, asking again does not)."""
    t = _clean(text)
    if names:
        listed = {n.lower() for n in names}
        for w in t.split():
            if w in listed:
                return w
    cand = ''
    for pat in _NAME:
        m = re.search(pat, t)
        if m and m.group(1) not in _FILLER:
            cand = re.sub(r"[^a-z]", '', m.group(1))
            break
    if not cand:
        words = [w for w in t.split() if w not in _FILLER]
        cand = re.sub(r"[^a-z]", '', words[0]) if len(words) == 1 else ''
    if not cand or not names:
        return cand
    return (snap_word(cand, [n.lower() for n in names]) if snap else '') or ('' if strict else cand)


def extract_drink(text, drinks=(), snap=True):
    """'I like orange juice please.' -> 'orange juice'; '' if no drink phrase; a short bare phrase counts as the drink.
    With a `drinks` list a near miss is snapped to it ('orange juse' -> 'orange juice'); a phrase that is not on it is kept as heard."""
    t = ' '.join(_clean(text).split())
    for x in sorted((x.lower() for x in drinks), key=len, reverse=True):      # a listed drink named anywhere in the sentence wins ('i like to drink milk' -> milk)
        if re.search(rf'\b{re.escape(x)}\b', t):
            return x
    d = _extract_drink(text)
    return (snap_word(d, [x.lower() for x in drinks]) or d) if d and drinks and snap else d


def _extract_drink(text):
    t = ' '.join(w for w in _clean(text).split())
    for pat in _DRINK:
        m = re.search(pat, t)
        if m:
            words = [w for w in m.group(1).split() if w not in ('please', 'thanks', 'thank', 'you')]
            while words and (words[0] in _FILLER or words[0] in _LEAD):
                words.pop(0)
            return ' '.join(words[:4])
    words = [w for w in t.split() if w not in _FILLER]
    return ' '.join(words) if 0 < len(words) <= 3 else ''


_LEAD = {'to', 'drink', 'drinking', 'have', 'having', 'be', 'get', 'order', 'take'}     # 'i like to drink milk' -> milk, not 'to drink milk'
_YES = {'yes', 'yeah', 'yep', 'yup', 'yea', 'sure', 'correct', 'right', 'ok', 'okay', 'affirmative', 'definitely'}
_NO = {'no', 'nope', 'nah', 'wrong', 'incorrect', 'not', "don't", 'negative'}


def is_yes(text):
    """True for an affirmative answer ('Yes, that is right'); False for a no, an unclear answer, or both ('yes... no')."""
    words = set(_clean(text).split())
    return bool(words & _YES) and not words & _NO


def _words(s):
    return ' '.join(re.findall(r"[a-z0-9']+", (s or '').lower()))


def heard_any(text, words):
    """True when one of the phrases (a comma separated string or a list) is in the heard text as whole words, case-insensitive:
    'please stop now' hears 'stop', 'nonstop' does not."""
    if isinstance(words, str):
        words = words.split(',')
    heard = f' {_words(text)} '
    return any(_words(w) and f' {_words(w)} ' in heard for w in words)


_PLACES = {'table': ('table', 'desk', 'counter'), 'shelf': ('shelf', 'shelves', 'cabinet', 'cupboard'),
           'bin': ('bin', 'dustbin', 'trash', 'rubbish', 'garbage', 'waste')}


def extract_place(text):
    """'put it in the dustbin' -> 'bin'; 'on the table' -> 'table'; '' when no arm place is named or two different ones are."""
    heard = set(_words(text).split())
    hits = [place for place, words in _PLACES.items() if heard & set(words)]
    return hits[0] if len(hits) == 1 else ''


_ONES = {w: i for i, w in enumerate('one two three four five six seven eight nine ten eleven twelve thirteen fourteen fifteen sixteen '
                                    'seventeen eighteen nineteen'.split(), 1)}
_TENS = {w: 10 * i for i, w in enumerate('twenty thirty forty fifty sixty seventy eighty ninety'.split(), 2)}


def extract_age(text):
    """'I am twenty five years old' / 'I'm 25' -> '25'; '' when no age from 1 to 120 is in it."""
    t = _clean(text).replace('-', ' ')
    digits = re.search(r"(?<![a-z])(\d{1,3})(?![a-z])", text.lower())
    if digits and 1 <= int(digits.group(1)) <= 120:
        return str(int(digits.group(1)))
    words, i = t.split(), 0
    while i < len(words):
        w = words[i]
        if w in _TENS:
            n = _TENS[w] + (_ONES[words[i + 1]] if i + 1 < len(words) and words[i + 1] in _ONES and _ONES[words[i + 1]] < 10 else 0)
            return str(n)
        if w in _ONES:
            return str(_ONES[w])
        i += 1
    return ''


_MAN = {'man', 'male', 'boy', 'guy', 'gentleman', 'he', 'him'}
_WOMAN = {'woman', 'female', 'girl', 'lady', 'she', 'her'}


def extract_gender(text):
    """'I am a woman' -> 'woman', 'male' -> 'man'; '' when neither word (or both) is heard."""
    words = set(_clean(text).replace('-', ' ').split())
    man, woman = bool(words & _MAN), bool(words & _WOMAN)
    return 'man' if man and not woman else 'woman' if woman and not man else ''
