"""Wake-word gating for transcripts (ROS-free)."""
import re
import time


def _norm(s):
    return re.sub(r'[^\w\s]', '', s.lower()).strip()


class WakeGate:
    def __init__(self, wake_word='', window_sec=8.0, clock=time.monotonic):
        self.w, self.win, self.clock = _norm(wake_word), window_sec, clock
        self.armed_until = float('-inf')

    def task(self, text):
        """Return the task text to forward, or None to ignore."""
        t = _norm(text)
        if not self.w:
            return text.strip() or None
        if t == self.w:                       # bare wake word arms the next utterance
            self.armed_until = self.clock() + self.win
            return None
        m = re.match(rf'^{re.escape(self.w)}\b[\s,]*(.*)$', t)
        if m and m.group(1):
            return m.group(1)
        if self.clock() <= self.armed_until:
            self.armed_until = float('-inf')
            return t or None
        return None


class StartGate:
    """The spoken signal that starts a task ("start the task"): True only when every word of a phrase is heard, in order.

    Each word must match a heard word almost exactly (short words exactly), so "stop the task" never starts anything.
    shortcut: character similarity per word, no phonetics; add the phrases the recogniser really produces ("start a task").
    """

    def __init__(self, phrases, cutoff=0.8):
        import difflib
        self._sim = lambda a, b: difflib.SequenceMatcher(None, a, b).ratio()
        self.phrases = [_norm(p).split() for p in phrases if _norm(p)]
        self.cutoff = cutoff

    def _word(self, want, heard):
        return want == heard or (len(want) > 3 and self._sim(want, heard) >= self.cutoff)

    def heard(self, text):
        words = _norm(text).split()
        for phrase in self.phrases:
            i = 0
            for w in words:
                if i < len(phrase) and self._word(phrase[i], w):
                    i += 1
            if i == len(phrase):
                return True
        return False
