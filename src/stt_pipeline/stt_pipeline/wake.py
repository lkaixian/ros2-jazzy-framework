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
