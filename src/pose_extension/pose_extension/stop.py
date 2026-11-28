"""When may the followed person stop the robot? A voice or hand signal counts only while that person is standing still. ROS-free.

Anyone in the room can say "stop", and the operator may wave at a crowd while walking: neither is an order. The robot stops for the word "stop" (or one of the given
phrases) or for a raised hand held up for `sign_sec`, but only when the tracked operator has hardly moved for `still_sec`. Without a known operator the voice stop
is accepted (a robot that cannot be stopped is worse).
shortcut: the voice cannot be tied to a speaker; the hand sign is "a hand held above the shoulder", not five counted fingers (that needs a hand-landmark model).
"""
import math
import re
from collections import deque

L_SH, R_SH, L_WR, R_WR = 5, 6, 9, 10


def hand_raised(kps, min_conf=0.4):
    """A wrist clearly above its shoulder (more than half a shoulder width) and not far out to the side: an open hand held up. kps: [(x, y, conf)] COCO-17."""
    if len(kps) <= R_WR or min(kps[L_SH][2], kps[R_SH][2]) < min_conf:
        return False
    sw = math.hypot(kps[L_SH][0] - kps[R_SH][0], kps[L_SH][1] - kps[R_SH][1])
    if sw < 1e-3:
        return False
    for sh, wr in ((L_SH, L_WR), (R_SH, R_WR)):
        if kps[wr][2] >= min_conf and kps[sh][1] - kps[wr][1] > 0.5 * sw and abs(kps[wr][0] - kps[sh][0]) < 1.5 * sw:
            return True
    return False


class StopGate:
    def __init__(self, words=('stop',), still_sec=1.5, still_m=0.3, sign_sec=1.0, voice_window=2.0):
        self.words, self.still_sec, self.still_m, self.sign_sec, self.voice_window = list(words), still_sec, still_m, sign_sec, voice_window
        self.pos, self.raised, self.voice_t, self.ignored = {}, {}, None, 0

    def see(self, pid, pos, kps, now):
        """One frame of one person: pos = (x, y) in the odom frame or None; kps = their keypoints (may be empty)."""
        if pos is not None:
            h = self.pos.setdefault(pid, deque())
            h.append((now, pos[0], pos[1]))
            while h and now - h[0][0] > 2 * self.still_sec + 1.0:
                h.popleft()
        if kps and hand_raised(kps):
            self.raised.setdefault(pid, now)
        else:
            self.raised.pop(pid, None)

    def heard(self, text, now):
        t = re.sub(r"[^a-z' ]", ' ', text.lower())
        if any(re.search(rf'\b{re.escape(w.strip().lower())}\b', t) for w in self.words if w.strip()):
            self.voice_t = now

    def still(self, pid, now):
        """True when the person's position over the last `still_sec` stayed within `still_m`; False when it moved or there is not enough history."""
        h = [p for p in self.pos.get(pid, ()) if now - p[0] <= self.still_sec]
        if len(h) < 3 or now - h[0][0] < 0.7 * self.still_sec:
            return False
        xs, ys = [p[1] for p in h], [p[2] for p in h]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys)) <= self.still_m

    def check(self, pid, now):
        """'voice', 'sign', or '' (keep going). pid = the tracked operator (None = unknown)."""
        heard = self.voice_t is not None and now - self.voice_t <= self.voice_window
        if heard and (pid is None or self.still(pid, now)):
            return 'voice'
        if heard:
            self.ignored += 1            # said while the operator was walking: not an order (counted, so a test or log can see it)
            self.voice_t = None
        if pid is not None and pid in self.raised and now - self.raised[pid] >= self.sign_sec and self.still(pid, now):
            return 'sign'
        return ''
