"""Energy-based VAD + utterance segmenter (pure Python/numpy, ROS-free)."""
from collections import deque

import numpy as np


def rms(frame: bytes) -> float:
    a = np.frombuffer(frame, dtype='<i2').astype(np.float64)
    return float(np.sqrt(np.mean(a * a))) if a.size else 0.0


class Segmenter:
    """Feed arbitrary-size PCM16 mono chunks; get complete utterances back."""

    def __init__(self, rate=16000, frame_ms=30, threshold_rms=500.0, noise_ratio=3.0,
                 min_speech_ms=150, hangover_ms=600, preroll_ms=300, max_utt_sec=15.0):
        self.rate = rate
        self.fbytes = rate * frame_ms // 1000 * 2
        self.fms = frame_ms
        self.thr, self.ratio = threshold_rms, noise_ratio
        self.min_frames = max(1, min_speech_ms // frame_ms)
        self.hang_frames = max(1, hangover_ms // frame_ms)
        self.preroll = deque(maxlen=max(1, preroll_ms // frame_ms))
        self.max_frames = int(max_utt_sec * 1000 // frame_ms)
        self.noise = None          # adaptive noise floor (RMS), tracks non-speech frames
        self.buf = b''
        self.reset()

    def reset(self):
        self.in_speech = False
        self.utt = []
        self.run = 0               # consecutive speech frames while idle
        self.quiet = 0             # consecutive silent frames while in speech
        self.pending = []          # speech frames seen before min_speech reached

    def _is_speech(self, level):
        floor = self.noise if self.noise is not None else 0.0
        return level > max(self.thr, floor * self.ratio)

    def feed(self, pcm: bytes):
        out = []
        self.buf += pcm
        while len(self.buf) >= self.fbytes:
            frame, self.buf = self.buf[:self.fbytes], self.buf[self.fbytes:]
            u = self._step(frame)
            if u is not None:
                out.append(u)
        return out

    def _step(self, frame):
        level = rms(frame)
        speech = self._is_speech(level)
        if not speech:   # update the noise floor only from non-speech
            self.noise = level if self.noise is None else 0.95 * self.noise + 0.05 * level
        if not self.in_speech:
            if speech:
                self.run += 1
                self.pending.append(frame)
                if self.run >= self.min_frames:
                    self.in_speech = True
                    self.utt = list(self.preroll)[:-len(self.pending) + 0 or None] \
                        if False else list(self.preroll) + self.pending
                    self.pending, self.quiet = [], 0
            else:
                self.run, self.pending = 0, []
                self.preroll.append(frame)
            return None
        self.utt.append(frame)
        self.quiet = 0 if speech else self.quiet + 1
        if self.quiet >= self.hang_frames or len(self.utt) >= self.max_frames:
            done = b''.join(self.utt)
            self.preroll.clear()
            self.reset()
            return done
        return None
