"""Tonal-sound (doorbell) detector on 16-bit PCM: a steady narrowband tone, not speech or noise.

Heuristic, tuned on synthetic tones and one synthesised speech clip only (no real doorbell recordings yet).
A window is "tonal" when one FFT peak holds >= `peak_ratio` of the 300-5000 Hz power and is loud enough; an event fires
when `need` consecutive tonal windows agree on the peak frequency within `tol` (speech pitch drifts, a bell does not).
"""
import numpy as np

LO, HI = 300.0, 5000.0


def tonal_peak(pcm, rate, peak_ratio=0.5, min_rms=300.0):
    """One window of int16 samples -> peak frequency in Hz if it is a clean loud tone, else None."""
    x = np.asarray(pcm, dtype=np.float64)
    if x.size < 256 or np.sqrt(np.mean(x * x)) < min_rms:
        return None
    spec = np.abs(np.fft.rfft(x * np.hanning(x.size))) ** 2
    freqs = np.fft.rfftfreq(x.size, 1.0 / rate)
    band = (freqs >= LO) & (freqs <= HI)
    if not band.any():
        return None
    i = int(np.argmax(np.where(band, spec, 0.0)))
    near = np.abs(freqs - freqs[i]) <= 2 * rate / x.size * 2     # the peak and its window-leakage neighbours
    return float(freqs[i]) if spec[near & band].sum() >= peak_ratio * spec[band].sum() else None


class ToneDetector:
    def __init__(self, rate, window_sec=0.12, need=3, tol=0.04, peak_ratio=0.5, min_rms=300.0, refractory_sec=2.0):
        self.rate, self.win, self.need, self.tol = rate, int(rate * window_sec), need, tol
        self.peak_ratio, self.min_rms, self.refractory = peak_ratio, min_rms, refractory_sec
        self.buf, self.run, self.t, self.last = np.zeros(0, dtype=np.int16), [], 0.0, -1e9

    def feed(self, pcm):
        """pcm: int16 array. Returns a list of (time_sec, freq_hz) events found in this chunk."""
        self.buf = np.concatenate([self.buf, np.asarray(pcm, dtype=np.int16)])
        out = []
        while self.buf.size >= self.win:
            w, self.buf = self.buf[:self.win], self.buf[self.win:]
            self.t += self.win / self.rate
            f = tonal_peak(w, self.rate, self.peak_ratio, self.min_rms)
            if f is None:
                self.run = []
                continue
            self.run = (self.run if self.run and abs(f - self.run[-1]) <= self.tol * self.run[-1] else []) + [f]
            if len(self.run) >= self.need and self.t - self.last >= self.refractory:
                out.append((self.t, float(np.mean(self.run))))
                self.last = self.t
        return out
