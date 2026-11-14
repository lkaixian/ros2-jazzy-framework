import os

import numpy as np
import pytest

from stt_pipeline.sound import ToneDetector

RATE = 16000


def tone(freq, sec, amp=8000):
    t = np.arange(int(RATE * sec)) / RATE
    return (amp * np.sin(2 * np.pi * freq * t)).astype(np.int16)


def events(pcm):
    d, out = ToneDetector(RATE), []
    for i in range(0, len(pcm), 1600):          # 0.1 s chunks like audio_capture
        out += d.feed(pcm[i:i + 1600])
    return out


def test_ding_dong_is_detected():
    pcm = np.concatenate([tone(880, 0.5), tone(660, 0.6), np.zeros(RATE, dtype=np.int16)])
    ev = events(pcm)
    assert len(ev) >= 1 and 600 < ev[0][1] < 950


def test_white_noise_and_quiet_tone_are_not_events():
    rng = np.random.default_rng(0)
    assert events((rng.normal(0, 3000, RATE * 2)).astype(np.int16)) == []
    assert events(tone(880, 1.0, amp=100)) == []                    # too quiet


def test_short_blip_and_wandering_pitch_are_not_events():
    assert events(tone(880, 0.15)) == []                            # under 3 windows
    glide = np.concatenate([tone(f, 0.12) for f in range(400, 1400, 120)])
    assert events(glide) == []                                      # pitch moves like speech


def test_one_ring_is_one_event():
    assert len(events(np.concatenate([tone(880, 1.0), np.zeros(RATE, dtype=np.int16)]))) == 1


SPEECH = '/tmp/robocup_speech_sample.wav'


@pytest.mark.skipif(not os.path.exists(SPEECH), reason='needs a speech WAV (16 kHz mono) at ' + SPEECH)
def test_speech_is_not_a_doorbell():
    import soundfile as sf
    d, sr = sf.read(SPEECH, dtype='int16')
    assert sr == 16000 and events(d) == []
