import numpy as np
import pytest

from stt_pipeline import engines
from stt_pipeline.vad import Segmenter
from stt_pipeline.wake import WakeGate

RATE = 16000


def tone(ms, amp=8000):
    t = np.arange(RATE * ms // 1000) / RATE
    return (amp * np.sin(2 * np.pi * 220 * t)).astype('<i2').tobytes()


def silence(ms):
    return np.zeros(RATE * ms // 1000, dtype='<i2').tobytes()


def run(seg, audio, chunk=1000):
    out = []
    for i in range(0, len(audio), chunk):
        out += seg.feed(audio[i:i + chunk])
    return out


def test_single_utterance_with_preroll():
    utts = run(Segmenter(RATE), silence(500) + tone(800) + silence(1200))
    assert len(utts) == 1
    ms = len(utts[0]) * 1000 // (2 * RATE)
    assert 800 <= ms <= 800 + 600 + 300 + 60      # speech + hangover + preroll


def test_two_utterances():
    a = silence(300) + tone(500) + silence(900) + tone(500) + silence(900)
    assert len(run(Segmenter(RATE), a)) == 2


def test_short_blip_ignored():
    assert run(Segmenter(RATE), silence(300) + tone(60) + silence(1500)) == []


def test_pure_silence_and_low_noise():
    noise = (np.random.default_rng(0).normal(0, 50, RATE * 2)).astype('<i2').tobytes()
    assert run(Segmenter(RATE), silence(1000) + noise) == []


def test_max_length_cut():
    utts = run(Segmenter(RATE, max_utt_sec=1.0), tone(3000))
    assert len(utts) >= 2


def test_odd_chunk_sizes_equal_result():
    a = silence(300) + tone(700) + silence(1000)
    assert [len(u) for u in run(Segmenter(RATE), a, 777)] == \
           [len(u) for u in run(Segmenter(RATE), a, 4096)]


def test_wake_gate_disabled_passes_everything():
    assert WakeGate('').task(' pick the cup ') == 'pick the cup'


def test_wake_gate_inline_and_armed():
    now = [0.0]
    g = WakeGate('Robot', 5.0, clock=lambda: now[0])
    assert g.task('pick the cup') is None
    assert g.task('Robot, pick the cup!') == 'pick the cup'
    assert g.task('robot') is None
    now[0] = 3.0
    assert g.task('go left') == 'go left'
    assert g.task('go right') is None                 # armed only once
    g.task('robot'); now[0] = 20.0
    assert g.task('late') is None                     # window expired


def test_engine_errors_are_loud():
    with pytest.raises(engines.SttError):
        engines.create('nope')
    with pytest.raises(engines.SttError):
        engines.create('vosk', model='/x')            # vosk not installed here


def test_mock_engine():
    assert engines.create('mock').transcribe(tone(500), RATE)[0].startswith('[mock')
