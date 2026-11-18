import pytest

from tts_pipeline import backends
from tts_pipeline.plan_announcer_node import describe


class P:
    def __init__(self, valid, errors=()):
        self.valid, self.errors = valid, list(errors)


def test_espeak_cmd_protects_text():
    cmd = backends.espeak_cmd('-rm -rf; hi', 'en-us', 150)
    assert cmd[-2:] == ['--', '-rm -rf; hi'] and '-v' in cmd and cmd[cmd.index('-s') + 1] == '150'


def test_piper_requires_model():
    with pytest.raises(backends.TtsError):
        backends.piper_cmd('', 22050)


def test_describe():
    assert describe(P(True)) == 'Plan ready.'
    s = describe(P(False, ['a', 'b', 'c']), True, 2)
    assert 'a b' in s and 'c' not in s.replace('could', '')
    assert describe(P(False, ['a']), False) == 'I could not make a valid plan.'


def test_pcm_conversion_clips_and_scales():
    import numpy as np
    pcm = backends.pcm16_from_float([0.0, 1.0, -1.0, 5.0, -5.0])
    assert list(np.frombuffer(pcm, '<i2')) == [0, 32767, -32767, 32767, -32767]


def test_aplay_cmd():
    assert backends.aplay_cmd(24000, 'hw:1')[-2:] == ['-D', 'hw:1']
    assert '24000' in backends.aplay_cmd(24000)


def test_kokoro_missing_is_loud():
    with pytest.raises(backends.TtsError):
        backends.Kokoro('m.onnx', 'v.bin')     # kokoro-onnx not installed here


def test_piper_launch_can_be_communicated_with(monkeypatch):
    """Regression: _launch closed stdin and communicate() then raised "flush of closed file" (every Piper utterance failed)."""
    import rclpy
    from tts_pipeline.tts_node import TtsNode
    monkeypatch.setattr(backends, 'piper_cmd', lambda *a: 'cat > /dev/null')     # stands in for `piper | aplay`
    rclpy.init(args=['--ros-args', '-p', 'engine:=piper', '-p', 'piper_model:=m.onnx'])
    try:
        node = TtsNode()
        proc = node._launch('hello', '')
        proc.communicate(timeout=10)
        assert proc.returncode == 0
        node.destroy_node()
    finally:
        rclpy.shutdown()
