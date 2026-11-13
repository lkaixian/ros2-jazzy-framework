from audio_capture.audio_capture_node import arecord_cmd


def test_cmd_default_device():
    assert arecord_cmd('', 16000, 1)[-2:] == ['-c', '1']


def test_cmd_device():
    c = arecord_cmd('plughw:2,0', 16000, 1)
    assert c[-2:] == ['-D', 'plughw:2,0'] and '16000' in c
