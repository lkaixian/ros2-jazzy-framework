from audio_capture.audio_capture_node import arecord_cmd


def test_cmd_default_device():
    assert arecord_cmd('', 16000, 1)[-2:] == ['-c', '1']


def test_cmd_device():
    c = arecord_cmd('plughw:2,0', 16000, 1)
    assert c[-2:] == ['-D', 'plughw:2,0'] and '16000' in c


def test_remove_dc_centres_a_biased_signal():
    import numpy as np
    from audio_capture.audio_capture_node import remove_dc
    x = (7400 + 300 * np.sin(np.arange(1600) / 5)).astype('<i2').tobytes()
    out, dc = remove_dc(x, None, 1)
    y = np.frombuffer(out, '<i2')
    assert abs(float(dc[0]) - 7400) < 50 and abs(float(y.mean())) < 50 and 150 < y.std() < 250
    assert len(remove_dc(x[:-1], dc, 1)[0]) == len(x) - 2      # a torn last sample is dropped, not crashed on
