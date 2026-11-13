"""TTS backends build a command; the node owns the process (so it can be interrupted)."""
import shlex
import shutil


class TtsError(RuntimeError):
    pass


def espeak_cmd(text, voice='', rate=160):
    exe = shutil.which('espeak-ng') or shutil.which('espeak')
    if not exe:
        raise TtsError('neither espeak-ng nor espeak found')
    cmd = [exe, '-s', str(int(rate))]
    if voice:
        cmd += ['-v', voice]
    return cmd + ['--', text]


def aplay_cmd(rate, device=''):
    cmd = ['aplay', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', str(int(rate)), '-c', '1']
    return cmd + (['-D', device] if device else [])


def pcm16_from_float(samples):
    """float32 [-1, 1] -> little-endian int16 bytes (clipped)."""
    import numpy as np
    a = np.clip(np.asarray(samples, dtype=np.float32), -1.0, 1.0)
    return (a * 32767.0).astype('<i2').tobytes()


class Kokoro:
    """Kokoro-82M (open weights) through kokoro-onnx; synthesis is in-process."""

    def __init__(self, model, voices):
        try:
            from kokoro_onnx import Kokoro as _K
        except ImportError as e:
            raise TtsError('kokoro-onnx not installed: pip install kokoro-onnx') from e
        if not model or not voices:
            raise TtsError('kokoro_model and kokoro_voices parameters are required')
        try:
            self._k = _K(model, voices)
        except Exception as e:  # noqa: BLE001  onnxruntime raises NoSuchFile etc.
            raise TtsError(f'kokoro could not load {model!r} / {voices!r}: {e}') from e

    def synth(self, text, voice, speed=1.0, lang='en-us'):
        """Returns (int16 PCM bytes, sample_rate)."""
        if not voice:
            raise TtsError('set the `voice` parameter to a Kokoro voice name')
        samples, rate = self._k.create(text, voice=voice, speed=speed, lang=lang)
        return pcm16_from_float(samples), int(rate)


def piper_cmd(model, rate, device=''):
    """Returns a shell pipeline string; text is fed on stdin."""
    piper = shutil.which('piper')
    if not piper:
        raise TtsError('piper not found: pip install piper-tts')
    if not model:
        raise TtsError('piper_model parameter is empty')
    aplay = aplay_cmd(rate, device)
    return (f'{shlex.quote(piper)} --model {shlex.quote(model)} --output_raw | '
            + ' '.join(shlex.quote(a) for a in aplay))
