"""STT backends. Third-party engines are imported lazily; a missing one is an error."""
import json


class SttError(RuntimeError):
    pass


class Mock:
    """Deterministic stand-in for plumbing tests (does NOT recognise speech)."""
    name = 'mock'

    def __init__(self, **_):
        pass

    def transcribe(self, pcm, rate):
        ms = len(pcm) * 1000 // (2 * rate)
        return f'[mock utterance {ms} ms]', '', -1.0


class Vosk:
    name = 'vosk'

    def __init__(self, model, **_):
        try:
            import vosk
        except ImportError as e:
            raise SttError('vosk not installed: pip install vosk') from e
        self._vosk = vosk
        vosk.SetLogLevel(-1)
        try:
            self.model = vosk.Model(model)
        except Exception as e:  # noqa: BLE001  vosk raises a bare Exception for a bad path
            raise SttError(f'vosk could not load model {model!r}: {e}') from e

    def transcribe(self, pcm, rate):
        rec = self._vosk.KaldiRecognizer(self.model, rate)
        rec.SetWords(True)
        rec.AcceptWaveform(pcm)
        res = json.loads(rec.FinalResult())
        words = res.get('result', [])
        conf = sum(w['conf'] for w in words) / len(words) if words else -1.0
        return res.get('text', ''), '', conf


class FasterWhisper:
    name = 'faster_whisper'

    def __init__(self, model, device='cpu', compute_type='int8', language='', initial_prompt='', **_):
        try:
            from faster_whisper import WhisperModel
        except ImportError as e:
            raise SttError('faster-whisper not installed: pip install faster-whisper') from e
        try:
            self.model = WhisperModel(model, device=device, compute_type=compute_type)
        except Exception as e:  # noqa: BLE001  bad name/path, no network, wrong compute type...
            raise SttError(f'faster-whisper could not load {model!r}: {e}') from e
        self.language = language or None
        self.initial_prompt = initial_prompt or None

    def transcribe(self, pcm, rate):
        import numpy as np
        audio = np.frombuffer(pcm, dtype='<i2').astype(np.float32) / 32768.0
        if rate != 16000:
            raise SttError('faster_whisper expects 16 kHz audio')
        segs, info = self.model.transcribe(audio, language=self.language, vad_filter=False,
                                         initial_prompt=self.initial_prompt)
        segs = list(segs)
        text = ' '.join(s.text.strip() for s in segs).strip()
        conf = (sum(2.718281828 ** s.avg_logprob for s in segs) / len(segs)) if segs else -1.0
        return text, info.language, conf


ENGINES = {'mock': Mock, 'vosk': Vosk, 'faster_whisper': FasterWhisper}


def create(engine, **kw):
    if engine not in ENGINES:
        raise SttError(f'unknown engine {engine!r}; choose from {sorted(ENGINES)}')
    return ENGINES[engine](**kw)
