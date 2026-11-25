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


def _preload_cuda12():
    """ctranslate2 wants CUDA 12 cuBLAS/cuDNN; pip's nvidia-cublas-cu12 / nvidia-cudnn-cu12 keep them off the loader path.
    Load them from there so no LD_LIBRARY_PATH export is needed. Silent when absent (CPU, or the system has them)."""
    import ctypes
    import glob
    import importlib.util
    spec = importlib.util.find_spec('nvidia')
    for root in (spec.submodule_search_locations if spec and spec.submodule_search_locations else []):
        for pat in ('cublas/lib/libcublas.so.12', 'cudnn/lib/libcudnn.so.9'):
            for lib in glob.glob(f'{root}/{pat}'):
                try:
                    ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass


class FasterWhisper:
    name = 'faster_whisper'

    def __init__(self, model, device='cpu', compute_type='int8', language='', initial_prompt='', **_):
        try:
            from faster_whisper import WhisperModel
        except ImportError as e:
            raise SttError('faster-whisper not installed: pip install faster-whisper') from e
        _preload_cuda12()
        try:
            self.model = self._load(WhisperModel, model, device, compute_type)
        except Exception as e:  # noqa: BLE001  bad name/path, no network, wrong compute type...
            if device == 'cpu':
                raise SttError(f'faster-whisper could not load {model!r}: {e}') from e
            try:    # a GPU choice made on another computer (no CUDA, missing cuBLAS/cuDNN): run on CPU instead of crashing
                print(f'faster-whisper: {device} unusable ({e}); falling back to cpu/int8', flush=True)
                self.model = self._load(WhisperModel, model, 'cpu', 'int8')
            except Exception as e2:  # noqa: BLE001
                raise SttError(f'faster-whisper could not load {model!r}: {e2}') from e2
        self.language = language or None
        self.initial_prompt = initial_prompt or None

    @staticmethod
    def _load(WhisperModel, model, device, compute_type):
        m = WhisperModel(model, device=device, compute_type=compute_type)
        if device != 'cpu':     # missing CUDA libraries only fail at the first encode, so run one now
            import numpy as np
            list(m.transcribe(np.zeros(16000, dtype=np.float32), vad_filter=False)[0])
        return m

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
