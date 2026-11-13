from object_detection import train

import pytest


def parse(argv):
    import argparse  # noqa: F401
    captured = {}

    class A:
        pass
    a = A()
    a.data, a.epochs, a.imgsz, a.batch, a.device, a.workers = 'd.yaml', 5, 640, 8, '', 2
    a.patience, a.project, a.name = 30, 'models/detectors', 'items'
    captured.update(train.build_args(a))
    return captured


def test_defaults_are_small_gpu_friendly():
    k = parse([])
    assert k['batch'] == 8 and k['imgsz'] == 640 and k['amp'] is True and k['cache'] is False
    assert k['workers'] == 2 and k['device'] is None


def test_missing_dataset_is_a_clear_error(tmp_path):
    with pytest.raises(SystemExit) as e:
        train.main(['--data', str(tmp_path / 'nope.yaml')])
    assert 'not found' in str(e.value)


def test_preflight_explains_the_numpy_matplotlib_conflict():
    def broken(_name):
        raise ImportError('numpy.core.multiarray failed to import')
    with pytest.raises(SystemExit) as e:
        train.check_environment(broken)
    msg = str(e.value)
    assert 'NumPy' in msg and 'matplotlib>=3.9' in msg and 'multiarray' in msg


def test_preflight_passes_when_import_works():
    train.check_environment(lambda _n: object())
