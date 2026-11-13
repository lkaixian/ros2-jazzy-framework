"""
Fine-tune a YOLO11 detector on your own items, with defaults sized for a 4 GB GPU.

  ros2 run object_detection train_detector --data my_items.yaml --epochs 100

Then use the result: set `model:` of object_detection_node in param/perception.yaml to the
printed best.pt. The node announces the model's own class names, so the BT validator and the
LLM only use labels the detector knows.

my_items.yaml is the standard Ultralytics dataset file (path/train/val + names). Boxes only
(detection), no segmentation: that is what keeps VRAM use low.
"""
import argparse
import os


def build_args(a):
    """Return the Ultralytics train() keyword arguments (kept separate so it can be tested)."""
    return {
        'data': a.data, 'epochs': a.epochs, 'imgsz': a.imgsz, 'batch': a.batch,
        'device': a.device or None, 'workers': a.workers, 'patience': a.patience,
        'project': a.project, 'name': a.name,
        'amp': True, 'cache': False, 'exist_ok': True, 'plots': False, 'seed': 0,
    }


def check_environment(import_module=None):
    """
    Explain the NumPy/matplotlib conflict instead of crashing deep inside Ultralytics.

    On this setup the apt matplotlib is built for NumPy 1 while pip's NumPy is 2.x, which makes
    the import that training needs crash.
    """
    import importlib
    try:
        (import_module or importlib.import_module)('matplotlib.font_manager')
    except Exception as e:  # noqa: BLE001 - any import-time failure means training will die
        raise SystemExit(
            f'matplotlib cannot be imported ({type(e).__name__}: {str(e).splitlines()[0]}).\n'
            'Usual cause: pip installed NumPy 2.x but the apt matplotlib is built for NumPy 1.x.\n'
            'Fix: /usr/bin/python3 -m pip install --user --break-system-packages '
            '"matplotlib>=3.9"')


def main(argv=None):
    """Run the fine-tune; see the module docstring."""
    ap = argparse.ArgumentParser(description='Fine-tune a YOLO11 detector (boxes only).')
    ap.add_argument('--data', required=True, help='Ultralytics dataset yaml')
    ap.add_argument('--base', default='yolo11n.pt',
                    help='weights to start from (n/s fit 4 GB)')
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=8,
                    help='lower it on CUDA out-of-memory (-1 = auto)')
    ap.add_argument('--device', default='', help='"" = auto, 0 = first GPU, cpu')
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--patience', type=int, default=30,
                    help='early stop after N epochs without improvement')
    ap.add_argument('--project', default='models/detectors')
    ap.add_argument('--name', default='items')
    a = ap.parse_args(argv)
    if not os.path.isfile(a.data):
        raise SystemExit(f'dataset yaml not found: {a.data}')
    check_environment()
    from ultralytics import YOLO
    kwargs = build_args(a)
    model = YOLO(a.base)
    task = getattr(model, 'task', 'detect')
    if 'seg' in os.path.basename(a.base) or task != 'detect':
        raise SystemExit(f'{a.base} is a {task} model; use a plain detect model (e.g. '
                         'yolo11n.pt): segmentation costs too much VRAM here')
    model.train(**kwargs)
    best = os.path.join(a.project, a.name, 'weights', 'best.pt')
    print(f'\nbest weights: {best}\nuse: model: {os.path.abspath(best)}  in param/perception.yaml')


if __name__ == '__main__':
    main()
