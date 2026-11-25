"""Which detected object to handle next and where it goes (param/object_categories.yaml). Pure, no ROS."""
import yaml


def load_table(path):
    """{task: {label: {group, nav, put}}}; raises ValueError with a readable message on a malformed file."""
    try:
        data = yaml.safe_load(open(path)) or {}
    except (OSError, yaml.YAMLError) as e:
        raise ValueError(f'cannot read {path}: {e}') from None
    for task, rows in data.items():
        for label, e in (rows or {}).items():
            if not isinstance(e, dict) or not all(isinstance(e.get(k), str) and e.get(k) for k in ('group', 'nav', 'put')):
                raise ValueError(f'{path}: {task}/{label} needs group, nav and put')
    return data


def pick_next(objects, rows, last_label='', min_conf=0.5):
    """(detection, entry) for the biggest confident detection whose label is in `rows`, preferring a label other than `last_label` (so an
    object that could not be picked does not block the others); (None, None) when nothing handleable is in view."""
    cands = [(o, rows[o.label]) for o in objects if o.label in rows and o.confidence >= min_conf]
    cands.sort(key=lambda c: -(c[0].x_max - c[0].x_min) * (c[0].y_max - c[0].y_min))
    for o, e in cands:
        if o.label != last_label:
            return o, e
    return cands[0] if cands else (None, None)
