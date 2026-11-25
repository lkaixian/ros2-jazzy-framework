import os
import types

import pytest

from pose_extension.categories import load_table, pick_next

TABLE = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param', 'object_categories.yaml')


def obj(label, conf=0.9, size=100):
    return types.SimpleNamespace(label=label, confidence=conf, x_min=0, y_min=0, x_max=size, y_max=size)


ROWS = {'cup': {'group': 'kitchenware', 'nav': 'dishwasher', 'put': 'dishwasher'},
        'bottle': {'group': 'drinks', 'nav': 'dustbin', 'put': 'bin'}}


def test_shipped_table_is_well_formed_and_covers_every_task():
    t = load_table(TABLE)
    assert {'clean_table', 'groceries', 'rubbish'} <= set(t) and all(t[k] for k in t)


def test_picks_the_biggest_handleable_confident_object():
    o, e = pick_next([obj('person', size=500), obj('cup', size=50), obj('bottle', size=80), obj('bottle', conf=0.2, size=900)], ROWS)
    assert (o.label, e['put']) == ('bottle', 'bin')


def test_prefers_another_label_than_the_last_but_falls_back():
    assert pick_next([obj('bottle', size=80), obj('cup', size=50)], ROWS, last_label='bottle')[0].label == 'cup'
    assert pick_next([obj('bottle')], ROWS, last_label='bottle')[0].label == 'bottle'
    assert pick_next([obj('person')], ROWS) == (None, None) and pick_next([], ROWS) == (None, None)


def test_malformed_table_is_refused(tmp_path):
    f = tmp_path / 't.yaml'
    f.write_text('clean_table:\n  cup: {group: kitchenware}\n')
    with pytest.raises(ValueError, match='needs group, nav and put'):
        load_table(str(f))
