from types import SimpleNamespace as NS

import numpy as np

from pid_extension.follow import Memory, select
from pid_extension.target import TargetModel

W, HFOV = 640, 69.0
rng = np.random.default_rng(1)


def look(centre, noise=0.08, d=64):
    v = centre + rng.standard_normal(d) * noise
    return v / np.linalg.norm(v)


def centres(d=64):
    a = rng.standard_normal(d)
    b = a * 0.7 + rng.standard_normal(d) * 0.7          # another person who resembles the target (cosine ~0.7 between centres)
    return a / np.linalg.norm(a), b / np.linalg.norm(b)


def trained(a, b, frames=40, **kw):
    m = TargetModel(**kw)
    for _ in range(frames):
        m.update(look(a), [look(b)])
    return m


def test_it_learns_the_target_against_a_person_who_resembles_them():
    a, b = centres()
    m = trained(a, b)
    assert m.ready
    tgt, oth = [m.score(look(a)) for _ in range(50)], [m.score(look(b)) for _ in range(50)]
    assert np.mean(tgt) > 0.8 and np.mean(oth) < 0.2 and min(tgt) > max(oth)


def test_it_is_not_ready_without_other_people_to_learn_against():
    a, _ = centres()
    m = TargetModel()
    for _ in range(40):
        m.update(look(a), [])                                # the target alone: nothing to tell them apart from
    assert not m.ready


def test_it_refuses_to_learn_from_a_frame_that_is_someone_else():
    a, b = centres()
    m = trained(a, b)
    n = len(m.pos)
    assert m.update(look(b), [look(a)]) is False and len(m.pos) == n    # the id switched: the 'target' scores like an other
    assert m.update(look(a), [look(b)]) is True


def test_buffers_stay_bounded():
    a, b = centres()
    m = trained(a, b, frames=600)
    assert len(m.pos) <= m.short and len(m.neg) <= m.short and len(m.lpos) <= m.long and len(m.lneg) <= m.long


def test_select_takes_a_candidate_only_after_five_clear_frames_and_never_the_lookalike():
    a, b = centres()
    m = trained(a, b)
    mem = Memory()
    mem.track_id = 1
    box = dict(x_min=300, x_max=340, y_min=100, y_max=300, distance_m=0.0, clothes_colour='')
    tgt = NS(track_id=9, embedding=look(a).tolist(), **box)              # the target back under a new id
    oth = NS(track_id=8, embedding=look(b).tolist(), **box)
    got = [select([oth, tgt], mem, (0, 0, 0), HFOV, W, lock=True, model=m) for _ in range(6)]
    assert [g.track_id if g else None for g in got[:4]] == [None] * 4 and got[4].track_id == 9 and got[5].track_id == 9


def test_a_candidate_that_drops_out_starts_the_count_again():
    a, b = centres()
    m = trained(a, b)
    mem = Memory()
    mem.track_id = 1
    box = dict(x_min=300, x_max=340, y_min=100, y_max=300, distance_m=0.0, clothes_colour='')
    tgt = NS(track_id=9, embedding=look(a).tolist(), **box)
    for _ in range(4):
        assert select([tgt], mem, (0, 0, 0), HFOV, W, lock=True, model=m) is None
    assert select([], mem, (0, 0, 0), HFOV, W, lock=True, model=m) is None          # gone for a frame
    assert select([tgt], mem, (0, 0, 0), HFOV, W, lock=True, model=m) is None       # count restarts at 1
