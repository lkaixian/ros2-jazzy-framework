import math

from pose_extension import gestures as g


def person(over=None):
    """Standing person facing camera; shoulders 100 px apart. over: {idx: (x, y)}."""
    over = over or {}
    k = [(0.0, 0.0, 0.0)] * 17
    k = list(k)
    k[g.L_SH], k[g.R_SH] = (400, 200, 0.9), (300, 200, 0.9)
    k[g.L_EL], k[g.R_EL] = (410, 280, 0.9), (290, 280, 0.9)
    k[g.L_WR], k[g.R_WR] = (415, 360, 0.9), (285, 360, 0.9)
    for i, (x, y) in over.items():
        k[i] = (x, y, 0.9)
    return k


def test_has_pose():
    assert g.has_pose(person(), 0.5)
    assert not g.has_pose([(0, 0, 0.0)] * 17, 0.5)
    assert not g.has_pose(person(), 0.95)


def test_pointing_image_right_and_left():
    right = person({g.L_EL: (480, 205), g.L_WR: (560, 210)})
    assert g.pointing(right, direction='') == 'right'
    assert g.pointing(right, direction='left') is None
    left = person({g.R_EL: (220, 205), g.R_WR: (140, 210)})
    assert g.pointing(left) == 'left'


def test_arm_down_or_bent_is_not_pointing():
    assert g.pointing(person()) is None
    bent = person({g.L_EL: (480, 205), g.L_WR: (470, 130)})   # forearm up
    assert g.pointing(bent) is None


def test_waving_needs_raised_swinging_wrist():
    frames = [(i * 0.1, person({g.R_WR: (300 + 70 * math.sin(i * 1.2), 120)}))
              for i in range(15)]
    assert g.waving(frames)
    still_up = [(i * 0.1, person({g.R_WR: (300, 120)})) for i in range(15)]
    assert not g.waving(still_up)
    swinging_low = [(i * 0.1, person({g.R_WR: (300 + 70 * math.sin(i * 1.2), 360)}))
                    for i in range(15)]
    assert not g.waving(swinging_low)
    assert not g.waving(frames[:3])                   # too few samples


def test_count_confident():
    from types import SimpleNamespace as N
    from pose_extension.pose_extension_node import count_confident
    items = [N(confidence=0.9, label='cup'), N(confidence=0.3, label='cup'), N(confidence=0.8, label='bowl')]
    assert count_confident(items, 0.5) == 2
    assert count_confident(items, 0.5, 'cup') == 1
    assert count_confident([], 0.5) == 0


def test_free_spots_chairs_whole_and_a_couch_by_the_room_beside_the_people_on_it():
    from pose_extension.people import free_spots
    chair_taken, chair_free = ('chair', (0, 0, 100, 100), 0.9), ('chair', (300, 0, 350, 50), 0.8)
    dim, table = ('chair', (700, 0, 800, 100), 0.2), ('dining table', (0, 0, 900, 100), 0.9)
    person = (10, 0, 90, 100)                                           # sits on the first chair
    got = free_spots([chair_taken, chair_free, dim, table], [person])
    assert [g[0] for g in got] == ['chair'] and got[0][1] == 325 and not got[0][3]      # only the free chair: taken, dim and non-seats dropped
    assert free_spots([chair_taken], [])[0][1] == 50                                    # nobody in view: free
    couch = ('couch', (200, 0, 600, 100), 0.7)                          # a 400 px sofa, three people wide
    assert free_spots([couch], []) == [('couch', 400, 400, False, (200, 0, 600, 100))]  # empty: the whole thing, centre
    one_end = [(210, 20, 300, 100)]                                    # one person (90 px) at the left end: the sofa seats 4 of them
    spots = free_spots([couch], one_end)
    assert len(spots) == 3 and all(sp[3] for sp in spots) and min(sp[1] for sp in spots) > 300      # three free seats, all to the right, all "shared"
    full = [(200, 20, 290, 100), (300, 20, 390, 100), (400, 20, 490, 100), (505, 20, 595, 100)]
    assert free_spots([couch], full) == []                              # every seat taken: no free place
    assert free_spots([couch], [(380, 0, 440, 25)])[0][3] is False      # a person standing in front, not reaching into the seat: not sitting on it


def test_a_three_seat_sofa_with_the_host_in_the_middle_offers_both_ends_and_a_wide_host_leaves_nothing():
    from pose_extension.people import free_spots
    sofa = ('couch', (100, 0, 400, 100), 0.8)                          # 300 px: three people of 100 px
    spots = free_spots([sofa], [(190, 20, 290, 100)])                  # the host in the middle, 100 px wide
    assert len(spots) == 2 and sorted(sp[1] for sp in spots)[0] < 190 and sorted(sp[1] for sp in spots)[1] > 290     # a free seat on each side
    assert free_spots([sofa], [(160, 20, 340, 100)]) == []             # a 180 px host (arms out) on a 300 px sofa: it counts as a two-seater and both are taken


def test_seat_place_says_which_side():
    from pose_extension.pose_extension_node import seat_place
    assert seat_place('chair', -0.6) == 'chair on my left'
    assert seat_place('couch', 0.6) == 'couch on my right'
    assert seat_place('bench', 0.1) == 'bench straight ahead'


def test_extreme_object_and_colour_count():
    from types import SimpleNamespace as N

    from pose_extension.pose_extension_node import count_confident, extreme_object
    big, small, thin = (N(label=n, confidence=0.9, x_min=0, y_min=0, x_max=w, y_max=h) for n, w, h in (('big', 100, 80), ('small', 20, 20), ('thin', 90, 5)))
    objs = [big, small, thin, N(label='ghost', confidence=0.1, x_min=0, y_min=0, x_max=500, y_max=500)]
    assert extreme_object(objs, 'biggest').label == 'big' and extreme_object(objs, 'smallest').label == 'small'
    assert extreme_object(objs, 'thinnest').label == 'thin' and extreme_object([], 'biggest') is None
    people = [N(confidence=0.9, clothes_colour='red'), N(confidence=0.9, clothes_colour='blue'), N(confidence=0.2, clothes_colour='red')]
    assert count_confident(people, 0.5) == 2 and count_confident(people, 0.5, colour='red') == 1
    people.append(N(confidence=0.9, clothes_colour='grey'))
    assert count_confident(people, 0.5, colour='Gray') == 1


def test_describe_adds_the_legs_only_when_their_colour_is_known():
    from types import SimpleNamespace as N
    from pose_extension.pose_extension_node import describe
    p = N(confidence=0.9, clothes_colour='blue', legs_colour='black', track_id=1, x_min=0, x_max=10, y_min=0, y_max=20)
    assert describe([p]) == 'a person wearing a blue top and black bottoms'
    p.legs_colour = ''
    assert describe([p]) == 'a person wearing a blue top'
