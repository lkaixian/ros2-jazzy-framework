from types import SimpleNamespace as N

from pose_extension.people import PeopleBook, location_phrase, pointed_object, posture, unique_description


def kps(**pts):
    out = [(0.0, 0.0, 0.0)] * 17
    idx = dict(l_sh=5, r_sh=6, l_el=7, r_el=8, l_wr=9, r_wr=10, l_hip=11, r_hip=12, l_knee=13, r_knee=14)
    for name, (x, y) in pts.items():
        out[idx[name]] = (x, y, 0.9)
    return out


def person(tid, top, legs='', box=(0, 0, 100, 300), k=None):
    k = k or []
    return N(track_id=tid, clothes_colour=top, legs_colour=legs, x_min=box[0], y_min=box[1], x_max=box[2], y_max=box[3],
             keypoints_x=[p[0] for p in k], keypoints_y=[p[1] for p in k], keypoints_conf=[p[2] for p in k])


def test_posture_from_the_knee_height_relative_to_the_hips():
    sitting = kps(l_sh=(40, 100), r_sh=(80, 100), l_hip=(45, 200), r_hip=(75, 200), l_knee=(40, 215), r_knee=(80, 215))
    standing = kps(l_sh=(40, 100), r_sh=(80, 100), l_hip=(45, 200), r_hip=(75, 200), l_knee=(45, 300), r_knee=(75, 300))
    assert posture(sitting) == 'sitting' and posture(standing) == 'standing'
    assert posture(kps(l_sh=(40, 100), r_sh=(80, 100))) is None                        # no hips: not claimed


def book_of(*rows, repeat=3):
    """A book where every (tid, top, legs, extras) has been seen `repeat` frames."""
    b = PeopleBook()
    for t in range(repeat):
        b.update([person(tid, top, legs, box=(i * 120, 0, i * 120 + 100, 300)) for i, (tid, top, legs, _) in enumerate(rows)], [], float(t), 640, 480)
    for i, (tid, _, _, extra) in enumerate(rows):
        b.remember_at((i * 120 + 50, 100), **extra)
    return b


def test_one_odd_frame_is_not_a_fact_and_a_name_comes_from_the_face_inside_the_box():
    b = PeopleBook()
    b.update([person(1, 'red')], [], 0.0, 640, 480)
    assert b.people[1].features() == {}                                                  # a single look: not yet
    b.update([person(1, 'red')], [N(identity='alice', similarity=0.8, x_min=30, y_min=10, x_max=70, y_max=60)], 1.0, 640, 480)
    assert b.people[1].features() == {'top': 'red'} and b.by_name('Alice').id == 1
    b.update([person(1, 'red')], [N(identity='unknown', similarity=0.9, x_min=30, y_min=10, x_max=70, y_max=60)], 2.0, 640, 480)
    assert b.people[1].name == 'alice'                                                   # an unknown face never renames anyone


def test_unique_description_uses_only_features_nobody_else_shares_and_says_when_it_cannot():
    b = book_of((1, 'blue', 'black', {'hair': 'brown'}), (2, 'blue', 'grey', {'hair': 'black', 'glasses': 'yes'}), (3, 'red', 'black', {'hair': 'black'}))
    text, unique = unique_description(b, 1)
    assert unique and text.index('brown hair') > 0 and 'black hair' not in text         # brown hair is rare; the second feature pads it to two; unique overall
    text, unique = unique_description(b, 2)
    assert unique and 'glasses' in text
    twins = book_of((1, 'blue', 'black', {}), (2, 'blue', 'black', {}))
    text, unique = unique_description(twins, 1)
    assert not unique and 'blue top' in text                                              # identical clothes: it still says what it sees, but not "unique"


def test_where_a_person_is_only_from_objects_that_are_really_there():
    couch, tv = ('couch', (0, 100, 300, 300)), ('tv', (500, 50, 640, 150))
    assert location_phrase((100, 50, 200, 280), [couch, tv], 'sitting') == 'sitting on the couch'
    assert location_phrase((100, 50, 200, 280), [couch], 'standing') == 'standing in front of the couch'
    assert location_phrase((400, 50, 480, 300), [couch, tv], None) == 'next to the tv'
    assert location_phrase((100, 50, 140, 280), [], None) == '' and location_phrase((0, 0, 40, 40), [('tv', (600, 0, 640, 40))], None) == ''


def test_the_bag_the_arm_points_at_is_the_one_on_the_ray():
    k = kps(l_sh=(300, 200), l_el=(250, 205), l_wr=(200, 210))                           # the arm out to the image left
    left, right = ('backpack', (20, 180, 120, 260)), ('handbag', (500, 180, 600, 260))
    assert pointed_object(k, [left, right]) == left
    assert pointed_object(k, [right]) is None and pointed_object(k, [('cup', (20, 180, 120, 260))]) is None
    assert pointed_object(kps(l_sh=(300, 200)), [left]) is None


def test_gender_age_decoding_and_voting_say_nothing_when_unsure():
    import numpy as np
    from pose_extension.genderage import decode, face_square, vote
    g, margin, age = decode(np.array([[-0.7, 0.7, 0.34]]))
    assert (g, age) == ('man', 34) and margin > 0.99 and decode(np.array([[0.9, -0.5, 0.2]]))[0] == 'woman'
    assert face_square((100, 100, 140, 160), 640, 480)[2] - face_square((100, 100, 140, 160), 640, 480)[0] == 90      # a square 1.5 x the longer side
    assert vote([('man', 0.9, 30), ('man', 0.8, 34), ('man', 0.7, 40)]) == ('man', 34)
    assert vote([('man', 0.9, 30), ('woman', 0.8, 34), ('man', 0.7, 40)])[0] == ''                       # 2 of 3 is not enough
    assert vote([('man', 0.05, 30), ('woman', 0.04, 34)])[0] == '' and vote([]) == ('', 0)


def test_a_bags_colour_is_the_middle_of_its_box_not_the_strap_or_floor():
    import numpy as np
    from pose_extension.people import dominant_colour
    img = np.full((100, 60, 3), (90, 90, 90), np.uint8)                  # grey floor around the bag
    img[15:85, 10:50] = (0, 0, 200)                                      # a red bag
    img[:, :4] = (255, 255, 255)                                         # a white strap at the edge
    assert dominant_colour(img) == 'red' and dominant_colour(np.zeros((100, 60, 3), np.uint8)) == 'black'
    assert dominant_colour(np.zeros((4, 4, 3), np.uint8)) == ''


def test_finger_counting_from_landmarks_and_the_crop_around_a_raised_wrist():
    import numpy as np
    from pose_extension.hands import anchors, extended_fingers, wrist_crop
    open_hand = np.zeros((21, 3))                                   # a flat hand, fingers up: tips far above their middle joints
    open_hand[0] = (50, 100, 0)                                      # wrist
    for base, x in ((5, 40), (9, 50), (13, 60), (17, 70)):
        for k, y in enumerate((60, 45, 32, 20)):
            open_hand[base + k - 0] = (x, y, 0) if k else open_hand[base]
        open_hand[base] = (x, 60, 0)
        open_hand[base + 1], open_hand[base + 2], open_hand[base + 3] = (x, 45, 0), (x, 32, 0), (x, 20, 0)
    open_hand[1], open_hand[2], open_hand[3], open_hand[4] = (35, 90, 0), (25, 80, 0), (15, 70, 0), (5, 62, 0)      # the thumb out to the side
    assert extended_fingers(open_hand) == [True] * 5
    fist = open_hand.copy()
    for tip, joint in ((8, 6), (12, 10), (16, 14), (20, 18)):
        fist[tip] = fist[joint] + (0, 30, 0)                         # the tip folded back down past the middle joint
    assert extended_fingers(fist)[1:] == [False] * 4
    assert anchors().shape == (2016, 2) and abs(anchors()[0][0] - 0.0208333) < 1e-6
    kp = [(0.0, 0.0, 0.0)] * 17
    kp[5], kp[7], kp[9] = (300.0, 300.0, 0.9), (300.0, 230.0, 0.9), (300.0, 150.0, 0.9)      # the left arm straight up: wrist above the shoulder
    x1, y1, x2, y2, wr = wrist_crop(kp, (480, 640, 3))
    assert wr == (300.0, 150.0) and x1 < 300 < x2 and y2 < 150 + 90 and y1 < 150          # the square sits on and beyond the wrist, up the forearm
    assert wrist_crop([(0.0, 0.0, 0.0)] * 17, (480, 640, 3)) is None


def test_the_vlm_reply_is_parsed_voted_and_turned_into_a_sentence_without_inventing():
    from pose_extension.vlm import combine, parse, sentence
    reply = '```json\n{"top_colour": "Black", "top_type": "t-shirt", "bottom_colour": null, "hair_colour": "black", "hair_style": "short", "glasses": "yes", "gender": "male", "age_estimate": "25-30"}\n```'
    a = parse(reply)
    assert a['top'] == 'black' and a['bottom'] == '' and a['glasses'] == 'yes' and a['gender'] == 'man' and a['age'] == 25
    assert parse('I cannot tell.') == {} and parse('{broken') == {} and parse('{"gender": "robot", "age_estimate": 400}')['gender'] == ''
    assert sentence(combine([a, dict(a, age=31)])) == 'a man of about 31 wearing a black t-shirt, with short black hair and glasses'
    assert combine([a, dict(a, gender='woman')])['gender'] == ''                         # the looks disagree: no gender
    assert sentence(combine([parse('{"gender": null, "top_colour": null}')])) == '' and combine([]) == {}


def test_a_person_can_be_pointed_at_from_the_picture_and_still_a_moment_after_they_are_hidden():
    from pose_extension.people import PeopleBook
    b = PeopleBook()
    pose = (0.0, 0.0, 0.0)
    for i in range(20):                                              # someone 2 m ahead walking left (+y) at 0.5 m/s: their box drifts across the picture
        t = i * 0.1
        y = 0.5 * t
        cx = 320 - 320 * (y / 2.0) / 0.6745                         # the image offset of a point y m to the left at 2 m with a 69 degree lens
        p = person(1, 'red', box=(cx - 50, 100, cx + 50, 400))
        p.distance_m = 2.0
        b.update([p], [], t, 640, 480, pose=pose)
    rad, direct = b.bearing(b.people[1], pose, 1.9, 640)
    assert direct and 0.3 < rad < 0.6                                # to the left, about atan(0.95 / 2)
    rad2, direct2 = b.bearing(b.people[1], pose, 3.4, 640)            # 1.5 s later nobody is in view: the filter keeps walking them left
    assert not direct2 and rad2 > rad
    assert b.bearing(b.people[1], pose, 9.0, 640) is None             # long gone: nothing to point at
    out = b.bearings(pose, 1.95, 640)
    assert '#1' in out and out['nearest']['direct'] and abs(out['#1']['rad'] - rad) < 0.1
