from pose_extension.stop import hand_raised, StopGate


def kps(raised=False):
    k = [(0.0, 0.0, 0.0)] * 17
    k[5], k[6] = (100.0, 200.0, 0.9), (160.0, 200.0, 0.9)                    # shoulders 60 px apart
    k[9] = (100.0, 150.0, 0.9) if raised else (100.0, 330.0, 0.9)              # left wrist above the shoulder / down by the hip
    k[10] = (160.0, 330.0, 0.9)
    return k


def walk(g, pid, t0, t1, speed, raised=False):
    """The person at `speed` m/s from t0 to t1, seen 10 times a second."""
    t = t0
    while t <= t1 + 1e-9:
        g.see(pid, (speed * t, 0.0), kps(raised), t)
        t += 0.1


def test_a_raised_hand_is_a_wrist_well_above_the_shoulder_not_an_arm_out_sideways():
    assert hand_raised(kps(True)) and not hand_raised(kps(False))
    out = kps(True)
    out[9] = (0.0, 190.0, 0.9)                                                   # arm held out to the side, level with the shoulder
    assert not hand_raised(out) and not hand_raised([])


def test_the_word_stop_counts_only_while_the_operator_stands_still():
    g = StopGate(words=['stop', 'we are here'])
    walk(g, 1, 0.0, 3.0, speed=0.8)                                              # the operator walks
    g.heard('somebody shouted STOP!', 3.0)
    assert g.check(1, 3.05) == '' and g.ignored == 1                             # moving: ignored, and the word is used up
    walk(g, 1, 3.0, 4.6, speed=0.0)                                              # then they stand still
    assert g.check(1, 4.6) == ''                                                 # the earlier word was not an order and does not count later
    g.heard('we are here', 4.7)
    assert g.check(1, 4.7) == 'voice'


def test_a_hand_held_up_for_a_second_stops_a_standing_operator_but_not_a_walking_one():
    g = StopGate(words=['stop'])
    walk(g, 1, 0.0, 2.0, speed=0.8, raised=True)                                 # waving at the crowd while walking
    assert g.check(1, 2.0) == ''
    g2 = StopGate(words=['stop'])
    walk(g2, 1, 0.0, 0.5, speed=0.0, raised=True)
    assert g2.check(1, 0.5) == ''                                                # held up for only 0.5 s
    walk(g2, 1, 0.5, 2.0, speed=0.0, raised=True)
    assert g2.check(1, 2.0) == 'sign'
    g2.see(1, (0, 0), kps(False), 2.1)                                          # hand down: starts over
    assert g2.check(1, 2.1) == ''


def test_without_a_known_operator_a_stop_word_is_accepted_and_other_people_never_stop_it_by_gesture():
    g = StopGate(words=['stop'])
    g.heard('stop', 1.0)
    assert g.check(None, 1.2) == 'voice'                                         # nobody tracked: still stoppable
    g.see(7, (0, 0), kps(True), 0.0)
    for t in (0.5, 1.0, 1.5, 2.0):
        g.see(7, (0, 0), kps(True), t)
    assert g.check(1, 2.0) == ''                                                # person 7 holds a hand up, but the operator is 1
    assert g.check(7, 2.0) == 'sign'
