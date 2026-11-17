from face_recognition.auto_register_node import UnknownTracker


def feed(t, start, end, unknown, free=True, step=0.1):
    """Feed frames every `step` s from start (inclusive) to end (exclusive); return the times at which it fired."""
    fired, now = [], start
    while now < end - 1e-9:
        if t.update(now, unknown, free):
            fired.append(round(now, 1))
        now += step
    return fired


def tracker():
    return UnknownTracker(hold=2.5, gap=0.6, cooldown=60)


def test_fires_once_after_hold_then_cooldown():
    t = tracker()
    assert feed(t, 0.0, 2.4, True) == []                  # 2.3 s of unknown: not yet
    assert feed(t, 2.4, 3.0, True) == [2.5]               # 2.5 s: ask once
    assert feed(t, 3.0, 62.0, True) == []                 # cooldown: no nagging
    assert feed(t, 62.0, 66.0, True) != []                # still there after the cooldown: ask again


def test_a_short_blink_is_forgiven():
    t = tracker()
    feed(t, 0.0, 1.0, True)
    feed(t, 1.0, 1.4, None)                               # 0.4 s without a usable face
    assert feed(t, 1.4, 3.0, True) == [2.5]


def test_a_long_gap_or_a_known_face_restarts_the_timer():
    t = tracker()
    feed(t, 0.0, 2.0, True)
    feed(t, 2.0, 3.0, None)                               # 1 s of nobody
    assert feed(t, 3.0, 5.0, True) == []                  # new streak, only 2 s so far
    t = tracker()
    feed(t, 0.0, 2.0, True)
    feed(t, 2.0, 3.0, False)                              # a recognised face took the frame
    assert feed(t, 3.0, 5.0, True) == []


def test_busy_robot_does_not_ask_but_does_once_free():
    t = tracker()
    assert feed(t, 0.0, 4.0, True, free=False) == []      # a task is running
    assert feed(t, 4.0, 4.3, True, free=True) == [4.0]    # the long-standing unknown is asked about when free
