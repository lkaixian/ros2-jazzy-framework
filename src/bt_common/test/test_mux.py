from bt_common.cmd_vel_mux import clamp, select


def test_clamp():
    assert clamp(5, 0.3) == 0.3 and clamp(-5, 0.3) == -0.3 and clamp(0.1, 0.3) == 0.1


def test_select_priority_and_staleness():
    last = [(10.0, 'a'), (10.4, 'b')]
    assert select(last, 10.2, 0.5) == 0           # both fresh -> highest priority
    assert select(last, 10.6, 0.5) == 1           # first stale -> falls to second
    assert select(last, 11.0, 0.5) is None        # all stale
    assert select([(None, 'x')], 0.0, 0.5) is None  # never received
