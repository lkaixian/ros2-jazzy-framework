from bt_common.cmd_vel_mux import StopLatch, clamp, select


def test_clamp():
    assert clamp(5, 0.3) == 0.3 and clamp(-5, 0.3) == -0.3 and clamp(0.1, 0.3) == 0.1


def test_select_priority_and_staleness():
    last = [(10.0, 'a'), (10.4, 'b')]
    assert select(last, 10.2, 0.5) == 0           # both fresh -> highest priority
    assert select(last, 10.6, 0.5) == 1           # first stale -> falls to second
    assert select(last, 11.0, 0.5) is None        # all stale
    assert select([(None, 'x')], 0.0, 0.5) is None  # never received


def test_latch_starts_armed_and_latches_on_hit():
    st = StopLatch()
    assert not st.latched
    assert st.event(('bumper', 1), True, 'bumper 1') is True
    assert st.latched and st.reason == 'bumper 1'
    assert st.event(('cliff', 0), True, 'cliff 0') is False   # already latched: not a new latch
    assert st.reason == 'bumper 1'


def test_release_alone_does_not_clear_latch():
    st = StopLatch()
    st.event(('bumper', 1), True)
    st.event(('bumper', 1), False)
    assert st.latched                       # a human must reset


def test_reset_refused_while_sensor_active():
    st = StopLatch()
    st.event(('cliff', 0), True)
    ok, msg = st.reset()
    assert not ok and 'still active' in msg and st.latched
    st.event(('cliff', 0), False)
    assert st.reset() == (True, 'reset') and not st.latched


def test_reset_needs_every_sensor_clear():
    st = StopLatch()
    st.event(('bumper', 0), True)
    st.event(('bumper', 2), True)
    st.event(('bumper', 0), False)
    assert not st.reset()[0]
    st.event(('bumper', 2), False)
    assert st.reset()[0]


def test_unavailable_sensor_blocks_reset():
    st = StopLatch()
    st.event('bumper-unavailable', True, 'type unavailable')
    assert not st.reset()[0] and st.latched


def test_reset_when_not_latched_is_ok():
    assert StopLatch().reset() == (True, 'not latched')
