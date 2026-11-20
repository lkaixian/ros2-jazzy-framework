from bt_common.cmd_vel_mux import BumpRecovery, CENTER, LEFT, RIGHT, StopLatch, clamp, select, slew


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


def _run(ids, until=10.0, dt=0.05):
    """Drive a BumpRecovery from t=0 and return the distinct (vx, wz) commands it gave."""
    r = BumpRecovery()
    r.begin(0.0, ids[0])
    for b in ids[1:]:
        r.hit(b)
    seen, t = [], 0.0
    while t < until:
        c = r.cmd(t)
        if c is None:
            break
        if not seen or seen[-1] != c:
            seen.append(c)
        t += dt
    return seen, r


def test_recovery_right_bump_backs_off_then_turns_left():
    seen, r = _run([RIGHT])
    assert seen == [(0.0, 0.0), (-0.1, 0.0), (0.0, 0.5)] and r.phase is None


def test_recovery_left_bump_turns_right():
    assert _run([LEFT])[0][-1] == (0.0, -0.5)


def test_recovery_centre_or_two_bumpers_is_a_wall_and_turns_further():
    for ids in ([CENTER], [LEFT, RIGHT]):
        seen, _ = _run(ids)
        assert seen[1] == (-0.1, 0.0) and seen[2] == (0.0, 0.5)
        r = BumpRecovery()
        r.begin(0.0, ids[0])
        [r.hit(b) for b in ids[1:]]
        r.cmd(0.4)                                   # past settle: verdict made
        assert r.t_turn > 3.0                        # 1.57 rad vs 0.5 rad for one side


def test_recovery_press_after_settle_cancels():
    r = BumpRecovery()
    r.begin(0.0, RIGHT)
    assert r.cmd(0.5) == (-0.1, 0.0)
    r.hit(LEFT)                                      # bumped again while backing
    assert r.cmd(0.6) is None


def test_slew_ramps_and_lands_exactly():
    v, steps = 0.0, 0
    while v != 0.2 and steps < 20:
        v = slew(v, 0.2, 0.03)
        steps += 1
    assert v == 0.2 and steps == 7                    # 6 full steps, then the remainder
    assert abs(slew(0.2, 0.0, 0.03) - 0.17) < 1e-9
    assert slew(0.1, 0.11, 0.03) == 0.11
