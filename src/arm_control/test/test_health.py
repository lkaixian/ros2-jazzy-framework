from arm_control.health import AT_REST_TOL, MAX_REBOOTS, PERSIST_SEC, RebootGate, assess, decode_hw

REST = {'joint1': 0.0, 'joint2': -1.85, 'joint3': 1.52, 'joint4': 0.45}
UP = {'joint1': 0.0, 'joint2': -0.5, 'joint3': 0.5, 'joint4': 0.0}


def test_decode_and_clean_arm():
    assert decode_hw(32 | 4) == ['overheating', 'overload']
    assert assess([11, 12], [0, 0], 0, {11: 30}, {11: 12.0}) == ([], False)


def test_overload_is_rebootable_but_heat_voltage_and_comm_are_not():
    p, r = assess([11, 12], [0, 32], 0)
    assert p == ['servo 12: overload error'] and r
    assert not assess([11], [32], 0, {11: 60})[1]                   # hot: the flag would come straight back
    assert not assess([11], [32 | 4], 0)[1] and not assess([11], [32], -5)[1]
    p, r = assess([11], [0], 0, {11: 52}, {11: 10.0})
    assert len(p) == 2 and not r                                    # warm + low voltage: report, never reboot


def test_reboot_only_when_parked_still_persistent_and_rationed():
    g = RebootGate()
    p = ['x']
    assert not g.allow(0.0, p, True, REST, REST, 0.0)[0]            # just appeared
    assert not g.allow(PERSIST_SEC + 1, p, True, UP, REST, 0.0)[0] and 'fall' in g.allow(PERSIST_SEC + 1, p, True, UP, REST, 0.0)[1]
    assert not g.allow(PERSIST_SEC + 1, p, True, REST, REST, 0.5)[0]
    assert g.allow(PERSIST_SEC + 1, p, True, REST, REST, 0.0)[0]
    for t in range(MAX_REBOOTS):
        g.record(100.0 + t)
    g.allow(200.0, p, True, REST, REST, 0.0)
    ok, why = g.allow(200.0 + PERSIST_SEC + 1, p, True, REST, REST, 0.0)
    assert not ok and 'needs a human' in why
    assert g.allow(800.0, p, True, REST, REST, 0.0)[0]              # the 10 min window has passed and the fault persisted throughout
    assert not g.allow(1.0, [], False, REST, REST, 0.0)[0] and AT_REST_TOL > 0


def test_a_cleared_fault_resets_the_persistence_timer():
    g = RebootGate()
    g.allow(0.0, ['x'], True, REST, REST, 0.0)
    g.allow(1.0, [], False, REST, REST, 0.0)
    assert not g.allow(PERSIST_SEC + 0.5, ['x'], True, REST, REST, 0.0)[0]
