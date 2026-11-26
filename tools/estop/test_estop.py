import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from estop_node import EstopState  # noqa: E402 - needs rclpy + pyserial (ROS sourced)


def run(events, **kw):
    """events: [(t, line or None)], ticks every 0.1 s -> list of (t, stopped, want_reset)."""
    s, out, ev = EstopState(**kw), [], dict(events)
    for k in range(0, 100):
        t = k / 10
        if ev.get(t):
            s.feed(ev[t], t)
        out.append((t, *s.tick(t)))
    return out


def test_stopped_until_heard_then_released_after_hold_and_reset_once_wanted():
    out = run([(0.0, 'E0')] + [(k / 10, 'E0') for k in range(1, 30)])
    assert out[5][1] and not out[5][2]                 # armed for 0.5 s: still stopped (hold 1.0)
    assert not out[11][1] and out[11][2]               # released, reset wanted


def test_press_stops_instantly_and_release_needs_the_hold_again():
    ev = [(k / 10, 'E0') for k in range(0, 20)] + [(2.0, 'E1')] + [(k / 10, 'E0') for k in range(21, 60)]
    out = run(ev)
    assert not out[19][1]
    assert out[20][1]                                  # E1: stopped at once
    assert out[25][1]                                  # E0 again but not held 1 s
    assert not out[35][1] and out[35][2]


def test_silence_and_garbage_are_a_stop():
    out = run([(0.0, 'E0'), (0.1, 'E0'), (0.2, 'E0')])
    assert out[9][1]                                   # no data for > 0.5 s
    out = run([(k / 10, 'xyz') for k in range(50)])
    assert all(o[1] for o in out)                      # garbage never releases
