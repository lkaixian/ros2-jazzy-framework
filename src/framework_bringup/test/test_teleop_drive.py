import importlib.machinery
import importlib.util
import os

p = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'teleop.py')
spec = importlib.util.spec_from_loader('teleop', importlib.machinery.SourceFileLoader('teleop', p))
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


def run(d, t0, t1, dt=0.05):
    out, t = [], t0
    while t < t1:
        out.append(d.tick(t, dt))
        t += dt
    return out


def test_release_ramps_down_then_sends_one_zero_then_goes_quiet():
    d = teleop.Drive()
    d.press(0.0, 'w', (0.2, 0.0))
    out = run(d, 0.0, 3.0)
    lin = [o[0] for o in out if o]
    assert max(lin) == 0.2
    peak = lin.index(0.2)
    assert all(a >= b for a, b in zip(lin[peak:], lin[peak + 1:]))       # monotone slow-down, never a snap
    assert max(a - b for a, b in zip(lin[peak:], lin[peak + 1:])) <= 0.4 * 0.05 + 1e-9
    assert lin[-1] == 0.0 and out[-1] is None                           # one zero published, then silence


def test_held_key_keeps_driving_and_repeat_hold_is_short():
    d = teleop.Drive()
    for i in range(10):                                                  # repeats every 0.05 s from the first press
        d.press(i * 0.05, 'w', (0.2, 0.0))
        d.tick(i * 0.05, 0.05)
    assert d.until - 0.45 < 0.16 and d.cur[0] > 0


def test_space_stops_at_once_and_publishes_zero():
    d = teleop.Drive()
    d.press(0.0, 'w', (0.2, 0.0))
    run(d, 0.0, 1.0)
    d.press(1.0, ' ', (0.0, 0.0))
    assert d.tick(1.0, 0.05) == (0.0, 0.0) and d.tick(1.05, 0.05) is None
