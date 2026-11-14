from bt_common.battery_guard import battery_level, BatteryWatch


def test_levels_and_charging():
    assert battery_level(158) == 'ok' and battery_level(135) == 'low' and battery_level(130) == 'critical'
    assert battery_level(130, charger=2) == 'low'          # plugged in: never critical
    assert battery_level(0) == 'unknown'


def test_watch_debounces_then_halts_repeatedly_and_resets():
    w = BatteryWatch(debounce_sec=3.0, repeat_sec=5.0)
    assert w.update('critical', 0.0) is None               # first sample: wait out the debounce (a voltage dip under load)
    assert w.update('critical', 2.0) is None
    assert w.update('critical', 3.0) == 'halt'
    assert w.update('critical', 4.0) is None and w.update('critical', 8.5) == 'halt'     # repeats every 5 s
    assert w.update('ok', 9.0) is None and w.update('critical', 10.0) is None            # recovered: debounce starts over


def test_low_only_warns_now_and_then():
    w = BatteryWatch(warn_sec=60.0)
    assert w.update('low', 0.0) == 'warn' and w.update('low', 30.0) is None and w.update('low', 61.0) == 'warn'
