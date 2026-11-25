from arm_tool import MIN_SEC, RATE, load_rest, park_seconds


def test_rest_pose_loads_four_joints():
    r = load_rest()
    assert list(r) == ['joint1', 'joint2', 'joint3', 'joint4'] and r['joint2'] == -1.85


def test_park_is_never_faster_than_rate():
    home, rest = [0.0, -1.0, 1.0, 0.0], load_rest()
    cur = dict(zip(rest, home))
    sec = park_seconds(cur, rest)
    assert sec >= MIN_SEC and max(abs(rest[j] - cur[j]) for j in rest) / sec <= RATE + 1e-9
    assert park_seconds(rest, rest) == MIN_SEC
