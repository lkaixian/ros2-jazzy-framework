"""When FollowTarget is blocked by someone other than the person it follows, ask them to make way. ROS-free."""


def other_person_near(persons, followed_id, max_d=2.0):
    """True when a person other than `followed_id` is depth-ranged within max_d metres."""
    return any(p.track_id != followed_id and 0.0 < p.distance_m <= max_d for p in persons)


def should_excuse(blocked_for, other_near, since_last, min_blocked=1.0, interval=12.0):
    """blocked_for = seconds blocked (None = not blocked); since_last = seconds since the last request (None = never)."""
    return bool(blocked_for is not None and blocked_for > min_blocked and other_near and (since_last is None or since_last >= interval))
