"""What the planner is told about one visible person. Pure logic (gesture rules live in
pose_extension.gestures, shared with the IsPersonWaving / IsPersonPointing nodes)."""
from pose_extension import gestures


def describe_person(track_id, history, distance_m=0.0, min_conf=0.5):
    """history: [(t, [(x, y, conf), ...])] over the recent window, newest last (may be empty).

    Keys other than `id` appear only when known, so the LLM is never told a guess: no `waving`
    key means "no usable skeleton", `waving: false` means "skeleton seen, not waving".
    """
    d = {'id': track_id}
    if distance_m and distance_m > 0:
        d['distance_m'] = round(float(distance_m), 1)
    if history and gestures.has_pose(history[-1][1], min_conf):
        d['waving'] = bool(gestures.waving(history, min_conf))
        side = gestures.pointing(history[-1][1], min_conf)
        if side:
            d['pointing'] = side
    return d
