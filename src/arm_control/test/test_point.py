import math

from arm_control.point import candidates, find_pose


def test_the_exact_direction_comes_first_and_the_turn_only_shrinks():
    c = list(candidates(0.8))
    a0, xyz0, pitch0 = c[0]
    assert a0 == 0.8 and pitch0 == 0.0 and math.isclose(xyz0[1] / xyz0[0], math.tan(0.8)) and xyz0[2] == 0.12
    turns = [x[0] for x in c]
    assert turns == sorted(turns, reverse=True) and turns[-1] == 0.0


def test_the_first_candidate_the_callers_checks_accept_is_taken_and_a_refused_pose_is_never_used():
    seen = []

    def solve(xyz, pitch):                                         # joint 1 can only reach 0.4 rad, and only at the lowest height
        a = math.atan2(xyz[1], xyz[0])
        seen.append(xyz)
        return {'joint1': a, 'z': xyz[2]} if abs(a) <= 0.4 and xyz[2] <= 0.12 else None
    q, info = find_pose(0.8, solve, lambda q: q['joint1'] != 0.0 or True)
    assert q is not None and info['turn'] == 0.8 * 0.5 and info['xyz'][2] == 0.12      # 100 % and 75 % are out of range: half the turn
    q2, info2 = find_pose(0.8, solve, lambda q: False)             # the route check refuses everything: nothing is returned, the arm would not move
    assert q2 is None and info2['tries'] > 0
    q3, info3 = find_pose(0.0, lambda xyz, pitch: None, lambda q: True, max_tries=7)
    assert q3 is None and info3['tries'] == 7
