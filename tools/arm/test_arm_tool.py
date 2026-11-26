from arm_tool import (MIN_SEC, RATE, TOUR_RATE, TURN_RATE, load_rest, park_seconds,
                      tour_plan)

LIMITS = {'joint1': 3.14, 'joint2': 1.5, 'joint3': 1.4, 'joint4': 1.7}   # URDF |limit| (rad), joint2-4 lower/upper differ slightly


def test_rest_pose_loads_four_joints():
    r = load_rest()
    assert list(r) == ['joint1', 'joint2', 'joint3', 'joint4'] and r['joint2'] == -1.85


def test_park_is_never_faster_than_rate():
    home, rest = [0.0, -1.0, 1.0, 0.0], load_rest()
    cur = dict(zip(rest, home))
    sec = park_seconds(cur, rest)
    assert sec >= MIN_SEC and max(abs(rest[j] - cur[j]) for j in rest) / sec <= RATE + 1e-9
    assert park_seconds(rest, rest) == MIN_SEC


def test_tour_ends_at_rest_and_never_exceeds_its_rates():
    rest = load_rest()
    plan = tour_plan(rest)
    assert plan[-1][1] == tuple(rest.values())
    cur = tuple(rest.values())
    for label, pos, sec in plan:
        rate = TURN_RATE if 'turn' in label or label == 'centre' else TOUR_RATE
        assert max(abs(a - b) for a, b in zip(pos, cur)) / sec <= rate + 1e-9, label
        cur = pos


def test_tour_poses_stay_inside_the_joint_limits_except_the_known_rest_pose():
    plan = tour_plan(load_rest())
    for label, pos, _ in plan[:-1]:
        assert all(abs(v) <= LIMITS[f'joint{i + 1}'] for i, v in enumerate(pos)), label


def test_tour_turn_is_180_degrees_through_the_front():
    plan = {label: pos for label, pos, _ in tour_plan(load_rest(), 'right')}
    assert plan['turn right 90'][0] < 0 < plan['turn other way 90'][0] and plan['centre'][0] == 0.0


def test_park_corridor_accepts_rest_and_home_but_not_the_poses_that_went_wrong():
    from arm_tool import in_corridor
    rest = load_rest()
    at = lambda q: dict(zip(('joint1', 'joint2', 'joint3', 'joint4'), q))  # noqa: E731
    assert in_corridor(rest, rest) and in_corridor(at((0.0, -1.0, 1.0, 0.0)), rest)
    assert in_corridor(at((0.0, -1.4, 1.25, 0.2)), rest)            # between rest and home
    assert not in_corridor(at((1.19, 2.07, -0.49, -0.03)), rest)    # the lying-down pose from the tour
    assert not in_corridor(at((0.0, 1.38, -1.38, -0.04)), rest)     # 'long'
    assert not in_corridor(at((0.0, -0.14, -1.38, -0.04)), rest)    # 'high'


def test_fk_reproduces_the_rest_and_home_gripper_positions():
    from omx_fk import Arm
    arm = Arm()
    p = arm.xyz('end_effector_link', load_rest())
    assert abs(p[0] - 0.125) < 0.005 and abs(p[2] - 0.072) < 0.005
    h = arm.xyz('end_effector_link', dict(joint2=-1.0, joint3=1.0))
    assert abs(h[0] - 0.167) < 0.005 and abs(h[2] - 0.149) < 0.005


def test_gravity_torque_grows_with_reach_and_the_cost_is_monotone():
    import math

    from torque import TorqueModel, cost
    m = TorqueModel(payload_kg=0.15)
    j2 = lambda **q: abs(m.torques(q)['joint2'])  # noqa: E731
    assert j2(joint2=-1.85, joint3=1.52, joint4=0.45) < j2(joint2=-1.0, joint3=1.0) < j2(joint2=1.38, joint3=-1.38, joint4=-0.04)
    for kind in ('linear', 'poly', 'exp'):
        cs = [cost(u, kind) for u in (0.3, 0.4, 0.5, 0.6, 0.7, 0.79)]
        assert cs == sorted(cs) and cs[0] == 0.0 and cost(0.8, kind) == math.inf


def test_free_map_grows_only_with_new_space_and_survives_save_load(tmp_path):
    from omx_fk import Arm
    from free_map import FreeMap
    arm, m = Arm(), FreeMap()
    rest, home = load_rest(), dict(joint1=0.0, joint2=-1.0, joint3=1.0, joint4=0.0)
    assert m.add(arm, rest) > 0 and m.add(arm, rest) == 0            # the same pose again opens nothing
    v1 = m.stats()['volume_l']
    m.add(arm, home)
    assert m.stats()['volume_l'] > v1
    m.save(tmp_path / 'm.npz')
    back = FreeMap.load(tmp_path / 'm.npz')
    assert back.stats()['voxels'] == m.stats()['voxels'] and back.n == m.n and back.hi['joint3'] == m.hi['joint3']
    assert any('@' in line for line in back.render(cur=(0.1, 0.0, 0.05)))


def test_load_map_keeps_best_and_worst_load_per_gripper_cell(tmp_path):
    from load_map import load_cells, summarise
    from torque import TorqueModel
    rows = ['t,joint1,joint2,joint3,joint4,ee_x,ee_y,ee_z',
            '0,0,-1.85,1.52,0.45,0.125,0.0,0.072', '1,0,-1.0,1.0,0.0,0.125,0.0,0.072', '2,0,1.38,-1.38,-0.04,0.39,0.0,0.07']
    f = tmp_path / 's.csv'
    f.write_text('\n'.join(rows) + '\n')
    cells = load_cells([str(f)], TorqueModel(payload_kg=0.15), 4.1, stride=1)
    assert len(cells) == 2                                                   # rows 0 and 1 land in the same 2 cm cell
    shared = cells[(6, 0, 3)]
    assert shared[0] < shared[1] and shared[2] == 2                          # best < worst, two poses seen there
    assert summarise(cells)['worst_max'] > shared[1]                         # the stretched-out cell is the heaviest


def test_path_check_accepts_swept_motion_refuses_unswept_space_and_out_of_range_joints():
    import numpy as np
    from free_map import FreeMap
    from omx_fk import Arm
    from path_check import check_path
    from torque import TorqueModel
    arm, fm = Arm(), FreeMap()
    rest, home, long_ = load_rest(), dict(joint1=0.0, joint2=-1.0, joint3=1.0, joint4=0.0), dict(joint1=0.0, joint2=1.38, joint3=-1.38, joint4=-0.04)
    for s in np.linspace(0, 1, 60):                                    # the hand-taught sweep: rest -> home only
        fm.add(arm, {k: rest[k] + s * (home[k] - rest[k]) for k in rest})
    model = TorqueModel(arm=arm)
    assert check_path(arm, fm, model, rest, home)['ok']
    far = check_path(arm, fm, model, home, long_)
    assert not far['ok'] and any('swept' in r or 'range' in r for r in far['reasons'])
    twisted = dict(rest, joint1=3.5)
    assert not check_path(arm, fm, model, rest, twisted)['ok']          # more than half a turn and outside the seen range


def test_route_follows_taught_poses_and_refuses_what_was_never_taught(tmp_path):
    import numpy as np
    import route
    from route import load_roadmap, plan
    # one hand-guided stream: rest -> home -> a side branch that ends at a "far" pose
    rest, home, far = np.array([0.0, -1.85, 1.52, 0.45]), np.array([0.0, -1.0, 1.0, 0.0]), np.array([0.8, -0.5, 0.5, 0.0])
    rows = ['t,joint1,joint2,joint3,joint4']
    for a, b in ((rest, home), (home, far)):
        for s in np.linspace(0, 1, 40):
            q = a + s * (b - a)
            rows.append('0,' + ','.join(f'{v:.4f}' for v in q))
    f = tmp_path / 's.csv'
    f.write_text('\n'.join(rows) + '\n')
    nodes, edges = load_roadmap([str(f)])
    wps, why = plan(nodes, edges, far, rest)
    assert wps is not None and np.allclose(wps[0], far) and np.allclose(wps[-1], rest)
    assert all(np.linalg.norm(nodes - w, axis=1).min() <= route.SNAP for w in wps)     # every waypoint is on the taught roadmap
    none, reason = plan(nodes, edges, np.array([2.0, 2.0, -1.0, 1.0]), rest)
    assert none is None and 'taught' in reason


def _state(pos=(0, 0, 0, 0), cur=(0, 0, 0, 0), err=(0, 0, 0, 0), temp=(30,) * 4, volt=(12.0,) * 4):
    import numpy as np
    return {k: np.array(v, float) for k, v in dict(pos=pos, cur=cur, err=err, temp=temp, volt=volt).items()}


def test_load_test_abort_rules_each_fire():
    import numpy as np
    from load_test import ABORT_UNITS, abort_reason
    g = np.zeros(4)
    assert abort_reason(_state(), g, 'hold') is None
    assert 'hardware error' in abort_reason(_state(err=(0, 32, 0, 0)), g, 'hold')
    assert 'current' in abort_reason(_state(cur=(0, ABORT_UNITS + 5, 0, 0)), g, 'move')
    assert 'position error' in abort_reason(_state(pos=(0, 0.1, 0, 0)), g, 'hold')
    assert abort_reason(_state(pos=(0, 0.1, 0, 0)), g, 'move') is None                      # looser while travelling
    assert 'temperature' in abort_reason(_state(temp=(30, 60, 30, 30)), g, 'hold')
    assert 'supply voltage' in abort_reason(_state(volt=(12, 12, 9.0, 12)), g, 'hold')
    assert 'far heavier' in abort_reason(_state(cur=(0, 300, 0, 0)), g, 'hold', pred_a=np.array([0, 0.1, 0, 0]))   # 0.8 A vs 0.1 A predicted


def test_load_test_on_a_simulated_bus_measures_the_model_and_freezes_on_overload(tmp_path):
    import numpy as np
    from load_test import FakeBus, build_ladder, run
    from route import load_roadmap
    from torque import TorqueModel
    rest = load_rest()
    q_rest = np.array(list(rest.values()))
    legs = [q_rest, np.array([0.0, -1.0, 1.0, 0.0]), np.array([0.0, 0.2, -0.3, 0.0]), np.array([0.0, 0.8, -1.0, 0.2])]
    rows = ['t,joint1,joint2,joint3,joint4']
    for a, b in zip(legs, legs[1:]):
        rows += ['0,' + ','.join(f'{v:.4f}' for v in a + s * (b - a)) for s in np.linspace(0, 1, 60)]
    f = tmp_path / 's.csv'
    f.write_text('\n'.join(rows) + '\n')
    nodes, edges = load_roadmap([str(f)])
    model = TorqueModel(payload_kg=0.15)
    ladder = build_ladder(nodes, edges, rest, model, [], k=3, min_gap=0.2)
    assert len(ladder) >= 2 and [c[0] for c in ladder] == sorted(c[0] for c in ladder)      # lightest pose first
    quiet = {'sleep': lambda s: None, 'ask': lambda p: 'q'}
    ok_bus = FakeBus(model, q_rest)
    results, why = run(ok_bus, model, ladder, rest, nodes, edges, 0.2, 3.0, **quiet)
    assert why is None and len(results) == len(ladder)
    ratio = [r['cur_units'][1] / max(r['pred_tau'][1], 1e-9) for r in results if r['pred_tau'][1] > 0.1]
    assert max(ratio) / min(ratio) < 1.1                                                     # the simulated arm draws what the model says
    peak = max(r['cur_units'].max() for r in results)
    hurt = FakeBus(model, q_rest, fault_units=0.6 * peak)                                     # overloads at 60 % of the heaviest pose's draw
    results, why = run(hurt, model, ladder, rest, nodes, edges, 0.2, 3.0, **quiet)
    assert why and 'hardware error' in why and len(results) < len(ladder)                    # stopped early, said why


def test_gripper_gap_mapping_matches_the_measured_ends_and_inverts():
    from gripper_tool import CLOSED_TICKS, OPEN_TICKS, gap_mm, ticks_for_gap
    assert abs(gap_mm(CLOSED_TICKS) - 20.0) < 1e-9 and abs(gap_mm(OPEN_TICKS) - 72.0) < 1e-9
    assert gap_mm(0) > gap_mm(300)                                    # open is the LOWER tick number on this gripper
    assert all(abs(ticks_for_gap(gap_mm(t)) - t) <= 1 for t in (-900, -400, 0, 400, 580))


def test_gripper_move_closes_stops_on_an_object_and_reports_its_width():
    from gripper_tool import SAFE_CLOSE, FakeGripperBus, describe, gap_mm, move
    quiet = lambda s: None  # noqa: E731
    free = FakeGripperBus(raw=-890, block=610)                                   # nothing in the fingers: it ends against the closed end
    res = move(free, SAFE_CLOSE, 120, quiet, hold=0.2)
    assert res['status'] in ('reached', 'stalled') and res['raw'] >= 590 and not free.on      # pressed on the closed end; released at the end
    obj = FakeGripperBus(raw=-890, block=200)                                    # something ~33 mm wide
    res = move(obj, SAFE_CLOSE, 120, quiet, hold=0.2)
    assert res['status'] == 'stalled' and 180 <= res['raw'] <= 210 and abs(gap_mm(res['raw']) - 33) < 3
    assert 'HOLDING something' in describe(res, SAFE_CLOSE)
    held = FakeGripperBus(raw=-890, block=200)
    move(held, SAFE_CLOSE, 120, quiet, hold=0.2, keep=True)
    assert held.on                                                               # --keep leaves it holding


def test_gripper_move_opens_and_refuses_to_enable_torque_at_an_unconfirmed_current():
    import pytest
    from gripper_tool import SAFE_OPEN, FakeGripperBus, GripperError, move
    quiet = lambda s: None  # noqa: E731
    bus = FakeGripperBus(raw=600, block=10 ** 6)
    res = move(bus, SAFE_OPEN, 120, quiet, hold=0.2)
    assert res['status'] == 'reached' and abs(res['raw'] - SAFE_OPEN) <= 15
    liar = FakeGripperBus(raw=-890, current_readback=1193)                        # servo reports the MAX goal current back
    with pytest.raises(GripperError):
        move(liar, 600, 120, quiet)
    assert not liar.on                                                           # torque was never enabled


def test_gripper_move_freezes_on_a_hardware_error_and_refuses_unready_servos():
    import pytest
    from gripper_tool import FakeGripperBus, GripperError, move, ready_problems
    quiet = lambda s: None  # noqa: E731
    bus = FakeGripperBus(raw=-890, block=10 ** 6, fault_after=8)
    res = move(bus, 620, 120, quiet, hold=0.2)
    assert res['status'] == 'aborted' and 'error' in res['why'] and res['raw'] < 400           # stopped early, did not keep driving
    assert ready_problems({'err': 0, 'mode': 3, 'volt': 12.0, 'temp': 30, 'raw': 0})            # wrong operating mode
    assert ready_problems({'err': 0, 'mode': 5, 'volt': 12.0, 'temp': 30, 'raw': 3000})         # outside the calibrated range
    with pytest.raises(GripperError):
        move(FakeGripperBus(raw=-890, fault_after=0), 620, 120, quiet)


def test_gripper_move_reports_the_peak_current_it_needed():
    from gripper_tool import SAFE_CLOSE, FakeGripperBus, describe, move
    res = move(FakeGripperBus(raw=-890, block=200), SAFE_CLOSE, 120, lambda s: None, hold=0.2)
    assert abs(res['peak_ma'] - 120 * 2.69) < 1 and 'peak current 323 mA' in describe(res, SAFE_CLOSE)     # the stall draws the goal current


def test_hand_eye_pixel_turns_back_to_the_raw_pixel_for_every_rotation():
    import cv2
    import numpy as np
    from hand_eye import raw_pixel
    w, h = 8, 4
    for deg, code in ((0, None), (90, cv2.ROTATE_90_CLOCKWISE), (180, cv2.ROTATE_180), (270, cv2.ROTATE_90_COUNTERCLOCKWISE)):
        raw = np.zeros((h, w), np.uint8)
        raw[1, 5] = 255                                                     # one marked raw pixel (row 1, column 5)
        up = raw if code is None else cv2.rotate(raw, code)
        v, u = np.argwhere(up == 255)[0]
        assert raw_pixel(int(u), int(v), w, h, deg) == (5, 1), deg


def test_hand_eye_a_point_straight_ahead_of_the_camera_lies_ahead_of_the_gripper():
    import numpy as np
    from hand_eye import load, to_arm
    from omx_fk import Arm
    arm, position = Arm(), load()[0]
    q = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0)               # all zero: the gripper axis is horizontal, +x of the arm
    ee = arm.xyz('end_effector_link', q)
    p = to_arm(arm, q, np.array([0.0, 0.0, 0.5]), position)                 # 0.5 m along the optical axis
    assert np.allclose(p - ee, [0.5 - 0.065, 0.0, 0.05], atol=1e-6)         # forward 0.435 m, centred, 50 mm above the pad centre
    p_up = to_arm(arm, q, np.array([0.0, 0.1, 0.5]), position)              # raw image DOWN is the gripper's UP (upside-down mount)
    assert p_up[2] > p[2] + 0.09


def test_touch_test_recovers_a_known_camera_offset_error_and_a_depth_scale_error():
    import numpy as np
    from hand_eye import R_G_C, load
    from omx_fk import Arm
    from touch_test import analyse
    arm, assumed = Arm(), load()[0]
    true_t = assumed + np.array([0.012, -0.006, 0.004])                  # the camera is really 12 mm / -6 mm / 4 mm away from what we assumed
    target = np.array([0.30, 0.02, 0.0])
    touch_q = dict(joint1=0.0, joint2=-0.2, joint3=0.3, joint4=0.2)      # any pose: only its FK position is the "touched" point
    touch_pos = arm.xyz('end_effector_link', touch_q)
    views = []
    for q in (dict(joint1=0.1, joint2=0.3, joint3=-0.6, joint4=0.1), dict(joint1=-0.3, joint2=0.5, joint3=-0.9, joint4=0.2),
              dict(joint1=0.4, joint2=0.2, joint3=-0.4, joint4=0.0), dict(joint1=0.0, joint2=0.6, joint3=-1.1, joint4=0.3)):
        m = arm.pose('end_effector_link', q)
        p_g = m[:3, :3].T @ (touch_pos - m[:3, 3])                       # the touched point seen from the gripper at this view pose
        views.append({'q': q, 'p_opt': R_G_C.T @ (p_g - true_t)})        # what a perfect camera at the TRUE offset would measure
    res = analyse(arm, touch_q, views, assumed)
    assert np.allclose(res['mean_err_grip_mm'], (assumed - true_t) * 1000, atol=1e-6)
    assert np.allclose(res['suggested_position_m'], true_t, atol=1e-9)    # the corrected offset recovers the truth
    assert max(res['std_err_grip_mm']) < 1e-6                              # a pure offset error is the same in the gripper frame at every view
    assert target is not None


def test_touch_test_marker_detection_and_depth_window():
    import cv2
    import numpy as np
    from touch_test import MARKER_PX_DICT, depth_at, detect_marker
    if not hasattr(cv2.aruco, 'ArucoDetector'):      # touch_test needs OpenCV >= 4.7; older apt builds (CI) lack it
        import pytest
        pytest.skip('OpenCV is older than 4.7 (no cv2.aruco.ArucoDetector)')
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_PX_DICT)), 0, 200)
    canvas = np.full((480, 640), 255, np.uint8)
    canvas[100:300, 300:500] = marker                                    # marker centre at (399.5, 199.5)
    img = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    u, v = detect_marker(img)
    assert abs(u - 399.5) < 2 and abs(v - 199.5) < 2
    u2, v2 = detect_marker(cv2.rotate(img, cv2.ROTATE_180))             # the camera is upside down: detection must not care
    assert abs(u2 - (640 - 1 - 399.5)) < 2 and abs(v2 - (480 - 1 - 199.5)) < 2
    assert detect_marker(np.full((480, 640, 3), 255, np.uint8)) is None
    depth = np.full((480, 640), 520, np.uint16)
    depth[198:201, 398:401] = 0                                          # a few invalid pixels at the centre are ignored
    assert depth_at(np.where(depth == 0, 0, 520).astype(np.uint16), 399, 199) is not None
    big_hole = depth.copy()
    big_hole[190:210, 390:410] = 0                                       # no valid pixel in the whole window: no reading
    assert depth_at(big_hole, 399, 199) is None
    assert abs(depth_at(depth, 399, 199) - 0.52) < 1e-9 and depth_at(np.zeros((480, 640), np.uint16), 100, 100) is None


def test_box_depth_takes_the_nearest_dense_band_not_a_percentile():
    import numpy as np
    from hand_eye import box_depth_m
    d = np.zeros((720, 1280), np.uint16)
    d[200:260, 600:700] = 3000                    # the wall seen through a clear bottle (raw frame; the box below is turned 180 deg)
    d[220:224, 640:660] = 450                     # ~80 px: the bottle surface is a small minority and below min_pixels...
    d[230:240, 640:660] = 450                     # ...with this, 280 px in total
    d[210, 650:655] = 300                         # 5 px of speckle nearer than the bottle
    box = (1279 - 700, 719 - 260, 1279 - 600, 719 - 200)
    assert box_depth_m(d, box, 1280, 720, 180) == 0.45
    assert box_depth_m(np.zeros((720, 1280), np.uint16), box, 1280, 720, 180) is None


def test_box_depth_threshold_scales_with_the_depth_image_size():
    import numpy as np
    from hand_eye import box_depth_m
    d = np.zeros((400, 640), np.uint16)           # half-size depth: a quarter of the pixels
    d[111:144, 300:350] = 3000                    # the box (raw rows 200-260 of 720) lands on rows 111-144 of 400
    d[120:130, 310:330] = 450                     # 200 px: under 150 would pass anyway, but 150 unscaled would also pass; 41 is what must hold
    box = (1279 - 700, 719 - 260, 1279 - 600, 719 - 200)
    assert box_depth_m(d, box, 1280, 720, 180) == 0.45


def test_fixed_camera_point_is_ahead_right_pixels_go_negative_y_and_down_pixels_go_negative_z():
    from hand_eye import fixed_cam_point
    k, xyz = (600.0, 600.0, 320.0, 240.0), (0.0, -0.08, -0.02)
    assert [round(float(x), 3) for x in fixed_cam_point(320, 240, 0.5, k, xyz)] == [0.5, -0.08, -0.02]
    p = fixed_cam_point(420, 340, 0.6, k, xyz)         # 100 px right and 100 px down at 0.6 m = 0.1 m each way
    assert [round(float(x), 3) for x in p] == [0.6, -0.18, -0.12]


def test_only_a_side_cut_off_box_is_unusable_top_and_bottom_are_fine():
    from hand_eye import clipped_at_side
    assert clipped_at_side((200, 100, 300, 400), 640) == ''
    assert clipped_at_side((0, 100, 80, 400), 640) == 'left' and clipped_at_side((560, 100, 639, 400), 640) == 'right'
    assert clipped_at_side((200, 0, 300, 479), 640) == ''          # a tall bottle at close range fills the 42 deg vertical view
