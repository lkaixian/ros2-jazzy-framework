# Arm bring-up: what exists, what must still work (started 2026-11-25)

OpenMANIPULATOR-X (Dynamixel XM430 IDs 11-15) on an OpenCR in USB pass-through mode, mounted ~80 cm above the Kobuki base, plus an OAK-D-LITE on the
gripper ("arm eye") and a RealSense next to the arm base. Everything below was done by hand-guiding and short, supervised moves. Tools are in `tools/arm/`.

## Works (evidence in brackets)
- Servo bus, ROS driver (`open_manipulator_bringup`, port by-id), slow 45 s moves, graceful stop: `tools/arm/arm_up.sh`, `arm_tool.py park` [real arm].
- Hand-taught collision-free map of the whole arm (`teach.py --map` -> `~/arm_free_space.npz`, baseline copy `~/arm_free_space_baseline.npz`), real joint ranges, contact points
  (`~/arm_points.yaml`), taught roadmap routes (`route.py`), map-based path check (`path_check.py`) used by `park` and the tour [offline + mock arm; hand data is real].
- Gripper (ID 15): calibration `src/framework_bringup/param/arm_gripper.yaml`; guarded `gripper_tool.py open|close|goto --gap` [real gripper: close, open, goto 45 mm].

## MUST still work before the arm does tasks
1. **Grip force calibration.** The gripper has NO force sensor; force = motor current (Goal Current caps it, 2.69 mA/unit). Calibrate newtons vs current with a kitchen scale
   between the pads (e.g. 60/100/150/200/250 units), then choose a grip current per object. (`gripper_tool.py calibrate` not built.)
2. **Real load test** `tools/arm/load_test.py --go` (support under the gripper, OAK and cable mounted): checks the gravity torque model against measured servo current. The model says gravity <= 34 %
   of stall everywhere in the free space, but the arm overloaded in the tour for other reasons (see lessons), so measure. RealBus code has only run on a simulated bus.
3. **Gripper on a real object:** close on a known object, check the stall detection, the reported width (+-3 mm) and the peak current.
4. **Health monitor + recovery in ROS:** the driver's own `dxl_state` said "no error" while servos 11-13 had overload (error 32); the only ROS-side sign was `RX_PACKET_ERROR ... Hardware error`
   in the driver log. Needed: watch tracking error, measured current (verify `/dynamic_joint_states` carries Present Current), and that log line; on error freeze, reboot ONLY errored servos
   (`reboot_dxl`), re-enable torque, retry once slower, then ask a human.
5. **MoveIt config:** group `arm`, ee link `end_effector_link`, frame names are still blank in `extensions.yaml`; URDF joint limits are too tight (real: j2 -2.12..+2.06, j3 -2.10..+1.50, j4 -1.66..+2.13)
   so MoveIt rejects the rest pose. Add keep-outs for the RealSense, mic boom and control board (positions in `~/arm_points.yaml`).
6. **Gripper in `arm_extension`:** stock open/close states (+0.019 / -0.010 m) are the OPPOSITE direction of this unit (open = lower ticks). Set `gripper_open ~ -0.050`, `gripper_closed ~ -0.026` (driver joint units)
   or correct the conversion in a description copy; verify the controller does not clamp them.
7. **Hand-eye: measured by hand (signs confirmed by the user), numbers NOT verified.** Camera 65 mm behind and 50 mm above the pad centre, centred, upside down (`param/arm_hand_eye.yaml`, `param/oak.yaml`,
   `tools/arm/hand_eye.py`). Verify with `tools/arm/touch_test.py` (marker card: `--make-marker`) before any grasp; then set `verified: true`. The OAK is on USB 3 now.
8. **Hardware:** a hardware e-stop on the 12 V line (software is not one); route the OAK/RealSense cables clear of the arm's sweep; the folded gripper overhangs ~11 cm ahead of the Kobuki front at ~87 cm height (invisible to bumpers and lidar).
9. **Re-survey** (`teach.py --map --fresh`) whenever the RealSense, mic or board moves; promote a better map by copying it over the baseline.
10. Update `docs/status.md`: arm and gripper moves were verified on the real hardware as listed above, nothing else.

## Lessons (each one cost something)
- The tour's turn stalled joint 1 against the RealSense (+1.28..+1.87 rad) for ~170 s -> overload error 32 on servo 11; servos 12, 13 later; the arm fell to the joint-2 stop (+2.06).
- "Driver stopped" does NOT mean "torque off": servos 14 and 15 were still holding. Read the servo's own Torque Enable.
- A straight joint-space park from an arbitrary pose nearly hit the control board. Moves must be checked against the taught free-space map or follow taught routes.
- rclpy's own SIGINT handler shuts the context down, so a Ctrl+C `cancel` was never sent: init with `SignalHandlerOptions.NO` and confirm the stop.
- Gripper Goal Current is 1193 (max) after a power cycle: set and read back <= 250 BEFORE enabling torque.
- A joint turned more than one turn by hand leaves the servo in a multi-turn count: turn it back inside one turn before the driver starts.
- Hand-guided "torque map" is a model estimate, never a measurement (torque was off).

## Added 2026-11-25 (later): GUI and perception
- `tools/arm/arm_gui.py` (+ `arm_gui.html`): read-only browser monitor (`http://127.0.0.1:8088`): arm skeleton over the free-space map (top/side), joints vs seen ranges, gripper gap/current history,
  servo health, gravity-load estimate, OAK view rotated 180 deg in software with optional YOLO (`--yolo yolo26s.pt`) and distance ahead. Uses ROS `/joint_states` when the driver runs, else reads the servos
  directly. `bus_lock.py` (flock) serialises tools and the GUI on the serial port; `telemetry.py` streams a running tool's readings (gripper_tool, load_test) to the GUI over UDP 8089.
- Models are now YOLO26 (objects/pose `yolo26s`, face-embedding `yolo26n-cls`, face detector `yolo26n-face.pt` user-supplied). Measured here: yolo26s 5 ms/frame, 92 MiB.
- OAK on USB 3 (SUPER): RGB 1280x720 ~19 Hz, depth aligned ~21 Hz, intrinsics fx 990.7. The image is upside down (rotate 180 deg in software: driver/relay still TODO for the detection node);
  a cable crosses the field of view.
- Still TODO for grasping: hand-eye calibration, object 3D pose, IK for a pre-grasp inside the free-space map, the pick sequence.
- `rotate` parameter on all YOLO nodes (shared base class); `tools/arm/arm_eye.sh` starts the OAK driver (hand-eye TF) + `object_detection_node` with `rotate:=180` -> `/arm_eye/objects`, `/arm_eye/image` (upright).
  The driver's TF with the measured pose was checked against my own derivation on the mock arm (optical z = gripper +x, optical y = gripper +z, optical x = gripper +y).

## Resume here (end of the 2026-11-25 session)
State: branch `feat/llm-e4b`, pushed through `d33bd6c` (perception YOLO26 + rotate; arm tooling; the other agent's dashboard). Nothing runs on a robot except what is listed under "Works".
Order I would continue in (each needs you at the arm unless marked software-only):
1. `tools/arm/touch_test.py` (print `~/arm_marker_4x4_id0.png`, ~50 mm): verifies the hand-eye numbers; then set `verified: true` in `param/arm_hand_eye.yaml`.
2. Gripper on a real object (`gripper_tool.py close --go`, a 30-50 mm soft object): stall + width + peak current. Then the force calibration with a scale (item 1 above).
3. `tools/arm/load_test.py --go --k 2 --hold 4` first (support under the gripper, OAK and cable mounted). Dry run first: `load_test.py`.
4. Software-only: detection pixel + depth -> arm-frame point (`hand_eye.py: upright_pixel_to_arm` exists), IK for a pre-grasp pose that stays inside the free-space map (`path_check.py`), the pick sequence
   (approach on a taught route, guarded gripper, retreat), ROS health monitor + servo reboot/retry, MoveIt limits/keep-outs.

Commands (source `env.sh` first; export `CYCLONEDDS_URI=file://$PWD/tools/net/cyclonedds_local.xml` in shells opened before the ~/.bashrc fix):
- Arm driver + graceful stop: `tools/arm/arm_up.sh` (parks on SIGTERM; refuses outside the map). Stop it before any tool that reads the servos directly.
- Mapping by hand (torque off, driver stopped): `teach.py --map` (continuous) or `--range`; map `~/arm_free_space.npz`, the checker uses the frozen `~/arm_free_space_baseline.npz` first.
- `path_check.py --from rest --to home`, `route.py --from a,b,c,d --to rest`, `load_map.py`, `gripper_tool.py status|range|open|close|goto --gap MM [--go]`.
- Camera: `tools/arm/arm_eye.sh` (OAK driver + YOLO26s with `rotate:=180`); monitor: `arm_gui.py [--yolo yolo26s.pt]` -> http://127.0.0.1:8088.
Data (in your home, not in git): `~/arm_points.yaml` (named contact points), `~/arm_range.csv`, `~/arm_stream*.csv`, `~/arm_free_space*.npz`, `~/arm_load_map.npz`, `~/arm_gripper.yaml`, `~/arm_maps/*.png`.
Gotchas: the open_manipulator launch defaults to `/dev/ttyUSB0`, which is the LIDAR here: always pass `port_name:=` (arm_up.sh does). A joint hand-turned past one full turn must be turned back before the driver starts.
The other agent's uncommitted work lives in the same tree (dashboard, `tools/camera`, `99-realsense-libusb.rules` untracked): do not revert it.

## Arm integration into the BT / LLM pipeline (2026-11-25, software and simulation only)
What exists now: `src/arm_control` (FK, free-space map, taught routes, hand-eye, IK limited to the taught range, `motion.plan_route`, `pick`, `health`), `arm_extension` (nodes ArmPark, MoveArmToPose, OpenGripper, CloseGripper, PickObject, PlaceObject; no MoveIt) and `arm_health_node` (`arm/health`, optional servo reboot). Macros `pick_object`, `put_down`, `move_object`; `llm_knowledge.md` and the GPSR set know the arm; orchestrator `has_arm` (default true). Tests: `src/arm_control/test`, `tools/integration/arm_sim_test.py` (own ROS domain 91: the real nodes against a fake arm).
Safety shape: `live:=false` by default (plans and checks only); every leg is checked against the free-space map and taught range before the first one runs; 0.03 rad/s; cancel / tracking error holds the arm; gripper refused while its Goal Current register > 250; live motion needs a clean `arm/health`.

Validate on the arm, in this order (each needs you at the arm, hand ready):
1. Touch test, then `verified: true` in `arm_hand_eye.yaml`.
2. Names on the real driver: `/gripper_controller/gripper_cmd`, `/dynamixel_hardware_interface/{get_dxl_data,dxl_state,reboot_dxl}`, `/arm_controller/controller_state`; does the gripper controller honour `max_effort`? Set the gripper Goal Current (<= 250) first.
3. `PickObject` with `live:=false` on a real detection (plans, checks, moves nothing): are the target point and the two poses sensible?
4. Gripper on a real object, then `live:=true` with one slow pick.
Known gaps: the OAK-D-Lite cannot measure depth much closer than ~20 cm, so the detection pose must stand far enough from the object (the pre-grasp stand-off is measured from the detection, not the camera); the arm stays at the pre-grasp pose while the base drives (no carry pose measured); `places` is empty (PlaceObject refuses until you add `name:x,y,z`); `object_width_mm` empty; workspace limits and `grasp_depth_offset_m` are estimates; servo reboot (`auto_reboot`) never tried on hardware and drops torque on ALL servos, so it is off by default and gated on the rest pose; the SLM needs retraining (catalog gained 6 nodes).
