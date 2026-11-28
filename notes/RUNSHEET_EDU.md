# Run sheet: RoboCup@Home Education (Carry My Luggage, Find My Mates, Receptionist)

Commands only. Every block says WHO/WHERE it runs. Repo root = `~/ros2-jazzy-framework`. Every terminal starts with:
```bash
cd ~/ros2-jazzy-framework && source env.sh && source ~/ros2_ws/install/setup.bash   # env.sh = system Python, ROS, install/, DDS config; ros2_ws = Kobuki + lidar drivers
```
Nothing here has run on the real robot yet. **Stop rule: hand on the e-stop for every live step; the first thing that surprises you, press it.** `dry_run:=true` is the default everywhere: the robot does not move until a command says `dry_run:=false`.

Emergency / stop (any time): hardware e-stop, then
```bash
ros2 service call /bt_executor/halt std_srvs/srv/Trigger     # halts the running plan
tools/stop_all.sh                                            # stops everything this project started
ros2 run framework_bringup teleop                            # w/a/s/d, space stop; the first key halts the plan and has priority
```

## 0. Laptop, the day before (no robot)
```bash
git checkout feat/llm-e4b && git pull        # the branch must be pushed first from the robot laptop: git push -u origin feat/llm-e4b
colcon build --symlink-install 2>&1 | tail -3 && source install/setup.bash
ls models/reid/osnet_ain_x1_0.onnx models/hand/*.onnx models/genderage/genderage.onnx models/face_parsing/*.pt yolo26s.pt yolo26s-pose.pt yolo26s-face.pt   # all must exist (models/ and *.pt are not in git: copy them from the laptop that has them)
python3 -m pytest src/*/test -q -p no:cacheprovider --deselect src/object_detection/test/test_flake8.py --deselect src/object_detection/test/test_pep257.py   # about 1 minute, all green (1285 passed)
ros2 run framework_bringup check_params                       # 26/26 consistent
python3 tools/integration/follow_sim_test.py                  # follower in the simulator: PASS lines (2 min)
FOLLOW_RETRACE=1 python3 tools/integration/follow_sim_test.py # same + the way back (retrace)
python3 tools/integration/help_me_carry_sim_test.py           # whole luggage-style macro in the simulator
```

## 1. At the venue: bring-up (terminal A: drivers, terminal B: checks)
```bash
# A
tools/start.sh base                      # e-stop reader + Kobuki driver (add --list to only print)
ros2 launch framework_bringup sensors.launch.py   # RPLIDAR + RealSense
# B
ros2 topic hz /scan                      # about 7-10 Hz
ros2 topic hz /camera/camera/color/image_raw      # about 15 Hz
ros2 topic echo --once /odom | head -5   # odometry alive
ros2 run framework_bringup preflight     # PASS/WARN/FAIL list: fix every FAIL before going on
```
Battery: Kobuki below 13.2 V halts plans. Arm supply: the gate is 10.5 V; watch it in `python3 tools/arm/arm_gui.py` while the arm moves.

## 2. Measurements nobody can do for you (5 minutes)
```bash
# robot on flat floor, floor in view, sensors.launch.py running:
python3 tools/camera/floor_plane.py --frames 10      # camera height, pitch, roll -> put z / pitch / roll into src/framework_bringup/param/navigation.yaml (base_to_camera)
# tape measure: lens height above the floor (m) -> cam_height in src/framework_bringup/param/extensions.yaml (pid_extension_node, default 0.8)
```
Then turn on depth obstacles (the small floor object and the hard-to-see object need them), with a spotter walking beside the robot the first time:
add `depth_obstacles:=true` to the `navigation.launch.py` line in step 3.

## 3. Map the arena (once, then reuse) and set the places
```bash
tools/map_session.sh arena1              # drive with the keys it shows, come back to the start spot, quit teleop (Ctrl-C): saves ~/maps/arena1.yaml
tools/map_session.sh --nav arena1 0,0,0  # navigate on it: robot standing where the mapping run started (x,y,yaw)
```
Edit `src/framework_bringup/param/places.yaml` with the REAL poses (read them off rviz "Publish Point" or `ros2 topic echo /amcl_pose` with the robot standing there): `start`, `door`, `living_room`, and the three Find My Mates checkpoints (`coffee_table`, `dining_table`, `shelves` are placeholders: rename or keep the names and fix the numbers). No rebuild needed (symlink install).
```bash
ros2 run framework_bringup preflight     # re-check; then test one drive:
python3 tools/edu/run_macro.py go_to_place place=living_room     # needs the full stack of step 4 running, dry_run:=false for it to move
```

## 4. Bench tests, one at a time (robot stationary on a stand or with the floor clear)
Start the stack for a task WITHOUT motion (dry run): `tools/start.sh task edu_reception` (swap the name for the other tasks). Logs: `/tmp/robot_start/*.log`.
```bash
# speech: say "start the task", then a name, a drink
ros2 topic echo /stt/transcript                         # what the recogniser heard (names/drinks should come out right; Whisper is primed with param/vocab.yaml)
ros2 topic echo /bt/task_spec --once                    # appears when "start the task" is heard (the task spec the launch file carries)
# people: stable ids and the memory board
ros2 run rqt_image_view rqt_image_view /detections/reid_memory      # rows = people it remembers; walk out and back in: same row
ros2 run rqt_image_view rqt_image_view /detections/image            # live boxes with the stable ids
# the five-finger stop sign (camera up, stand 1-2 m away)
python3 tools/hand/check.py                             # hold up five / three / no fingers; saves /tmp/hand_check.jpg
# vision-language description (the planner's llama-server must say "loaded multimodal model" in /tmp/robot_start/*.log)
python3 tools/edu/run_macro.py describe_person          # says what colour it sees; the Receptionist uses the full version
```
Enrol the HOST once (the host sits in front of the camera, face visible, 1.5-3 m):
```bash
python3 tools/edu/run_macro.py enroll_host host=John    # use the host name announced for the test (Liam, Noah, ... any word works)
```

## 5. Rehearsals: dry run first, then live. Same three steps for each task.
Pass the day's facts on the command line (`start_spec` replaces the default; nothing else needs editing).

**Receptionist**
```bash
# dry run: robot does not move, it speaks and decides (host facts as announced 2 h before the test: name, drink, he/she)
ros2 launch framework_bringup edu_reception.launch.py start_spec:='{"goal":"edu reception task","steps":[{"macro":"hric_round","args":{"host":"John","host_drink":"milk","host_pron":"he","living_place":"living_room"}}]}'
# live (floor clear, e-stop in hand): same command + dry_run:=false, or tools/start.sh task edu_reception --move  (type go)
# arm_extension runs in this launch (live: false = it only plans, prints DRY:): the plan names arm nodes and the validator refuses a plan whose nodes nobody advertises.
# The seat pointing needs the arm driver too (see below) and extensions.yaml live: true
# then say: "start the task"; a volunteer steps up to the robot at the door; answer the name and drink questions; follow the robot
```
**Find My Mates**
```bash
ros2 launch framework_bringup edu_mates.launch.py start_spec:='{"goal":"edu mates task","steps":[{"macro":"mates_round","args":{"cp1":"coffee_table","cp1_words":"near the coffee table","cp2":"dining_table","cp2_words":"near the dining table","cp3":"shelves","cp3_words":"near the shelves"}}]}'
# add dry_run:=false for the live run; say "start the task"; people stand at the checkpoints; the robot reports at `start`
```
**Carry My Luggage**
```bash
ros2 launch framework_bringup edu_luggage.launch.py            # dry run (add dry_run:=false for live; the bag handover needs the arm driver and live: true)
# say "start the task"; point at a bag; hand it the handle (arm) or skip; say "follow me"; walk; stand still and say "stop" (or hold a hand up) at the car
```
Arm (only for the bag handover or seat pointing; never while someone stands within reach):
```bash
tools/arm/arm_up.sh                                              # terminal C: Ctrl+C parks the arm slowly; keep a hand on the 12 V supply
tools/arm/arm_eye.sh                                             # terminal D (OAK eye, only if used)
# src/framework_bringup/param/extensions.yaml: arm_extension_node live: true  (false = it only plans and prints DRY:), and dry_run:=false for the launch
# pointing speed: point_rate 0.08 rad/s (validated rate is 0.03): raise in steps 0.03 -> 0.05 -> 0.08 and watch for "joint tracking error"
```

## 6. Record and report (every rehearsal)
```bash
tools/record_run.sh reception_try1                               # topics that matter -> ~/robocup_bags/reception_try1_<time>/ (add --images for video; large)
ros2 run framework_bringup dash --once                           # one frame of every node's state and the running tree
ros2 topic echo --once /task/state                               # idle / planning / ready / running / done / failed
```
Send back: `/tmp/robot_start/*.log`, the bag folder, the last 20 lines of the terminal, and one sentence of what you saw.

## 7. Competition day, in this order
1. `tools/start.sh base`; `ros2 launch framework_bringup sensors.launch.py`; `ros2 run framework_bringup preflight` (no FAIL).
2. `tools/map_session.sh --nav arena1 <x,y,yaw of the announced start>` only if the arena map is the one you made; if the arena changed, map again (step 3) and fix `places.yaml`.
3. Receptionist: host facts announced 2 h before -> `enroll_host`, then step 5 command with `dry_run:=false` (`--move`).
4. Find My Mates: checkpoints announced -> edit `places.yaml`, step 5 command.
5. Carry My Luggage: step 5 command; operator stands in front, points; the stop word is "stop" or a raised hand while standing still.
6. Skip rule: if a step keeps failing, let it skip (the macros already fall through); never retry the same subgoal by hand.

## 8. Settings you may want to change on the day (all in `src/framework_bringup/param/`, no rebuild)
| What | File: key | Default |
|---|---|---|
| Names / drinks the recogniser expects | `vocab.yaml`: names, drinks | the 15 names, 10 drinks; any other word still works |
| Same-person threshold (splits one person into two: lower it; merges two people: raise it) | `perception.yaml`: person_detection_node.reid_sim | 0.64 |
| Switch re-identification off | `perception.yaml`: reid_model: "" | auto |
| Stop word(s) | the macro argument `words` (`follow_until_told`) | stop, we are here, we have arrived |
| Camera height for the face check | `extensions.yaml`: pid_extension_node.cam_height | 0.8 m (measure it) |
| Arm pointing speed | `extensions.yaml`: arm_extension_node.point_rate | 0.08 rad/s |
| Arm supply gate | `src/arm_control/arm_control/health.py` V_MIN | 10.5 V |
