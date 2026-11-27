# Startup commands (plain version)

Every terminal starts the same way (`tools/start.sh` does this for you):
```
cd ~/ros2-jazzy-framework && source env.sh
source ~/ros2_ws/install/setup.bash        # only where the Kobuki driver runs
```

## The short way: `tools/start.sh`
One terminal, every program logs to `/tmp/robot_start/<name>.log`, **Ctrl+C stops everything it started**. It never turns on real motion by itself.
```
tools/start.sh base                    # e-stop reader + Kobuki driver
tools/start.sh arm [--oak]             # RealSense + object detector + arm extension (+ the OAK eye)
tools/start.sh task gpsr               # base + the GPSR stack, dry run (nothing commands hardware)
tools/start.sh task gpsr --move        # same, dry_run:=false: the base and extensions WILL move (asks you to type go)
tools/start.sh stop                    # stop everything this project started
tools/start.sh task gpsr --list        # print the commands, start nothing (works for every profile)
```
Task names: `gpsr egpsr inspection receptionist restaurant help_me_carry storing_groceries clean_the_table give_me_a_hand robocup`.
The LLM server starts by itself with the stack (`param/llama_server.yaml`, `start: true`, E4B at 32k context).

## A. Robot tasks without the arm (GPSR, inspection, receptionist, restaurant, help me carry)
```
tools/start.sh task gpsr                       # wait for "running"; first start downloads/loads the LLM, give it a minute
ros2 run framework_bringup dash                # second terminal: what is online / busy / in error
```
Give it a task (speech does this on its own; by hand):
```
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'go to the kitchen'}"
ros2 service call /bt_executor/run std_srvs/srv/Trigger      # run the stored plan (auto_run is off); in a dry run nothing moves
ros2 service call /bt_executor/halt std_srvs/srv/Trigger    # stop the plan
```
Real motion: restart with `--move` only with the floor clear and your hand on the e-stop.

## B. The arm picks something
Terminal 1 (own terminal, on purpose: **Ctrl+C here parks the arm slowly, let it finish**):
```
tools/arm/arm_up.sh          # driver. Wait ~10 s: a clean log has no "Timeout" lines
```
Terminal 2:
```
tools/start.sh arm           # add --oak for the gripper camera
```
Check before touching anything:
```
ros2 topic echo arm/health --once --field data | cut -c1-300        # "ok": true, volts of every servo above 11.0
ros2 topic hz /objects                                              # a rate while an object is in front of the RealSense
ros2 run rqt_image_view rqt_image_view /detections/image            # the picture with boxes and labels
```
Dry pick (plans only, prints `DRY: <label> at [x, y, z] ...`; replace `bottle` with the label the detector shows):
```
ros2 action send_goal /arm_extension/execute bt_pipeline_msgs/action/ExecuteNode "{node_id: PickObject, port_names: [label], port_values: [bottle]}"
ros2 action send_goal /arm_extension/execute bt_pipeline_msgs/action/ExecuteNode "{node_id: FaceObject, port_names: [label], port_values: [bottle]}"   # how far to turn the base
```
Settings and where they are: `docs/arm-parameters.md` (regenerate with `python3 tools/gen_arm_params.py`).

### Real arm motion (live)
Only with: rigid narrow object 22-32 cm from the arm axis, nothing else in reach, health ok **with the supply holding above 11 V while it moves**, someone beside it.
```
sed -i '92s/live: false/live: true/' src/framework_bringup/param/extensions.yaml     # the master switch (line 92 of that file)
pkill -f "[a]rm_extension_node"                                                      # it respawns in ~5 s with the new value
<send the PickObject goal above>         # Ctrl+C in that terminal cancels and holds the arm
```
Afterwards, with a hand under the object:
```
ros2 action send_goal /arm_extension/execute bt_pipeline_msgs/action/ExecuteNode "{node_id: OpenGripper, port_names: [], port_values: []}"
ros2 action send_goal /arm_extension/execute bt_pipeline_msgs/action/ExecuteNode "{node_id: ArmPark, port_names: [], port_values: []}"
sed -i '92s/live: true/live: false/' src/framework_bringup/param/extensions.yaml; pkill -f "[a]rm_extension_node"
```
Never cut the arm's 12 V while it is raised (torque off: it falls). If health refuses to let you release or park (low supply): support the arm by hand,
`ros2 param set /arm_extension_node require_health false` for that recovery only, then `... require_health true`.

Stop: Ctrl+C in terminal 2, then Ctrl+C in terminal 1 (it parks the arm; wait for it).

## C. Gripper test (driver STOPPED: Ctrl+C `arm_up.sh` first)
```
python3 tools/arm/gripper_tool.py status                       # mode 5, no error, voltage >= 11, goal current well under 250
python3 tools/arm/gripper_tool.py open --go --current 120      # nothing in the fingers
python3 tools/arm/gripper_tool.py close --go --current 80 --hold 3   # on the object; prints the width and the peak current
```

## D. Arm tasks (egpsr, storing_groceries, clean_the_table, give_me_a_hand)
Same as B first (terminal 1 `arm_up.sh`, `tools/arm/arm_eye.sh`), then `tools/start.sh task storing_groceries`. Arm places are still empty, so the put-down steps will say so.

## E. Mapping a room (not yet done on the real robot)
```
tools/map_session.sh room2            # drive; quit teleop to save ~/maps/room2 and stop everything
tools/map_session.sh --nav room2      # navigate on that map
```

## F. When something is stuck
```
tools/stop_all.sh -n        # list what is running
tools/stop_all.sh           # stop it all (SIGINT, then TERM, then KILL)
ros2 run framework_bringup check_params    # are the parameter files consistent with the nodes
```

## Two different switches (easy to confuse)
| switch | controls | where |
|---|---|---|
| `dry_run:=false` | behaviour-tree plans really drive the base and extensions | launch argument (`--move`) |
| `live: true` | the **arm node** really moves the arm | `extensions.yaml` line 92 |
A direct goal to `/arm_extension/execute` ignores `dry_run`; only `live` matters for it.
