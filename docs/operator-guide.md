# Operator guide

Everything you type while running the robot, in one place. Always `source env.sh` first. **`bt_executor` starts in dry-run: plans are only
logged. Never pass `dry_run:=false` for a real robot without a person ready at the hardware e-stop.**

## Start
```
ros2 launch launch/framework.launch.py image_topic:=/camera/image_raw start_llama_server:=false dry_run:=true
llama-server -m models/llm/gemma-4-E2B-it-Q4_0.gguf --host 127.0.0.1 --port 8081 -c 6144 -np 1 -ngl 99 --reasoning off   # E2B on the 4 GB GPU
ros2 run framework_bringup preflight                    # PASS/WARN/FAIL: key, SROS2, mux, executor dry-run, LLM, /scan, TF, clock, disk
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'go to the door and say hello'}"
ros2 service call /bt_executor/run std_srvs/srv/Trigger       # run the validated plan (dry-run: logged only)
```

## Robot mode, recovery and mission policy (`robot_mode_node`, `bt_orchestrator/mode.py`)
`ros2 topic echo robot/mode` (latched JSON, 2 Hz): `init` -> `ready` <-> `active` (a task runs) / `teleop` (a human drives); `safe_stop` on a bumper/cliff
latch (from `cmd_vel_mux`) or a critical battery; `init_failed` when `required` signals (executor, llm, optionally scan) are not heard within 60 s.
`safe_stop` and `teleop` halt a running plan. `run_ok` is true only in `ready`/`active`. `ros2 service call /robot/reset std_srvs/srv/Trigger` clears
`init_failed`; it is refused while a hazard is present (first clear the cause, bumper/cliff through `safety_reset`). `mission_budget_sec > 0` halts a task that runs too long.
**Enforcement is off by default**: `pipeline.launch.py mode_gate:=true` makes `bt_executor` refuse `run`/`auto_run` unless a fresh `robot/mode` says `run_ok` (no mode node = refused).
**Recovery** (`max_repairs` = how many recoveries per task, 0 = none): after a failed run the policy picks by where it broke: navigation `retry, replan, go_home`;
detection `retry, replan, ask_human`; speech `retry, ask_human`; anything else `replan, ask_human`; timeout `go_home` once; a crashed tree or a human halt = `abort`.
`go_home`/`ask_human` run fixed specs (no LLM). The table is a starting point (`RecoveryPolicy.TABLE`): tune it with real runs.
Tested: `test_mode.py` (logic), `tools/integration/mode_test.py` (real nodes, gate, latch) and `tools/integration/recovery_test.py` (a live executor with fake extensions that fail on command: retry, replan, go_home, ask_human). Not tried on the robot.

## Groot2 (the BehaviorTree.CPP GUI)
`ros2 launch framework_bringup pipeline.launch.py groot2:=true` (or `groot2: true` in `param/pipeline.yaml`) serves each running tree on port 1667.
Start `~/Groot2/groot2.sh`, Monitor tab, connect to `127.0.0.1`: node colours follow the run. The server lives for one plan, so reconnect for the next one.
To edit trees with the extension nodes in the palette, set `nodes_model_path: /tmp/bt_nodes_model.xml` (written at each run) and import it in Groot2.
Tested: `tools/integration/groot2_test.py` (port opens, palette written); not tried with the Groot2 GUI itself.

## Dashboard (`ros2 run framework_bringup dash`)
Shows domain/SROS2 state, LLM endpoints with link type, Kobuki battery, task state, **the plan tree with live node status**
(`>` running, `+` done, `x` failed, `-` halted, `.` not reached), cluster jobs and every node ONLINE/BUSY/ERROR/OFFLINE.
Keys: `j`/`k` or arrows select a node, `r`/`R` restart it (graceful/force), `s`/`S` stop the whole system, `w`/`W` restart it, `h` halt the
task, `t` hide/show the tree. `s S w W` ask y/n. `dash --logs --level warn --node llm_bridge` streams `/rosout`; `dash --once` prints one frame.

## Take over / stop
- `ros2 run framework_bringup teleop`: w/a/s/d, space = stop, q = quit. The first key halts the running plan; teleop has the highest priority at
  `cmd_vel_mux`, so it wins while keys arrive and control returns to the plan source 0.5 s after the last key.
- `ros2 service call /bt_executor/halt std_srvs/srv/Trigger`, or `h` in `dash`. The mux zeroes `cmd_vel` 0.5 s after its inputs stop; a Kobuki
  bumper/cliff event latches a stop until `safety_reset` (refused while still pressed).
- Safety guards (no replacement for a hardware e-stop): lidar guard stops `DriveDistance`/`NavigateTo` (0.30 m) and `FollowTarget approach`
  (0.40 m) and refuses `RotateBy` with anything closer than 0.22 m; `battery_guard_node` halts the plan when the Kobuki battery is below 13.2 V
  for 3 s (thresholds are the driver defaults, unverified on this pack). Set `scan_required: true` in `param/extensions.yaml` on the real robot so
  a missing scan means "do not drive".

## Machines, security, time
- `ros2 run cluster_core host_info [host]`: CPU, RAM, GPU/VRAM, temperature and battery of every machine with a cluster agent.
- Workspaces (SROS2): `ws create lab`, `ws serve lab`, `ws join lab --token ...`, `ws invite lab --role observer|operator`, `ws members lab`,
  `ws revoke lab HOST` (re-keys; the rest must re-join). Roles: `full`, `operator` (reads all, publishes `bt/task` + `teleop/cmd_vel`),
  `observer` (reads all). Details in the README ("Workspaces and security").
- Clocks: `tools/net/timesync.sh server <subnet>` on one machine and `client <ip>` on the others (prints the chrony config, `--apply` installs it);
  `preflight` reads `chronyc tracking` and warns above 50 ms.
- Restart after a reboot: `tools/systemd/install.sh --enable` (user units for the LLM server and the stack; dry-run stays on).

## Record and replay
`tools/record_run.sh NAME [--images]` records the topics that matter (mcap, latched QoS kept) to `~/robocup_bags/`.
Replay: `ros2 bag play <dir> --clock`, consumers with `use_sim_time:=true`.

## Test and rehearse without a robot
`python3 -m pytest src` (unit), `bash tools/integration/rehearsal.sh` (9 task rehearsals in dry-run, checks the exact node sequence),
`python3 tools/integration/llm_eval.py` against a running pipeline + LLM (15 spoken-style commands, scores valid plans and the chosen macro),
`python3 tools/integration/prompt_lab.py` (replays a captured bridge request with sampler variants). Each integration script uses its own DDS
domain and cleans up; if you start stacks by hand, stop them with SIGTERM (background `&` jobs ignore SIGINT) and scope by `ROS_DOMAIN_ID`.
