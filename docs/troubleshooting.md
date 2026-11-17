# Troubleshooting

Find the symptom, read the cause, apply the fix. Everything here cost real time once.

> [!TIP]
> Most start-up problems are one of three things: system Python not first on `PATH`, a stale `install/` after a Python edit (rebuild), or a shared DDS domain (use your own `ROS_DOMAIN_ID`).

## Build and environment

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: yaml`, `No module named 'em'`, no pytest | `python3` on PATH is `~/bin/python3` and `~/pyvenv.cfg` makes it hide system packages | `source env.sh` (system Python first) or `export PATH=/usr/bin:$PATH`; delete stale `build/<pkg>` and `install/<pkg>` after a wrong-Python configure |
| `cannot import name ...` after editing Python | `ament_python` copies `.py` files into `install/` even with `--symlink-install` | `colcon build --packages-select <pkg>` and re-source |
| Training fails importing matplotlib | pip's NumPy 2 in `~/.local` shadows apt NumPy 1.26 | `./setup.sh --only python` (installs a compatible matplotlib) or pin NumPy < 2 |
| `planner_server` exit -11 on the first goal | static core library linked into a shared plugin without `-fPIC` | `POSITION_INDEPENDENT_CODE ON` (done for `dstar_lite_core`) |
| Everyone's workspace broke after an experiment | an experimental binary was installed into the shared `install/` | experiments only with `colcon build --build-base B --install-base I --packages-select X`, then `source I/setup.bash` on top |
| CMake target not found | Jazzy package target is `behaviortree_cpp::behaviortree_cpp`, not `BT::...` | use the namespaced target |

## ROS and DDS

| Symptom | Cause | Fix |
|---|---|---|
| `error creating node` | the DDS domain ran out of participants (leaked or duplicated stacks) | stop everything, check `ps`, use a fresh `ROS_DOMAIN_ID`; never start `extensions.launch.py` twice |
| Nodes die at start (`RCLError: error creating node`, exit 1 or -6; the lidar and extension nodes start last) with the full stack plus Nav2 | Cyclone's default allows ~32 DDS participants per machine and every ROS process is one; Nav2 pushes the stack past it, so no scans, no `map` frame, costmap "Timed out waiting for transform" | `export CYCLONEDDS_URI=file://$PWD/tools/net/cyclonedds_local.xml` (limit 200) in **every** terminal (`dash`, `teleop` and `ros2` CLI too) and restart everything. The name is all caps: `CycloneDDS_URI` is silently ignored |
| Tests see someone else's robot | shared domain | own `ROS_DOMAIN_ID` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Extension nodes unknown to the validator | its manifest was missed (latched publication loses races at startup) | manifests are republished every 3 s; start extensions **before** sending tasks; check `bt_validator/catalog` |
| A parameter in my YAML does nothing | unknown keys are ignored silently | `ros2 run framework_bringup check_params` |
| Empty list parameter crashes or is typed wrong | an empty YAML array has no type | use comma-separated strings (`roles`, `tags`) or `[""]` for "unset" |
| `ros2 launch` children survive | a killed parent leaves children | check `ps` and the process's `ROS_DOMAIN_ID` in `/proc/<pid>/environ` before killing, so you never touch another session |
| Service name rejected | a token starting with a digit (a container id as host id) | sanitise with `host_service_name` |
| Restricted SROS2 role: node will not start | rclpy needs parameter-service replies | keep them in the role's permissions |
| `create_publisher` raises under SROS2 | a denied DDS writer raises at creation, not later | check the role (`observer` cannot publish) |

## Planning and the LLM

| Symptom | Cause | Fix |
|---|---|---|
| Reply "not a JSON object", truncated | Gemma 4 spent `max_tokens` thinking | start `llama-server` with `--reasoning off` |
| Plans get truncated or odd; prompt cut | four slots sharing a 4096 context | `-c 6144 -np 1` |
| Model picks macros without args | `nlohmann::json` sorts keys alphabetically, so `args` came before `macro` in the schema | send the schema as `ordered_json` (guarded by a gtest) |
| `min_conf: ">0.5"` | the model invented an operator | validator now rejects non-numbers; the error is fed back |
| Plan invalid: "unknown node" | the node is not in any running extension's manifest | start that extension; or fix the node name |
| `plan uses unknown nodes [...]; refusing to drop them` | the retry path found a node it does not know | same as above; this used to silently shorten the plan |
| `<Parallel> starts two nodes of extension 'X'` | one extension accepts one Action at a time | put them in a `Sequence`, or use nodes from two extensions |
| Gemma 4 will not load | llama.cpp too old | build >= b8637; the repo may be gated (accept the licence, `hf auth login`) |
| Whole stack "halted" after start | every LLM endpoint down: emergency halt | start `llama-server`; `startup_grace_sec` (60 s) covers slow model loads |
| Eval scores look shifted or low | stale plans in flight, a stack missing an extension, a leaked stack | see [Evaluation](evaluation.md#pitfalls-that-produced-wrong-numbers) |

## Execution

| Symptom | Cause | Fix |
|---|---|---|
| Plan valid but nothing happens | `auto_run: false` by default | `ros2 service call /bt_executor/run std_srvs/srv/Trigger` |
| Run refused: "mode gate: no fresh robot/mode..." | `mode_gate: true` and no mode node, a stale mode, or `run_ok` is false | start `robot_mode_node`; read `ros2 topic echo robot/mode` for `reasons` and `missing`; `robot/reset` after clearing the cause |
| Robot does not move though the plan ran | `dry_run: true` (default) | `dry_run:=false` **only** with a person at the e-stop |
| Robot stopped by itself | mux latch (bumper/cliff), lidar guard (0.30 m), battery guard (13.2 V), watchdog (0.5 s), or a teleop key | `ros2 topic echo safety/stop_state`; `safety_reset` once the sensor is clear |
| `RotateBy` refuses | something within 0.22 m | move the obstacle; set `scan_required: true` on the real robot |
| A BT node fails with `bad port: ...` | the extension range-checked a port | fix the value; the message names the port |
| `Speak` hangs | `tts_node` not running | `audio.launch.py enable_stt:=false` |
| Port named `name`/`ID` rejected | reserved by BehaviorTree.CPP | rename the port |
| Polling a condition spins | `RetryUntilSuccessful` retries immediately | `Fallback(Sequence(...), ForceFailure(Sleep msec=200))` |
| A condition returning RUNNING errors | only action nodes may return RUNNING | extension nodes are registered as actions already |

## Voice

| Symptom | Fix |
|---|---|
| STT node refuses to start | install an engine: `pip install faster-whisper` or `vosk` (`mock` only tests plumbing) |
| Robot answers itself | STT mutes while `tts/status` is SPEAKING; check the TTS node is the one publishing status |
| Heard place names are wrong | `hint_places: true` in `stt_pipeline.yaml` (Whisper prompt from `places.yaml`); `snap_places` in `transcript_to_task` snaps close matches (Vosk ignores hints) |
| `espeak` writes empty files | use `-w file` without `-q` |

## Crashes and logs

* Core dumps go to `/var/crash/*.crash` (apport): `apport-unpack x.crash dir; gdb -batch -ex bt <exe> dir/CoreDump`.
* `ros2 run framework_bringup dash --logs --level warn --node llm_bridge` streams `/rosout` filtered.
* `ros2 run framework_bringup preflight` checks the stack before a run (key, SROS2, mux, executor dry-run, LLM, `/scan`, TF, clock, disk).
* Disk is tight on the dev laptop (about 6 GB free): models live in `models/` (gitignored); prune old models and datasets before training.
* `tools/record_run.sh` records a run with rosbag2.

## Process tips learned the hard way

* An edit that matches nothing changes nothing and reports success. Re-grep after scripted edits (`tools/strict_sub.py` raises on a wrong match count).
* `git add -A` once swept agent worktrees into a commit; check `git status` first.
* Background jobs from a non-interactive shell ignore SIGINT: use SIGTERM.
