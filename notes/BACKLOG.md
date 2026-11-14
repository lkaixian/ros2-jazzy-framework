# Backlog (logged 2026-11-14, to start the next session)

Picked up from the end of a long session. Order inside each group is the suggested order. IDs B-xx
are new; T-xx point at `technical_debt.md`. Update this file as items are done (delete or tick).

## Start here
- [x] **CI check (latest run green, 2026-11-14).** The last full GitHub run failed only in `Sim - workspaces (create, discover, join) +
  SROS2 enforcement` (the secured DDS probes printed neither RECEIVED nor NOTHING in the container; it
  passes locally). Since then: `ros-jazzy-rmw-cyclonedds-cpp` added to the apt list and the probe now
  prints its stderr. Look at the newest run (`gh run list`), read the probe stderr, fix. Everything
  else in CI passed in the earlier run, including the D* Lite Nav2 sim.
- [ ] **Macro list (the user is thinking about it).** `src/framework_bringup/param/macros.yaml` has 9
  macros. Add yours: an entry + `example_args`; `test_macro_library.py` validates every macro against
  the real catalog. Remember a macro cannot use decorators (grammar = sequence/fallback groups + retry).
- [x] **Define the competition tasks** (GPSR, HRIC, PnPC, Laundry: see `notes/TASKS.md`; next: turn them into acceptance tests). Write the target tasks down (even 5 lines each); they decide the
  priorities below and become acceptance tests. Nothing in the repo defines them yet.

- [ ] **Macros: execute `notes/MACROS_PLAN.md`** (work plan P0-P6; start with P1 macro batch A and P0 node outputs = B-01).

## Can be built and tested without the robot
- [x] **B-01 Node outputs (T-09), DONE 2026-11-14:** extension handlers return `(ok, msg, {"out_x": v})`, manifests list `out_x` as an optional port, `bt_executor` writes it to the blackboard; round trip tested by `tools/integration/outputs_test.py` (also in CI). Nodes still to use it: see `MACROS_PLAN.md` P2. Original: Extension nodes cannot return values (no output ports, all ports are
  strings), e.g. `GetPersonPose` cannot hand keypoints on. Blocks genuinely complex tasks. Suggest first.
- [x] **B-02 Preflight check.** `ros2 run framework_bringup preflight`: lidar publishing, TF chain sane,
  mux alive, `LLM_API_KEY` set, workspace/SROS2 state, LLM endpoints healthy, executor in dry-run.
- [~] **B-03 Richer task grammar (T-07) (`repeat` loops DONE 2026-11-14; Parallel/SubTree/blackboard loops open; SLM not retrained with it).** Parallel, loops, SubTree/blackboard in the spec grammar AND
  `bt_slm/fallback.py` AND the dataset generator together (the SLM prompt format must stay in sync).
- [~] **B-04 Obstacle awareness for plain motion (PARTLY DONE 2026-11-14):** `bt_common.obstacle.ScanGuard`; `DriveDistance`/`NavigateTo` (odom_extension) stop for anything closer than 0.30 m in the travel direction (front, or rear when reversing) and `FollowTarget approach=true` (pid_extension) stops at 0.40 m; `scan_required: true` makes them refuse to drive without a fresh scan (default false so lidar-less sims work; set it true on the real robot). Tested with fake scans (`tools/integration/guard_test.py`, unit tests); `RotateBy` is unguarded (turns in place); the lidar yaw offset and mounting are unverified; a hardware e-stop is still required. Original: `DriveDistance`/`RotateBy`/`FollowTarget`
  drive blind; route them through `collision_monitor` or a scan check. Document a hardware e-stop.
- [x] **B-05 Recording/replay (DONE 2026-11-14):** `tools/record_run.sh NAME [--images]` (mcap, latched QoS kept; recorded and read back with `ros2 bag info`); replay: `ros2 bag play <dir> --clock`. Original: rosbag2 record profile for a run (topics list, QoS), plus a replay recipe.
- [~] **B-06 Time sync across machines** (preflight now WARNs when the clock is not NTP-synchronised; the chrony setup recipe for two PCs is still open). (chrony/NTP recipe + a check in preflight). Needed before two PCs.
- [x] **B-07 Manual override (DONE 2026-11-14):** `ros2 run framework_bringup teleop` (w/a/s/d, first key also halts the executor) publishes to `teleop/cmd_vel`, now the FIRST (highest priority) input of `cmd_vel_mux`; releasing the keys returns control after the 0.5 s watchdog. Tested: key map + config; not tried on the real robot. Original: Teleop path into `cmd_vel_mux` at the highest priority (and a way to
  take control back during a run).
- [~] **B-08 Battery/health monitoring** (Kobuki battery now shown in `dash` with low 14.0 V / critical 13.2 V thresholds taken from the driver defaults, unverified on this pack; NO automatic halt on low battery, that is a safety decision to make with the robot). (laptop/PC side DONE: `host_info` service + CLI; still open: Kobuki battery topic, show in `dash`, warn/halt threshold). Kobuki publishes battery state; nothing consumes it. Show it in
  `dash`, warn/halt under a threshold.
- [~] **B-09 Process supervision on the robot (units written 2026-11-14, NOT enabled or run):** `tools/systemd/` user units `robocup-llm` and `robocup-stack` with `Restart=always`, dry-run kept on; `systemd-analyze --user verify` is clean; install with `tools/systemd/install.sh [--enable]`. Cluster agent as a service still open. Original: systemd units (or the cluster agent as a service with
  `Restart=always`) so a reboot brings the stack back. Pairs with T-15.
- [ ] **B-10 Hygiene** (LICENSE added; rest open). `LICENSE` (package.xml says Apache-2.0, T-20), lint beyond `object_detection`
  (T-19), `setup.sh` version pins + a tested venv-or-pin story for NumPy 2 (T-17).
- [~] **B-11 D* Lite speedup: NOT adopted (2026-11-14).** Finding: colcon builds the plugin with no build type, i.e. no optimisation (`CMAKE_BUILD_TYPE=` empty). Core micro-benchmark (`test_core` `FreshSearchTiming`, 240x160): 50 ms -> 8 ms with Release, and all 10 core tests (Dijkstra oracle) still pass, also under ASAN/UBSAN at -O2. BUT with the optimised plugin `planner_server` segfaults (exit -11) on the first goal in nav_sim; Debug/default builds are fine. Backtrace (RelWithDebInfo overlay, core dump): `Core::reset` -> `Core::insert` at core.cpp:51 inside `DStarLite::createPlan` planner.cpp:95, with a sane `Core` (w=240, h=160, goal 209,129) and a garbage `s`; root cause NOT found (suspect UB or a threading/ordering problem in the plugin glue, visible only when optimised). Reverted; the shipped build is unchanged and passes `nav_planner_test.py dstar_lite` (46.3 s, 0.23 m). Do not set a Release default for this package until that is understood. Original: Reuse the cost-grid buffer and convert only changed
  regions; today it is ~6 ms for a new goal vs ~0.5 ms for NavFn on the sample map.
- [x] **B-12 Workspaces: per-topic permissions and revocation (DONE 2026-11-14):** roles `full|operator|observer` with SROS2 permissions per enclave, per-role tokens/invites, `ws revoke` = remove + rekey (new CA) + rotate tokens and LLM key. Tested end to end on one machine in `tools/integration/workspace_test.py` (observer reads but cannot publish; revoked keys reach nothing; old token refused; a remaining member re-joins). Still one machine only; roles are fixed in code. Original: Members can use every topic today;
  removing a member means creating a new workspace.

## Needs the robot or real assets
- [-] DEFERRED (user, 2026-11-14: cannot be done for now) **B-20 Real hardware bring-up** (the order in STATUS "Suggested next steps" 1): Kobuki driver ->
  RPLIDAR (`sllidar_ros2`) -> TF check -> slam_toolbox map -> measure lidar/camera mounts and robot
  radius (`param/navigation.yaml`, `nav2_params.yaml`) -> real `places.yaml` -> navigate dry-run, then
  live with a spotter. Re-map: `~/maps/my_map.yaml` is unusable. Verify the bumper/cliff latch with the
  real driver (topic names/timing are assumed).
- [~] **B-21 Real LLM (PARTLY DONE 2026-11-14):** llama.cpp b12375 CUDA prebuilt + Gemma 4 E2B Q4_0 run on the GTX 1650 (about 1-2 s per plan, 15/15 valid and sensible plans on `llm_eval.py`; details in `RUNBOOK.md`). Still open: E4B on the big PC, `LLAMA_API_KEY`/E4B/E2B cluster flags, SLM vs LLM comparison, harder commands. Original: Build llama.cpp (>= b8637), run `llama-server`; Gemma E4B does not fit the
  4 GB GPU well (4.28 GiB Q4_0), E2B Q4_0 is 2.65 GiB. The E4B/E2B flags in `cluster_jobs.yaml` and
  `LLAMA_API_KEY` support are UNVERIFIED. Then evaluate LLM -> spec -> XML quality and compare the
  fine-tuned SLM with the rule generator on real LLM specs (the held-out 0.99/0.985 is synthetic).
- [ ] **B-22 Perception assets.** Collect images of the competition items and train the detector;
  provide YOLO face weights and evaluate face accuracy (T-12); pose models for gestures (T-13).
- [~] **B-23 Audio (PARTLY DONE 2026-11-14):** Piper voice + faster-whisper base.en downloaded (`setup.sh --only voice`); Piper->Whisper round trip exact; a spoken WAV went through capture -> VAD -> STT -> `bt/task` -> E2B -> valid plan. Still open: real microphone, VAD in noise, wake word, Kokoro, speaker playback, T-14. Original: Download model files (Whisper/Vosk model, Piper voice, Kokoro files), test the
  real microphone, VAD in noise, wake word (T-14).
- [-] DEFERRED (user, 2026-11-14: cannot be done for now) **B-24 Two machines.** Cluster failover over WiFi/Ethernet (T-15), LLM failover to the E2B,
  workspace join across the LAN (unicast discovery between the peers is unverified), dash link
  check against a real wired/wireless peer, SROS2 across hosts.
- [-] DEFERRED (user, 2026-11-14: cannot be done for now) **B-25 Arm** (out of scope for now): model, MoveIt config, gripper action (T-11).

## Done in the last session (so it is not redone)
Bumper/cliff latch in `cmd_vel_mux`; GitHub Actions CI; LLM bridge failover + emergency halt + hosts
joining/leaving via cluster heartbeats; TCP connection reuse + API key; sync/async node tags; executor
`failed_node` + task lifecycle FSM + optional LLM repair (`max_repairs`); `llm_knowledge.md` with drift
guards; scene summary with gestures; `dash`; D* Lite plugin (`planner:=dstar_lite`); `ws` workspaces
with SROS2; macros engine + 9 macros; SLM `rules_first`; README updated. First SLM fine-tune ran
(`models/bt-slm`, not wired into any launch default). Details: `STATUS.md`, `technical_debt.md`, git log.
