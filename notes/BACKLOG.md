# Backlog (logged 2026-11-14, to start the next session)

Picked up from the end of a long session. Order inside each group is the suggested order. IDs B-xx
are new; T-xx point at `technical_debt.md`. Update this file as items are done (delete or tick).

## Start here
- [ ] **CI check.** The last full GitHub run failed only in `Sim - workspaces (create, discover, join) +
  SROS2 enforcement` (the secured DDS probes printed neither RECEIVED nor NOTHING in the container; it
  passes locally). Since then: `ros-jazzy-rmw-cyclonedds-cpp` added to the apt list and the probe now
  prints its stderr. Look at the newest run (`gh run list`), read the probe stderr, fix. Everything
  else in CI passed in the earlier run, including the D* Lite Nav2 sim.
- [ ] **Macro list (the user is thinking about it).** `src/framework_bringup/param/macros.yaml` has 9
  macros. Add yours: an entry + `example_args`; `test_macro_library.py` validates every macro against
  the real catalog. Remember a macro cannot use decorators (grammar = sequence/fallback groups + retry).
- [ ] **Define the competition tasks.** Write the target tasks down (even 5 lines each); they decide the
  priorities below and become acceptance tests. Nothing in the repo defines them yet.

## Can be built and tested without the robot
- [ ] **B-01 Node outputs (T-09).** Extension nodes cannot return values (no output ports, all ports are
  strings), e.g. `GetPersonPose` cannot hand keypoints on. Blocks genuinely complex tasks. Suggest first.
- [ ] **B-02 Preflight check.** `ros2 run framework_bringup preflight`: lidar publishing, TF chain sane,
  mux alive, `LLM_API_KEY` set, workspace/SROS2 state, LLM endpoints healthy, executor in dry-run.
- [ ] **B-03 Richer task grammar (T-07).** Parallel, loops, SubTree/blackboard in the spec grammar AND
  `bt_slm/fallback.py` AND the dataset generator together (the SLM prompt format must stay in sync).
- [ ] **B-04 Obstacle awareness for plain motion (T-01 rest).** `DriveDistance`/`RotateBy`/`FollowTarget`
  drive blind; route them through `collision_monitor` or a scan check. Document a hardware e-stop.
- [ ] **B-05 Recording/replay.** rosbag2 record profile for a run (topics list, QoS), plus a replay recipe.
- [ ] **B-06 Time sync across machines** (chrony/NTP recipe + a check in preflight). Needed before two PCs.
- [ ] **B-07 Manual override.** Teleop path into `cmd_vel_mux` at the highest priority (and a way to
  take control back during a run).
- [ ] **B-08 Battery/health monitoring.** Kobuki publishes battery state; nothing consumes it. Show it in
  `dash`, warn/halt under a threshold.
- [ ] **B-09 Process supervision on the robot.** systemd units (or the cluster agent as a service with
  `Restart=always`) so a reboot brings the stack back. Pairs with T-15.
- [ ] **B-10 Hygiene.** `LICENSE` (package.xml says Apache-2.0, T-20), lint beyond `object_detection`
  (T-19), `setup.sh` version pins + a tested venv-or-pin story for NumPy 2 (T-17).
- [ ] **B-11 Cheap D* Lite speedup (optional).** Reuse the cost-grid buffer and convert only changed
  regions; today it is ~6 ms for a new goal vs ~0.5 ms for NavFn on the sample map.
- [ ] **B-12 Workspaces: per-topic permissions and revocation.** Members can use every topic today;
  removing a member means creating a new workspace.

## Needs the robot or real assets
- [ ] **B-20 Real hardware bring-up** (the order in STATUS "Suggested next steps" 1): Kobuki driver ->
  RPLIDAR (`sllidar_ros2`) -> TF check -> slam_toolbox map -> measure lidar/camera mounts and robot
  radius (`param/navigation.yaml`, `nav2_params.yaml`) -> real `places.yaml` -> navigate dry-run, then
  live with a spotter. Re-map: `~/maps/my_map.yaml` is unusable. Verify the bumper/cliff latch with the
  real driver (topic names/timing are assumed).
- [ ] **B-21 Real LLM.** Build llama.cpp (>= b8637), run `llama-server`; Gemma E4B does not fit the
  4 GB GPU well (4.28 GiB Q4_0), E2B Q4_0 is 2.65 GiB. The E4B/E2B flags in `cluster_jobs.yaml` and
  `LLAMA_API_KEY` support are UNVERIFIED. Then evaluate LLM -> spec -> XML quality and compare the
  fine-tuned SLM with the rule generator on real LLM specs (the held-out 0.99/0.985 is synthetic).
- [ ] **B-22 Perception assets.** Collect images of the competition items and train the detector;
  provide YOLO face weights and evaluate face accuracy (T-12); pose models for gestures (T-13).
- [ ] **B-23 Audio.** Download model files (Whisper/Vosk model, Piper voice, Kokoro files), test the
  real microphone, VAD in noise, wake word (T-14).
- [ ] **B-24 Two machines.** Cluster failover over WiFi/Ethernet (T-15), LLM failover to the E2B,
  workspace join across the LAN (unicast discovery between the peers is unverified), dash link
  check against a real wired/wireless peer, SROS2 across hosts.
- [ ] **B-25 Arm** (out of scope for now): model, MoveIt config, gripper action (T-11).

## Done in the last session (so it is not redone)
Bumper/cliff latch in `cmd_vel_mux`; GitHub Actions CI; LLM bridge failover + emergency halt + hosts
joining/leaving via cluster heartbeats; TCP connection reuse + API key; sync/async node tags; executor
`failed_node` + task lifecycle FSM + optional LLM repair (`max_repairs`); `llm_knowledge.md` with drift
guards; scene summary with gestures; `dash`; D* Lite plugin (`planner:=dstar_lite`); `ws` workspaces
with SROS2; macros engine + 9 macros; SLM `rules_first`; README updated. First SLM fine-tune ran
(`models/bt-slm`, not wired into any launch default). Details: `STATUS.md`, `technical_debt.md`, git log.
