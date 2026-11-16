# Status

Snapshot (2026-11-14, evening): branch `main`, 28 ROS packages, ~940 Python tests + 14 gtests passing, `check_params`
25/25 nodes consistent. The two `object_detection` lint tests fail (they lint `build/`; T-19). Update this file when you change the picture.

Verification levels used below:
- **unit** = automated tests pass; **sim** = run end-to-end against a simulator or fake inputs on
  one machine; **hw** = ran on the real robot/devices. **Nothing has been run on hardware.**

## What exists

| Area | Packages | unit | sim | hw | Notes |
|---|---|---|---|---|---|
| Object detection (YOLO11 boxes) | `object_detection`, `yolo11_common`, `object_detection_msgr` | yes | real photo + real model: yes | no camera | boxes identical to Ultralytics; announces its classes; `train_detector` wrapper smoke-trained on GTX 1650 (peak 1.6 GB) |
| Person detect/track/pose | `person_detection`(+`_msgr`) | partly | no (needs weights/video) | no | `Person.distance_m` added by depth ranging node |
| Face detect + features + enroll | `face_recognition`(+`_msgr`) | gallery yes | stub only | no | needs user-supplied YOLO11 *face* weights; features are YOLO cls embeddings (weak, see debt) |
| LLM bridge (C++) | `llm_bridge` | yes | **real llama-server + Gemma 4 E2B (CUDA, GTX 1650)**: 15/15 valid, sensible plans, 1-2 s each (`llm_eval.py`); fake servers for failover | no robot | prompt = macros + nodes + descriptions + labels via `bt_validator/catalog`; replies constrained by a JSON schema (macro whitelist, real ports); `--reasoning off` required; see RUNBOOK |
| SLM | `bt_slm` | yes | rule-based fallback only | n/a | **never fine-tuned**; dataset generator + `finetune_bt_slm` exist, finetune never run |
| Validator | `bt_validator` | yes | yes | n/a | label vocabulary gate, `validate_bt` CLI |
| Orchestrator | `bt_orchestrator` | light | yes | n/a | generate -> validate -> retry loop; `submit_bt` for hand-written trees |
| BT executor (C++) | `bt_executor` | yes | yes | no | dry-run default, `run`/`halt` services, built-in perception conditions |
| Extensions | `odom_`, `pid_`, `pose_`, `social_`, `arm_`, `nav_extension` | yes | odom/pid/social/nav: yes | no | arm: only goal construction tested (arm model undecided) |
| LLM failover + hosts joining/leaving | `llm_bridge`, `cluster_core`, `cluster_msgs` | yes | fake servers; real agent+scheduler+bridge on one machine | no | endpoints advertised in heartbeats; breaker; probation until /health; emergency halt when all down. llama.cpp/Gemma never run |
| Task lifecycle + repair | `bt_orchestrator` (`task_fsm`), `bt_executor` (`failed_node`) | yes | fake LLM, dry-run executor | no | `max_repairs` default 0; node `mode` sync/async in the LLM listing only (SLM format unchanged) |
| Planner knowledge + scene | `llm_bridge`, `framework_bringup/param/llm_knowledge.md`, `bt_orchestrator` (`scene.py`) | yes | prompt captured over real HTTP | no | node list with sync/async tags + robot facts/places (drift-guarded by test) + per-person distance/waving/pointing + known faces. Face and pose models are NOT present here: the skeletons in the sim are synthetic |
| Workspaces + SROS2 | `framework_bringup ws`, `dstar_lite_planner` is separate | yes | create/discover/join + secured pub/sub + 2 outsider kinds on one machine | no | token join over TLS with channel binding; no per-topic ACL; no revocation |
| D* Lite planner | `dstar_lite_planner` | 9 gtests vs Dijkstra | Nav2 door route in nav_sim: same 45.5 s as NavFn | no | `planner:=dstar_lite`; incremental repair verified at core level only |
| Macros | `bt_common/macros.py`, `param/macros.yaml`, orchestrator, validator | every macro validated against the real catalog; whitelist + schema tests | real E2B chooses macros (e.g. `go_and_say`, `patrol`, `meet_guest`) | no | 27 whitelisted macros (`notes/MACROS_PLAN.md`); SLM skipped by `rules_first` for in-grammar specs |
| WICL task memory | `bt_memory`, `bt_rehearsal`, `bt_orchestrator` (`memory_hook`) | yes | kinematic 2D sim + real Gemma E2B over HTTP; mocked LLM in CI | no | off by default (`memory_enabled`); benchmark on the production prompt: memory 100% vs 75% on alias tasks (8 tasks, best case); no gain elsewhere (already 100%); no Nav2 backend; see docs/wicl.md |
| Terminal view | `framework_bringup dash` | yes | against sims | no | status table + `--logs` verbose; keys for node/system restart, stop, halt; Kobuki battery line |
| Safety mux | `bt_common/cmd_vel_mux_node` | yes | yes | no | priority + 0.5 s watchdog + clamps; Nav2 is the lowest-priority input; bumper/cliff latch + `safety_reset` service (fake events only) |
| Dashboard + executor feed | `framework_bringup dash`, `bt_executor` (`bt/tree`, `bt/node_status`) | yes | live tree against a real executor; 3-check feed test in CI | no | keys for restart/stop/halt, plan tree, Kobuki battery |
| Task-spec grammar | `bt_common/macros.py`, `bt_slm/fallback.py`, `bt_validator`, `llm_bridge` | yes | `rehearsal.sh`: 9 task rehearsals in dry-run | no | macros, `retry`, `repeat`, `Parallel`, `foreach`; docs/task-spec-grammar.md |
| Guards | `bt_common` (`ScanGuard`, `battery_guard_node`), `odom_extension`, `pid_extension` | yes | fake scans / fake battery, CI | no | lidar stop distances, turn clearance, battery halt: thresholds unverified on the robot |
| Speech nodes | `social_extension` (`WaitForTranscript`, `ExtractAnswer`, `DetectSound`), `stt_pipeline` | yes | synthetic WAVs through the real audio path | no | real doorbell/microphone untested |
| Workspace roles | `framework_bringup ws` | yes | observer reads/cannot publish, revoke re-keys (one machine) | no | no second machine |
| Operations | `tools/record_run.sh`, `tools/systemd/`, `tools/net/timesync.sh`, `preflight`, `teleop` | parsers/keys | rosbag record+info; units verified with systemd-analyze, not enabled | no | chrony not installed here |
| Audio | `audio_msgr`, `audio_capture`, `stt_pipeline`, `tts_pipeline` | yes | mock engines | no | no real STT engine installed; espeak works; Piper/Kokoro untested |
| Navigation | `nav_sim`, `nav_extension`, launch+params in `framework_bringup` | yes | real Nav2+AMCL+slam_toolbox+Cartographer on a simulated lidar | no | RTAB-Map/RealSense only launch-checked |
| Cluster | `cluster_msgs`, `cluster_core` | 621 tests | scheduler + 3 simulated hosts, one machine | no 2nd machine | failover ~5.2 s, restart blackout 1.6 s |
| Bring-up | `framework_bringup` + root `launch/` | check_params | yes | no | `framework.launch.py` has switches `enable_audio`, `enable_sensors`, `enable_navigation` (all off by default) |
| Tooling | `setup.sh`, `env.sh`, `tools/integration/*` | n/a | yes | n/a | `setup.sh`: only dry-run and `--only build` were executed |

## Verified end to end (sim / fake inputs)
- task text -> fake llama -> SLM fallback -> validator -> executor -> odom_extension -> mux -> simulated
  robot: 0.29 m / 0.47 rad for a 0.30 / 0.50 request; dry-run does not move; halt stops; SIGKILL of the
  extension is stopped by the mux watchdog within 0.5 s.
- Greeting BT: fake person+face -> `ExtractFaceFeature` -> `Speak "Hello alice"` (mock TTS);
  unknown face -> "Hello there"; enrol tree -> stub service called.
- Navigation: `NavigateToPlace door` through BT -> Nav2 -> mux -> `nav_sim`; AMCL pose within 0.12 m of
  the goal; unknown place fails cleanly.
- Detector: bus.jpg through `object_detection_node`, boxes match Ultralytics; custom 2-class model
  announced -> validator rejects `mug`, accepts `pedestrian`; LLM prompt contains labels.

## NOT done / NOT verified
- **Everything on real hardware**: Kobuki driver, RPLIDAR via `sllidar_ros2`, RealSense stream,
  RTAB-Map mapping, depth obstacles in costmaps, Nav2 tuning, odom/PID motion on the floor.
- **Placeholders that must be measured** (`param/navigation.yaml`): lidar mount, camera mount; robot
  radius (0.18) in `param/nav2_params.yaml`; `param/places.yaml` coordinates (sample map only).
- `~/maps/my_map.yaml` is unusable for planning (2.6 m^2 connected free space): re-map with slam_toolbox.
- SLM: re-trained on the 45-node catalog with the new grammar: held-out valid 0.995 / exact 0.935 vs the rule generator on synthetic specs; wired by auto-discovery of `models/bt-slm` (`ROBOCUP_SLM_MODEL` overrides, `model_path: none` = rules only); never evaluated on real LLM output.
- Gemma 4 E4B on the big PC and `LLAMA_API_KEY` support (E2B + llama.cpp b12375 DO run here: see RUNBOOK); no comparison against E4B.
- Live microphone and speaker, VAD in noise, wake word, Kokoro; (faster-whisper + Piper work on synthetic audio).
- Detector trained on the user's items (dataset does not exist yet); YOLO face weights; face accuracy.
- Arm: model unknown (ask the user's teammate), MoveIt config, gripper action, pick/place.
- Two physical machines / WiFi for the cluster; half-partition behaviour; SROS2.
- `setup.sh`: the `prebuilt` llama.cpp and `voice` steps were reproduced by hand and dry-run; apt/rosdep and a clean-machine run were not executed.
- Bumper/cliff latch, lidar guards and the battery guard are tested with fake events only (real Kobuki topics, lidar mounting yaw and battery thresholds unverified); no hardware e-stop.

## Suggested next steps (in order; the up-to-date short list is in BACKLOG.md "NEXT")
1. Bring up the real robot step by step with the user present: Kobuki driver -> lidar -> TF check ->
   `navigation.launch.py slam:=slam_toolbox` -> save map -> measure the placeholder transforms ->
   `map:=...` navigation with `dry_run` still true, then live with a spotter.
2. Finish T-01: verify the bumper/cliff latch on the real Kobuki, add a scan-based check for extension motion, document a hardware stop.
3. Collect images of the user's items, train the detector (`train_detector`), then regenerate SLM
   data with `--labels`.
4. Decide the face-recognition approach (technical_debt T-12), install STT/TTS engines, test audio.
5. Compare the SLM with the rule generator on real LLM specs; consider a runtime failure -> SLM repair loop (T-06).
   runtime failure -> SLM repair loop (the "fast brain" idea, T-06).
6. Pick the arm model; fill `arm_*` params; test MoveIt in its simulator.
7. Two-machine cluster test; set real job costs in `param/cluster_jobs.yaml`.
8. Push and watch the first run of `.github/workflows/ci.yml`, fix what GitHub finds (T-02; never run there yet).
