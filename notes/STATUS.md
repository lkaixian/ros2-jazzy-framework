# Status

Snapshot: branch `main`, 28 ROS packages, ~780 Python tests + 13 C++ gtests passing, `check_params`
23/23 nodes consistent. Update this file when you change the picture.

Verification levels used below:
- **unit** = automated tests pass; **sim** = run end-to-end against a simulator or fake inputs on
  one machine; **hw** = ran on the real robot/devices. **Nothing has been run on hardware.**

## What exists

| Area | Packages | unit | sim | hw | Notes |
|---|---|---|---|---|---|
| Object detection (YOLO11 boxes) | `object_detection`, `yolo11_common`, `object_detection_msgr` | yes | real photo + real model: yes | no camera | boxes identical to Ultralytics; announces its classes; `train_detector` wrapper smoke-trained on GTX 1650 (peak 1.6 GB) |
| Person detect/track/pose | `person_detection`(+`_msgr`) | partly | no (needs weights/video) | no | `Person.distance_m` added by depth ranging node |
| Face detect + features + enroll | `face_recognition`(+`_msgr`) | gallery yes | stub only | no | needs user-supplied YOLO11 *face* weights; features are YOLO cls embeddings (weak, see debt) |
| LLM bridge (C++) | `llm_bridge` | yes | fake llama server | no real llama-server | prompt = all nodes + descriptions + detector labels via `bt_validator/catalog` |
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
| Terminal view | `framework_bringup dash` | yes | against sims | no | status table + `--logs` verbose |
| Safety mux | `bt_common/cmd_vel_mux_node` | yes | yes | no | priority + 0.5 s watchdog + clamps; Nav2 is the lowest-priority input; bumper/cliff latch + `safety_reset` service (fake events only) |
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
- SLM: a 3000-row fine-tune ran (held-out valid 0.99 / exact 0.985, measured against the rule-based generator, so it only shows the model learned the rules). Not evaluated on real LLM output; `models/bt-slm` is not wired into any launch default (pass `slm_model_path`). LLM -> spec -> XML quality with a real model: not evaluated.
- Real llama-server / Gemma 4 E4B (needs llama.cpp build >= b8637; model may be gated).
- Real STT (faster-whisper/Vosk), Piper, Kokoro, live microphone.
- Detector trained on the user's items (dataset does not exist yet); YOLO face weights; face accuracy.
- Arm: model unknown (ask the user's teammate), MoveIt config, gripper action, pick/place.
- Two physical machines / WiFi for the cluster; half-partition behaviour; SROS2.
- `setup.sh` apt/rosdep/pip/llama.cpp/models steps (dry-run only).
- Bumper/cliff latch exists in the mux but is untested with the real Kobuki driver (topic names, event timing); no scan-based obstacle check for extension motion; no hardware e-stop.

## Suggested next steps (in order)
1. Bring up the real robot step by step with the user present: Kobuki driver -> lidar -> TF check ->
   `navigation.launch.py slam:=slam_toolbox` -> save map -> measure the placeholder transforms ->
   `map:=...` navigation with `dry_run` still true, then live with a spotter.
2. Finish T-01: verify the bumper/cliff latch on the real Kobuki, add a scan-based check for extension motion, document a hardware stop.
3. Collect images of the user's items, train the detector (`train_detector`), then regenerate SLM
   data with `--labels`.
4. Decide the face-recognition approach (technical_debt T-12), install STT/TTS engines, test audio.
5. Fine-tune the SLM, compare against the rule-based generator on held-out specs, then add the
   runtime failure -> SLM repair loop (the "fast brain" idea, T-06).
6. Pick the arm model; fill `arm_*` params; test MoveIt in its simulator.
7. Two-machine cluster test; set real job costs in `param/cluster_jobs.yaml`.
8. Push and watch the first run of `.github/workflows/ci.yml`, fix what GitHub finds (T-02; never run there yet).
