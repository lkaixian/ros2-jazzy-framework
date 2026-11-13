# Technical debt

Severity: **S1** can hurt people/hardware or invalidate results; **S2** likely to bite soon;
**S3** friction/cleanup. IDs are stable so notes and commits can cite them.

## S1 - safety and correctness
| ID | Debt | Why it matters | Suggested fix |
|---|---|---|---|
| T-01 | **No bumper/cliff/hardware-stop integration.** Kobuki bumper and cliff events are not read by anything; `odom_extension`/`pid_extension` motion (DriveDistance, RotateBy, FollowTarget) has no obstacle awareness at all. The mux watchdog is software only. | Live motion can drive into things; a hung PC can still coast ~0.5 s. | Subscribe to `/events/bumper|cliff`, make the mux output zero on events; route extension motion through `collision_monitor`/a scan check; document a hardware e-stop. |
| T-02 | **No CI.** Build, tests, `check_params`, launch smoke and the integration scripts run only by hand. Several silent breakages (no-op edits) were found late. | Regressions go unnoticed. | GitHub Actions on Ubuntu 24.04 + ROS Jazzy container: build, pytest, gtest, `check_params`, `launch --show-args` for every launch file; a test asserting each launch switch exists. |
| T-03 | **Unauthenticated DDS.** Any host on the network can publish `bt/task`/`bt/plan`/`cluster/assignments` or call `bt_executor/run`. Cluster agents only run local-catalog jobs, but with `dry_run:=false` a rogue `bt/plan` moves the robot. | Network = control surface. | SROS2 (enclaves + access control), or at least restrict domain/interface; require an arming service before live runs. |
| T-04 | **Nothing has run on hardware.** All motion, nav, audio, depth, driver paths are sim/mock only; simulator lidar is near-ideal. | Parameters (speeds, gains, costmap, thresholds) are untuned guesses. | Bring-up checklist in STATUS.md, tune with the user present. |
| T-05 | **Placeholder geometry**: lidar/camera mounts (`param/navigation.yaml`), robot radius 0.18, places coordinates. | Wrong TF = wrong maps/obstacles. | Measure; consider a real URDF/`robot_state_publisher` for the Kobuki. |
| T-12 | **Face "features" are YOLO classifier embeddings** (`yolo11n-cls`), not identity embeddings; threshold 0.6 is arbitrary and unvalidated. Face detector weights are not provided. | Recognition may confuse people; "Hello alice" could be wrong. | Evaluate on real faces; use a proper face-embedding model (e.g. ArcFace/AdaFace via onnx) if "YOLO-only" is not a hard requirement; calibrate threshold on data. |

## S2 - design gaps
| ID | Debt | Fix idea |
|---|---|---|
| T-06 | **The "fast brain" loop is missing.** Runtime BT node failures are not fed back to the SLM for repair; the SLM is only used for spec->XML and validator-error repair. | Executor publishes failure context; orchestrator asks the SLM for a patched tree; validator; resume or re-run. |
| T-07 | **SLM never trained or evaluated.** Dataset is synthetic with rule-based ground truth, so the best case equals the rule; spec grammar is only sequence/fallback groups with retry (no Parallel, SubTree, blackboard planning). `finetune_bt_slm` has only unit tests of masking/collation. | Run the fine-tune; build a held-out eval from real LLM outputs; extend grammar + data together. |
| T-08 | **LLM output is not validated or retried.** `llm_bridge` publishes whatever JSON text llama-server returns; invalid JSON just fails later in the SLM node. No retry, no conversation memory, single in-flight request. | Validate/repair the task_spec in the bridge (JSON schema, re-ask on error), add timeouts/retry. |
| T-09 | **Extension nodes cannot return values** (no output ports): e.g. `GetPersonPose` cannot hand keypoints to later nodes. Ports are all strings. | Typed ports + output blackboard writes in `ExtNode`. |
| T-10 | **Odom-frame-only motion** (`NavigateTo`, `DriveDistance`) drifts; no TF/map awareness; two nav mechanisms exist (`odom_extension.NavigateTo` and Nav2 via `nav_extension`). | Prefer Nav2 paths; deprecate or document the odom variant. |
| T-11 | **Arm is a stub**: model undecided; only goal construction tested; no gripper, grasp, pick/place (PickObject/PlaceObject were removed because 2D detections cannot support them). | Decide model, configure MoveIt, test in sim, add depth-based grasp targets. |
| T-13 | **Gestures** (wave/point) are hand-tuned geometry thresholds tested on synthetic skeletons; `FollowTarget` bbox-height distance is crude (depth mode exists but untested on a real camera). | Collect pose clips, tune/replace with a learned classifier. |
| T-14 | **Audio**: energy VAD is fragile in noise; wake word is text matching after STT; no engine installed here; Piper/Kokoro/Vosk/faster-whisper untested; mute-while-speaking is a timer. | Install engines, measure on the real mic, consider a neural VAD/keyword spotter. |
| T-15 | **Cluster**: catalog costs are guesses (shipped catalog overflows a 4-core board); no standby scheduler (SPOF for changes); half-partition untested; killing an agent kills its jobs (needs systemd `Restart=always`); `cluster_jobs.yaml` duplicates the launch files so they can drift; only simulated hosts tested. | Test on 2 machines; generate the catalog from launch definitions; add scheduler standby. |
| T-16 | **Two parameter sources**: per-package `launch/<pkg>.launch.py` use each package's `config/`, `framework_bringup` launches use `param/`. `check_params` has a hand-maintained node table (Nav2 nodes not covered). | Make per-package launches read `param/` too; auto-discover nodes in `check_params`. |

## S3 - hygiene / friction
| ID | Debt |
|---|---|
| T-17 | `setup.sh`: only dry-run and `--only build` executed; installs with `--user --break-system-packages`; no version pins/lockfile; llama.cpp built from `HEAD` (unpinned); model step can wait 2 h; NumPy 2 vs apt packages conflict is worked around, not solved (venv or pin). |
| T-18 | Nav2 params are generated from the stock file by `scripts/gen_nav2_params.py` and committed: they will drift from future Nav2 releases; depth-obstacle layer is documented but not enabled; RTAB-Map/RealSense only launch-checked; `my_map.yaml` is unusable. |
| T-19 | Lint: only `object_detection` has the ament flake8/pep257 tests (and they pass from inside the package); the rest of the code is not linted and has unused imports / long lines. Many generated files had odd formatting. |
| T-20 | No `LICENSE` file although package.xml says Apache-2.0; no CHANGELOG; README is long and duplicates `docs/` and `notes/`. |
| T-21 | `Person.distance_m` uses 0 as "unknown" (sentinel); `Face`/`Person`/`Object` messages share no common base; `bt/status` is a hand-built JSON string. |
| T-22 | Test coverage is thin for ROS node wiring (most node classes are exercised only by the integration scripts, which are not part of `colcon test`). Integration scripts live in `tools/integration/` and are run manually. |
| T-23 | Python packages copy files on install, so "edit, relaunch" needs a rebuild (documented, still friction); consider `--symlink-install` + `develop` setup.cfg for dev. |
| T-24 | Version skew risks: ultralytics 8.4.211, torch 2.15, numpy 2.4 in `~/.local`, apt cv/matplotlib; kobuki driver is a source build in `~/ros2_ws` with pinned submodules. |
