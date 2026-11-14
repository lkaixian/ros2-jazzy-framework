# Status: what is verified and what is not

> [!WARNING]
> Verified in simulation and CI only. Not verified on a real robot, a real arm, a second machine, or real microphone audio.

Read this before trusting any claim. The honest summary: **everything works in simulation and CI; nothing has run on a real robot.**

## Verified

| Area | Evidence |
|---|---|
| Build | workspace builds on Jazzy in CI on every push; builds on Humble in a `ros:humble` container (`build-test-humble`) |
| Planning chain | task -> stub or real llama-server -> SLM fallback -> validator -> executor, including a rejected-then-repaired attempt |
| Real LLM (Gemma 4 E2B on a 4 GB GPU) | GPSR-style eval: 99 % on 100 generated commands, 95 % on 44 hand-written ones, all plans valid ([Evaluation](evaluation.md)) |
| SLM | held-out valid / exact vs the rule generator ([benchmark](slm-benchmark.md)); published to Hugging Face |
| Execution in a **simulated robot** | a node integrating `cmd_vel` into `odom`: live drive + turn (0.29 m / 0.47 rad for 0.30 / 0.50 requested), dry-run does not move, halt mid-drive, SIGKILL of an extension stops the robot within the mux timeout |
| Safety logic | latch, watchdog, lidar guard, battery guard, mode gate, recovery policy, all against real nodes with fakes ([Testing and CI](testing-and-ci.md)) |
| Controllers | closed-loop unit tests for the PID controllers; gesture rules on synthetic skeletons |
| Trees | greeting and enrolment trees through the real chain against a fake perception node and mock TTS |
| Audio plumbing | a synthesized WAV and mock engines: VAD -> transcript -> `bt/task`; TTS status |
| Cluster, failover, workspaces | simulated hosts and one machine: scheduler placement and failover, LLM endpoint join/leave, emergency halt, SROS2 enforcement, roles and revoke |
| Navigation | Nav2 + D* Lite door route in `nav_sim` (45.5 s, controller-limited) |
| Groot2 feed | port opens and the node palette is written |

## Not verified

| Area | Why it matters |
|---|---|
| Any **real robot** (Kobuki base) | bumper/cliff latch, lidar and battery thresholds, `scan_required`, real stopping distances |
| Real RPLIDAR with SLAM and real RealSense depth | only a simulated lidar was used |
| Arm and MoveIt execution | only goal construction is tested; the arm model is undecided |
| YOLO nodes on a real camera; face and pose models | the scene summary was fed synthetic skeletons; gesture accuracy on real video unknown |
| Real speech recognition, Piper and Kokoro playback, live microphone | the word error rate is unmeasured |
| Any second machine or WiFi | cluster, workspaces, LLM failover across hosts |
| `setup.sh` apt/pip/llama.cpp/model steps | only a dry-run, and `--only build` |
| Nav2 on Humble | `nav2_params.yaml` is generated from the Jazzy defaults; nothing ran on a Humble machine |
| The Groot2 GUI itself | the server side is tested, not a GUI session |
| The SLM on real LLM output | scored against synthetic specs and the rule generator only |

## Known limitations

* The refusal guard for arm tasks is a regex heuristic (`shortcut:` in `bt_orchestrator/policy.py`).
* `RecoveryPolicy.TABLE` is a starting point to tune with real runs.
* `DescribePerson` reports the top clothes colour only.
* Battery thresholds come from the Kobuki driver's documented defaults.
* Do not mix Jazzy and Humble machines on one DDS domain.

The engineering notes in the repository (`notes/`: STATUS, RUNBOOK, GOTCHAS, BACKLOG, technical_debt) hold the detailed history and the open list.
