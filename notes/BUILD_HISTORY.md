# Build history

How this framework was built, in order, with what was measured and what went wrong. Dates are from `git log` (machine clock, UTC+8).
Everything below was built and tested on one laptop (GTX 1650 4 GB, 15 GB RAM, Ubuntu 24.04, ROS 2 Jazzy); **nothing has run on the real
robot**. Commit hashes were rewritten on 2026-11-14 (chat-link trailers removed); use messages/dates, not old hashes, to find things.

## Phases

### Phase 1: the harness (2026-11-13 23:14 to 2026-11-14 01:10)
YOLO11 perception, the LLM -> SLM -> validator -> executor chain, audio STT/TTS, extension packages (odom, pid, pose, social, arm, nav),
`cmd_vel_mux`, SLM dataset/fine-tune tooling, central `param/` folder, `setup.sh`, per-package launch files. Decisions that stuck:
validator is the single source of truth for what may run; port values are strings; extensions advertise nodes through a latched manifest;
everything defaults to dry-run. First SLM prompt/inference skew found and fixed with a guard test (`prompt_for`).

### Phase 2: platform (00:45 to 03:50)
Cluster scheduler/agents (DDS job placement, failover), Nav2 + SLAM + RealSense stack and `nav_sim` (a 2D simulator so navigation can be tested
without a robot), person depth ranging, CI (GitHub Actions, ROS Jazzy container), bumper/cliff stop latch in the mux, LLM endpoint failover with
circuit breaker and emergency halt, node sync/async tags, task lifecycle FSM with optional LLM repair, `llm_knowledge.md` with drift guards,
`dash`, D* Lite planner plugin, SROS2 workspaces (`ws`), macros engine, SLM `rules_first`.

### Phase 3: competition planning (2026-11-14 afternoon)
Read the RoboCup@Home 2027 draft score sheets (GPSR 1490, HRIC 1450, PnPC 3515, Laundry 4415 points per try) and wrote the target tasks
(`TASKS.md`), a macro plan (`MACROS_PLAN.md`) and a work plan P0-P6. Hardware reality check from photos/video: a small Dynamixel-class arm with an
OpenCR board and a D435 camera, on a Kobuki that carries about 5 kg: PnPC/Laundry manipulation points are mostly out of reach (B-20/B-24/B-25 deferred by the user).

### Phase 4: capabilities for the tasks
Node outputs (`out_` ports to the blackboard), `WaitForTranscript` + `stt/capture` (so an answer is not turned into a task), `ExtractAnswer`,
`CountPeople/Objects`, `DetectSound` (doorbell), `FindFreeSeat`, 36 macros, `host_info` service, loops/Parallel/foreach in the grammar.

### Phase 5: the real LLM and voice on the 4 GB GPU
Prebuilt llama.cpp (CUDA 12.8, no toolkit), Gemma 4 E2B Q4_0, Piper + faster-whisper. Findings in `RESEARCH.md`: `--reasoning off`, constrained
JSON-schema decoding, the schema key-order bug. Result: 15/15 valid and sensible plans in 1-2 s (was 1/10 valid, about 60 s).

### Phase 6: operations and safety
Dashboard keys and plan tree, preflight, teleop takeover, record/replay, systemd units, lidar guard (incl. turning), battery guard, chrony
tooling, workspace roles + revoke, pinned Python/llama.cpp versions, one lint config.

## Measurements worth keeping
| What | Result | Where |
|---|---|---|
| E2B plans on 10-15 spoken-style commands | 1/10 valid before schema+reasoning fix; 15/15 valid, right macro 8/10 after | `tools/integration/llm_eval.py` |
| Plan latency | about 60 s -> 1-2 s (first request after start about 11 s, cold prompt cache) | same |
| STT | faster-whisper base.en int8 on CPU: 2.2 s audio -> 0.6 s; Piper -> Whisper round trip exact (3/3) | RUNBOOK |
| Voice loop | WAV -> capture -> VAD -> STT -> bt/task -> E2B -> `go_and_say(door, hello)` valid | RUNBOOK |
| SLM (rules are tried first, so it only matters on retries) | old catalog 0.99/0.985 (valid/exact); 42 nodes, 472 rows 0.895/0.795; 768 tokens 0.975/0.915; 45 nodes + Parallel/Repeat, 2835 rows 0.995/0.935 (current, `models/bt-slm`) | `finetune_bt_slm` |
| D* Lite fresh search 240x160 | 50 ms (no optimisation) -> 8.4 ms (Release) | `test_core` `FreshSearchTiming` |
| Nav door route in nav_sim | NavFn 44.8 s, D* Lite 45.9-46.5 s, final error 0.15-0.23 m | `nav_planner_test.py` |
| Tests | about 940 Python tests + 14 gtests pass; 2 `object_detection` lint tests lint `build/` and fail (T-19) | `pytest src` |

## Incidents (what broke, why, what was learned)
| When | What | Root cause | Fix / lesson |
|---|---|---|---|
| 11-14 | `.claude/` agent worktrees committed | `git add -A` | removed, ignored; check `git status` first |
| 11-14 | E2B returned "reply is not a JSON object" | Gemma "thinks" for hundreds of tokens and exhausts `max_tokens` | `--reasoning off` |
| 11-14 | E2B looped `go_home` x12, chose `greet_person` for "go to the door" | `nlohmann::json` sorts keys, so the schema made the model write `args` before `macro` | `ordered_json`; gtest keeps the order |
| 11-14 | `planner_server` segfault (also for the user) | an experimental optimised plugin was installed into the shared `install/`; its static core lib was built without `-fPIC` | fixed `POSITION_INDEPENDENT_CODE`; experiment in scratch overlays only |
| 11-14 | `scout_room` rehearsal ran only 4 steps | validator did not yet know every manifest; retry dropped unknown steps | harness waits for all extensions; hazard documented |
| 11-14 | CI: `cluster_agent_node` did not start | container host id starts with a digit; ROS names must not | `host_service_name()` sanitiser |
| 11-14 | observer role could not start any node | every ROS node serves its own parameter services; the role denied the replies | allowed `/*/get_parameters` etc. replies for restricted roles |
| 11-14 | test stacks piled up (160 processes, 4 GB RAM left) | background `&` jobs ignore SIGINT | stop with SIGTERM, scope by `ROS_DOMAIN_ID` |
| 11-14 | rewritten history | chat-session trailer in commit messages | messages cleaned, force-pushed once at the user's request |
