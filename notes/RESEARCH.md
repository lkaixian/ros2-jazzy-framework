# Research notes

Questions that needed an answer before building, what was found, how it was checked, and what is still open. Each entry says how sure it is.
(Verification words: **measured** = run here with numbers; **tested** = automated test; **read** = from a document/source, not run; **memory** = from
my general knowledge, not checked here, recheck before relying on it.)

## 1. What does the competition reward? (RoboCup@Home 2027 draft score sheets, rev 2026-10-14)
Source: https://robocupathome.github.io/RuleBook/scoresheets/master.pdf (**read**; a draft, re-read the rulebook before locking plans).
Per try, 7:00 limit, 3 tries: GPSR 1490 (3 commands: understand 80, show plan 100, solve 250 each + 200 interleaved bonus; penalties for custom
operator/rephrasing/ASR bypass), HRIC 1450 (receptionist: doorbell, open door 2x200, seat 2x100, introductions, bag handover/follow/drop),
PnPC 3515 (12 objects x (10+50+40), dishwasher, milk, pouring), Laundry 4415 (fold 800 + 5x400, machine, basket, stacking), Final demo 8450.
Consequence: GPSR and HRIC are mostly software (speech, planning, people); PnPC/Laundry points are manipulation-bound and the hardware
(small Dynamixel arm, Kobuki payload about 5 **memory**) cannot reach most of them. Detail: `TASKS.md`.

## 2. Local LLM on a 4 GB GPU
- **Which model?** Gemma 4 E4B Q4_0 is 4.28 GiB (**read** from the model card repo/notes): too big for a GTX 1650 with context; E2B Q4_0 is 2.6 GiB and
  runs fully on the GPU at about 60 tokens/s (**measured**). The big PC runs E4B; the robot laptop runs E2B (cluster jobs `llm_big`/`llm_small`).
- **Runtime:** official prebuilt llama.cpp release b12375 with CUDA 12.8 and its bundled cudart needs only the NVIDIA driver (**measured**: installed
  and serving without nvcc or sudo). Gemma 4 needs build >= 8637 (**read** in setup notes).
- **Thinking mode:** Gemma 4 emits hidden reasoning that counts against `max_tokens`. A 60-token answer took 431 tokens; plans failed as "not a JSON
  object". `llama-server --reasoning off` (or per request `chat_template_kwargs.enable_thinking=false`) fixed it (**measured**).
- **Constrained decoding:** `response_format: json_schema` makes llama.cpp emit only schema-valid JSON. The schema lists whitelisted macros with
  their args and every leaf node with its real ports (string values). Result with E2B: 1/10 -> 15/15 valid plans (**measured**). The schema is built
  by the validator (`GetCatalog.schema`) so it cannot drift from the catalog.
- **Key order matters (bug found by replaying the bridge's exact request):** llama.cpp emits object properties in the order the schema lists them.
  `nlohmann::json` sorted them, so `args` preceded `macro` and the model chose macros that need no arguments. Fixed with `ordered_json`
  (**tested**, gtest). Sampler tweaks (repeat/DRY penalty, temperature) made no difference; do not chase sampling first.
- **E2B is prompt sensitive:** replacing a prompt line or adding nodes changed which macro it picked until the schema order was fixed. Keep
  `tools/integration/llm_eval.py` (gold macro per command) as a regression check after any prompt, catalog or macro change.
- **Replay method:** put a tiny logging proxy between `llm_bridge` and llama-server, save the request body, then replay it with variants
  (`prompt_lab.py --body`). A rebuilt prompt is only an approximation (the bridge adds e.g. "Known places").
- **Open:** no comparison against E4B quality; only 15 commands; semantic slips remain (e.g. `min_conf: ">0.5"`, the validator only checks structure).

## 3. SLM (230M fine-tune) vs rules
The rule-based generator is exact for every spec in the grammar and about 25000x faster than the model (0.06 ms vs 1.5 s, **measured** earlier), so
`bt_slm` runs rules first; the model only handles retries/odd specs. Re-train on the 42-45-node catalog: rows are long (median 737 tokens), batch 1
fits the 4 GB GPU only up to 768 tokens, and the LLM server must be stopped first (**measured**). Held-out scores are against the rule generator on synthetic
specs, so they prove the model learned the rules, not that it handles real LLM output.

## 4. SROS2 roles and revocation
- **Per-topic permissions** come from a policy XML per enclave (`ros2 security create_permission`); one profile with `node="*"` covers all nodes in
  the enclave (**tested**). Restricted roles must still allow replies for the node's own parameter services (get/set/list/describe/types/
  `get_type_description`) or any rclpy node fails to start (**tested**; found the hard way).
- **Denied writers** fail at `create_publisher` (the enclave refuses the writer locally), so a member cannot even attempt to publish (**tested**).
- **Revocation:** the enclave certificates are valid for 3650 days (sros2 `_utilities.py`, **read**), so removing a member's files is not enough. `ws revoke`
  creates a new keystore (new CA), rotates tokens and the LLM key; the old CA's keys are rejected by new-CA participants (**tested** on one machine).
- **Open:** two real machines, multicast-off discovery over a real LAN, `llama-server` accepting the workspace API key.

## 5. D* Lite as the Nav2 global planner
Core search matches a Dijkstra oracle on random grids and repairs incrementally (**tested**). Plugin was compiled with no optimisation because colcon
passes no build type (**measured** 50 ms vs 8.4 ms for a fresh 240x160 search). An optimised build segfaulted `planner_server` because the static
`dstar_lite_core` was not built with `-fPIC` while linked into a shared plugin; with `-fPIC` it works. How it was found: core dump of a
RelWithDebInfo overlay (gdb), ASan/UBSan on the core (clean), bounds assertions (clean), the linker's "recompile with -fPIC" message under ASan, then a control
build with and without `POSITION_INDEPENDENT_CODE`. Which PIE assumption exactly failed was not traced further.

## 6. Speech
faster-whisper `base.en` int8 on CPU (0.6 s for 2.2 s of speech, **measured**) leaves the GPU to the LLM; Piper `en_US-lessac-medium` for TTS; a
Piper -> Whisper loop returned the exact sentences (**measured**). Real microphone, noise robustness, wake word and Kokoro are untested.
`faster-whisper` cannot decode files with the installed PyAV (`metadata_errors` TypeError); the nodes pass numpy arrays so this is not hit.

## 7. Doorbell detection
Heuristic: one FFT peak holding >= 50 % of the 300-5000 Hz power, loud enough, stable within 4 % for 3 windows of 0.12 s (speech pitch drifts, a bell
does not). Positive control: synthetic ding-dong; negative controls: white noise, quiet tone, short blip, gliding pitch, one Piper speech clip
(**tested**). No real bell recordings yet: the thresholds are guesses.

## 8. Safety and obstacles
Lidar guard windows: 35 degrees either side of the travel direction (front, or rear when reversing), stop at 0.30 m (0.40 m when following a person);
turning in place checks every beam against 0.22 m (Kobuki radius about 0.18 m **memory**). Battery thresholds 14.0 V low / 13.2 V critical are
the Kobuki driver defaults for the 4-cell pack (**memory**, unverified on this pack). Lidar mounting yaw (`scan_yaw_offset`) is unknown.

## 9. Clock sync
Two machines sharing a task need clocks within tens of ms for timestamps, TF lookups and bag replay. chrony with one machine as server (`local stratum 10`
so it also serves offline) is the simple setup (**memory**); `tools/net/timesync.sh` prints/applies it. `chronyc` is not installed on this laptop, so only
the parser is tested.

## Links
- RoboCup@Home rulebook / score sheets: https://robocupathome.github.io/RuleBook/
- llama.cpp releases: https://github.com/ggml-org/llama.cpp/releases (JSON-schema grammar: `response_format` in the server README)
- BehaviorTree.CPP v4: https://www.behaviortree.dev/ (Parallel, Repeat, RetryUntilSuccessful, SubTree)
- SROS2 / DDS-Security: ROS 2 Jazzy security docs (`ros2 security create_permission`, policy XML)
- Nav2 planner plugins: `nav2_core::GlobalPlanner`
