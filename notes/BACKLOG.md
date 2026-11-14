# Backlog (logged 2026-11-14, to start the next session)

## NEXT SESSION (written 2026-11-15; read this first, older lists below are mostly done)
State: all pushed. SLM v5 is `models/bt-slm` (v4 kept as `models/bt-slm-v4`) and on the public HF repo. Stack and llama-server are stopped. Disk ~6 GB free.
`src/dstar_lite_planner/src/planner.cpp` carries a Humble-compat `#if __has_include` change from the other agent (builds on Jazzy; not tested on Humble).

Needs you (hardware or a human):
1. Record ~100 real sentences on the real microphone, measure the STT word error rate (hints and snapping are in, unmeasured on real audio).
2. Real robot bring-up (B-20): tune `RecoveryPolicy.TABLE` (`bt_orchestrator/mode.py`) and the mission budget on real runs; check the Kobuki battery/lidar thresholds, chrony, a hardware e-stop.
3. Arm (B-25): HRIC door/handover, PnPC, Laundry rehearsals; then the arm-driver decision (Jazzy-only, see memory: own driver or vendor source in a Jazzy workspace, no translator).
4. Two machines (B-24): `ws` roles/revoke, `timesync.sh`, `host_info`.
5. Your call: `tools/systemd/install.sh --enable`; restart Claude Code and run `/mcp` to use the Colab MCP (needs an open Colab notebook).

Software-only, can start without you:
6. Residual GPSR misses: question phrasings ("Is there anyone in the right room", "Do you see a laptop anywhere"); refusal verbs in `bt_orchestrator/policy.py` are a heuristic. Re-run `llm_eval.py --gen 100 --seed 1` and `--jsonl tools/gpsr/handwritten.jsonl` after any prompt change.
7. v5 can still emit Parallel on one extension (old `bt_dataset_v3` rows have it; the validator rejects it, so valid caps at 0.975). Filter the old rows and retrain only if the retry path starts failing on it.
8. Add real GPSR wording to `tools/gpsr/handwritten.jsonl` when you have it. Add `DescribePerson` beyond the top colour (T-31).
9. Disk is 95% full, mostly outside the project: find what else to free (`du` on home); the llama.cpp `cudart` folder is required, keep it.


Picked up from the end of a long session. Order inside each group is the suggested order. IDs B-xx
are new; T-xx point at `technical_debt.md`. Update this file as items are done (delete or tick).

## NEXT: what to do first (written 2026-11-14 night; supersedes the order below where they differ)
State: everything is committed and pushed; CI was last seen green except for races fixed in the final commits (check the Actions tab first). `llama-server`
(E2B) may still be running on port 8081 (stop it before training on the GPU). SLM v4 is at `models/bt-slm` and on the public HF repo `larmkaixian/bt-sim-slm`.

1. **DONE 2026-11-14 (98 % generated, 95 % hand-written, see RUNBOOK "misses fixed"); leftovers: refusals that also drive, question phrasings.** Was: Read the 14 misses of the GPSR-style baseline and fix them without training** (86 % -> aim above 95 %). Run `llm_eval.py --gen 100 --seed 1` with a
   stack up (see `docs/operator-guide.md` "Test and rehearse"), read the `MISS` lines, then fix in this order: macro descriptions (`macros.yaml`), worked examples
   (`llm_knowledge.md`, each is test-validated), the system prompt in `llm_bridge_node.cpp`. Known misses: doorbell -> `WaitForTranscript` instead of
   `DetectSound`; "patrol N times"; "come with me"; one refusal that drove somewhere. Re-run `llm_eval.py` (15 commands) and `prompt_lab.py` afterwards, the
   model is prompt sensitive (T-21).
2. **DONE (44 commands in `tools/gpsr/handwritten.jsonl`; add the real GPSR wording when you have it).** Was: Make the evaluation trustworthy: add 30-50 hand-written commands in your own words (and the real GPSR wording if you have it) to a JSONL next to
   `tools/gpsr/commands.py`; the generated set is template-based and can flatter the model.
3. **DONE 2026-11-14 (tests added).** Was: Close the hazards found today: T-22 (retry path silently drops unknown steps: fail instead), T-21 value checks in the validator (`min_conf ">0.5"` passes),
   T-25 (`Parallel` on one extension). Small, testable, no hardware.
4. **Not needed now (95 %+ without training).** Was: Only if step 1 still leaves misses: fine-tune E2B on a free Colab T4 (QLoRA, fp16, checkpoints). Data = `tools/gpsr/commands.py` gold plans (command -> plan,
   validator-checked by `test_gpsr_commands.py`); build the notebook, train, export a LoRA adapter, load with `llama-server --lora`, compare with `llm_eval.py --gen`.
   Gemma 4 support in peft/trl and the adapter conversion are untested. Do NOT train on this laptop (4 GB).
5. **Hints + fuzzy snapping DONE 2026-11-14 (`stt_pipeline/vocab.py`: Whisper `initial_prompt` from places.yaml, `snap_places` in `transcript_to_task`; Vosk ignores hints); still open: ~100 real recordings for the word error rate.** Was: STT without training first: vocabulary hints (`hotwords`/`initial_prompt` in faster-whisper) + fuzzy matching of heard names/places in `stt_node` / `ExtractAnswer`;
   record about 100 real sentences from the real microphone to measure word error rate before any Whisper fine-tune. TTS fine-tune: skip.
6. **Next SLM version (optional):** the rules run first, so gain is small; if you do it, add the new row to `docs/slm-benchmark.json`, run
   `tools/slm/plot_benchmark.py`, then `tools/slm/publish_hf.py` (this updates the PUBLIC repo).
7. **Still software-only:** `DescribePerson` is done for the top colour (T-31), HRIC door/handover and PnPC/Laundry rehearsals (need the arm),
   a two-machine test of `ws` roles/revoke, `timesync.sh` and `host_info` (needs a second machine; deferred B-24).
8. **Housekeeping:** the local branch `backup-before-history-rewrite` is deleted (2026-11-14); prune `models/bt-slm-v1..v3`,
   `bt_dataset*` dirs and `~/llama.cpp` leftovers when disk gets tight (about 6 GB free); `tools/systemd/install.sh --enable` once the stack should survive reboots.

Blocked on hardware or on the user (unchanged): B-20 real robot bring-up, B-24 two machines, B-25 arm (all deferred by the user), B-22 detector images,
B-23 real microphone/speaker/doorbell recordings, chrony and the Kobuki battery/lidar thresholds on the real robot, a hardware e-stop.

## Start here (older list, partly done: see the ticks)
- [x] **CI check (latest run green, 2026-11-14).** The last full GitHub run failed only in `Sim - workspaces (create, discover, join) +
  SROS2 enforcement` (the secured DDS probes printed neither RECEIVED nor NOTHING in the container; it
  passes locally). Since then: `ros-jazzy-rmw-cyclonedds-cpp` added to the apt list and the probe now
  prints its stderr. Look at the newest run (`gh run list`), read the probe stderr, fix. Everything
  else in CI passed in the earlier run, including the D* Lite Nav2 sim.
- [x] **Macro list: 36 macros, all whitelisted** (`param/macros.yaml`, reference in `docs/macros.md`, regenerated by `tools/gen_macro_docs.py`, a test fails when it is stale). Add yours: an entry + `example_args` + a name in `whitelist:`; `test_macro_library.py` validates it against the real catalog. A macro body cannot use `foreach`.
  macros. Add yours: an entry + `example_args`; `test_macro_library.py` validates every macro against
  the real catalog. Remember a macro cannot use decorators (grammar = sequence/fallback groups + retry).
- [x] **Define the competition tasks** (GPSR, HRIC, PnPC, Laundry: see `notes/TASKS.md`; next: turn them into acceptance tests). Write the target tasks down (even 5 lines each); they decide the
  priorities below and become acceptance tests. Nothing in the repo defines them yet.

- [x] **Macros: `notes/MACROS_PLAN.md` executed as far as hardware allows** (P0-P3 and P5 done, P6 preflight extended, P2 `DescribePerson` open, P4 arm deferred).

## Can be built and tested without the robot
- [x] **B-01 Node outputs (T-09), DONE 2026-11-14:** extension handlers return `(ok, msg, {"out_x": v})`, manifests list `out_x` as an optional port, `bt_executor` writes it to the blackboard; round trip tested by `tools/integration/outputs_test.py` (also in CI). Nodes still to use it: see `MACROS_PLAN.md` P2. Original: Extension nodes cannot return values (no output ports, all ports are
  strings), e.g. `GetPersonPose` cannot hand keypoints on. Blocks genuinely complex tasks. Suggest first.
- [x] **B-02 Preflight check.** `ros2 run framework_bringup preflight`: lidar publishing, TF chain sane,
  mux alive, `LLM_API_KEY` set, workspace/SROS2 state, LLM endpoints healthy, executor in dry-run.
- [x] **B-03 Richer task grammar (DONE 2026-11-14):** `repeat`, `Parallel` (success/failure counts) and `foreach` (unrolled at plan time, task specs only); schema, prompt, `fallback.py`, dataset sampler and tests updated together; `SubTree` not needed (macros expand inline). The SLM is re-trained with the new data (see RUNBOOK). Original:
  `bt_slm/fallback.py` AND the dataset generator together (the SLM prompt format must stay in sync).
- [x] **B-04 Obstacle awareness for plain motion (DONE 2026-11-14):** shared `ScanGuard`: `DriveDistance`/`NavigateTo` stop at 0.30 m in the travel direction, `FollowTarget approach` at 0.40 m, `RotateBy` refuses with anything inside 0.22 m. Tested with fake scans (`guard_test.py` 6 checks). Set `scan_required: true` on the real robot; the lidar mounting yaw and the Kobuki radius margin are unverified; a hardware e-stop is still required. Original:
  drive blind; route them through `collision_monitor` or a scan check. Document a hardware e-stop.
- [x] **B-05 Recording/replay (DONE 2026-11-14):** `tools/record_run.sh NAME [--images]` (mcap, latched QoS kept; recorded and read back with `ros2 bag info`); replay: `ros2 bag play <dir> --clock`. Original: rosbag2 record profile for a run (topics list, QoS), plus a replay recipe.
- [x] **B-06 Time sync (DONE 2026-11-14 as tooling):** `tools/net/timesync.sh server|client|status` (chrony; prints the config, `--apply` installs it) and `preflight` reads `chronyc tracking` (warn > 50 ms, fail > 0.5 s). Only the parser is tested: chrony is not installed on this laptop, nothing was run on two machines.
- [x] **B-07 Manual override (DONE 2026-11-14):** `ros2 run framework_bringup teleop` (w/a/s/d, first key also halts the executor) publishes to `teleop/cmd_vel`, now the FIRST (highest priority) input of `cmd_vel_mux`; releasing the keys returns control after the 0.5 s watchdog. Tested: key map + config; not tried on the real robot. Original: Teleop path into `cmd_vel_mux` at the highest priority (and a way to
  take control back during a run).
- [x] **B-08 Battery/health (DONE 2026-11-14):** `host_info` service (laptop/PC), Kobuki battery in `dash`, and `battery_guard_node` halts the running plan when the base battery is below 13.2 V for 3 s (not charging); `battery_test.py` in CI. Thresholds are the driver defaults, unverified on this pack; teleop and Nav2 are not stopped by the guard.
  `dash`, warn/halt under a threshold.
- [~] **B-09 Process supervision on the robot (units written 2026-11-14, NOT enabled or run):** `tools/systemd/` user units `robocup-llm` and `robocup-stack` with `Restart=always`, dry-run kept on; `systemd-analyze --user verify` is clean; install with `tools/systemd/install.sh [--enable]`. Cluster agent as a service still open. Original: systemd units (or the cluster agent as a service with
  `Restart=always`) so a reboot brings the stack back. Pairs with T-15.
- [x] **B-10 Hygiene (DONE 2026-11-14):** LICENSE (Apache-2.0), one `.flake8` for the repo (clean on `src` and `tools`, CI step), unused imports removed, `requirements-lock.txt` (pip constraints, NumPy 2 story) and a pinned llama.cpp tag in `setup.sh`. Still open: the two `object_detection` ament lint tests lint `build/` when run from the repo root (T-19).
  (T-19), `setup.sh` version pins + a tested venv-or-pin story for NumPy 2 (T-17).
- [x] **B-11 D* Lite speedup (DONE 2026-11-14, after a failed first attempt).** Two findings: (1) colcon builds with no build type, i.e. no optimisation: the plugin now defaults to Release; fresh search on 240x160: 50 ms -> 8.4 ms. (2) ROOT CAUSE of the planner_server segfault that the first attempt caused: `dstar_lite_core` is a STATIC library linked into the SHARED plugin and was not position independent (it only linked because Ubuntu's GCC defaults to PIE); optimised, the PIE-compiled core crashed in `Core::reset` -> `insert`. Fix: `POSITION_INDEPENDENT_CODE ON` on that target. Evidence: the crashing config (RelWithDebInfo) crashes planner_server on the first goal; the identical config plus `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` plans correctly; the repo fix (Release -O3 + PIC) passes `nav_planner_test.py dstar_lite` (45.9 s, 11.11 m, 0.21 m error) and all 10 core tests; `-D_GLIBCXX_ASSERTIONS` (no bounds failure) and ASan/UBSan on the core in unit tests (clean) ruled out an indexing bug in the algorithm. Not understood in detail: exactly which PIE assumption broke (a PIE object in a shared object is invalid, so no further hunt was done). Original: Reuse the cost-grid buffer and convert only changed
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
