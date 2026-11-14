# Runbook

## Every terminal
```
source ~/ros2-jazzy-robocup-fw/env.sh     # created by ./setup.sh --only build; puts /usr/bin first, sources ROS + workspace
# or by hand:  export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash; source install/setup.bash
```
(`~/pyvenv.cfg` makes `~/bin/python3` hide system packages; always put `/usr/bin` first.)

## Setup (new machine)
```
./setup.sh --dry-run          # read the plan
./setup.sh                    # apt, rosdep, pip, llama.cpp, LLM + SLM base downloads, build
./setup.sh --only build       # just colcon build + env.sh   (this is the only step ever executed so far)
./setup.sh --only python      # incl. a NumPy-2-compatible matplotlib (needed for detector training)
./setup.sh --only slm         # optional: SLM dataset + fine-tune (never run yet)
```

## Build and test
```
colcon build --symlink-install      # ament_python COPIES .py into install/: rebuild after Python edits
ls src/*/test/test_*.py | grep -vE "test_(flake8|pep257|copyright)" | xargs python3 -m pytest -q -p no:cacheprovider
./build/bt_executor/test_registry ; ./build/llm_bridge/test_llama_client       # C++ gtests
ros2 run framework_bringup check_params          # starts each node on domain 97, compares file vs loaded params
```
Run pytest from the workspace root with the workspace sourced (tests import installed packages).

## Macros (shorter LLM output = faster plans)
`param/macros.yaml`: named step lists with parameters. The LLM writes `{"macro": "go_to_place", "args":
{"place": "door"}}`; the orchestrator expands it to real nodes before the SLM/rule generator (plan keeps
the short form). The macro list is appended to the LLM's node listing by the validator. Add a macro =
add an entry with `example_args`; `test_macro_library.py` expands and validates every macro against
the real catalog, so a typo in a node or port fails the build. Unknown macro / missing arg -> the plan
is refused with a message naming the problem.
The SLM node has `rules_first: true`: the exact rule-based generator runs first (0.06 ms vs 1.5 s for
the 230M model on the GTX 1650) and the model only runs for retries or specs the rules cannot handle.

## Workspaces (SROS2 keys + domain + LLM key, joined by token)
```
ros2 run framework_bringup ws create lab          # first machine: keystore, TLS cert, token; prints the invite
source ~/.robocup/workspaces/lab/env.sh           # every terminal on it (domain, SROS2 Enforce, LLM key)
ros2 run framework_bringup ws serve lab           # keep running while others join (announces on UDP 47611)
ros2 run framework_bringup ws discover            # other machine: what is announced on this LAN
ros2 run framework_bringup ws join lab --token T  # or: ws join --invite 'rcw1~lab~<ip>~47610~<fp>~<token>'
source $(ros2 run framework_bringup ws env lab)
ros2 run framework_bringup ws members lab | list | invite lab
python3 tools/integration/workspace_test.py       # whole flow + outsiders on one machine
```
Authentication + encryption for every ROS node; each member may use every topic (no per-topic
permissions), and removing a member means creating a new workspace. Verified on ONE machine only.

## Secure the LLM link and discovery
```
export LLM_API_KEY=...        # bridge machine: sent as `Authorization: Bearer` (name set by api_key_env)
export LLAMA_API_KEY=...      # llama-server machine, same value (jobs inherit the agent's env)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$PWD/tools/net/cyclonedds_lan.xml
export ROBOCUP_PEER=<other PC's address>    # discovery without multicast, only that peer
python3 tools/integration/transport_test.py                          # TCP reuse
TRANSPORT_REQUIRE_KEY=1 python3 tools/integration/transport_test.py  # key accepted
```
The bridge->llama-server link is HTTP over TCP with one reused, keep-alive connection. DDS (node to
node) is UDP; the unicast config only hides the robot from strangers, SROS2 is the authentication.

## See what is running (terminal dashboard)
```
ros2 run framework_bringup dash              # 1 Hz table: LLM endpoints, cluster jobs, every node ONLINE/BUSY/ERROR/OFFLINE
ros2 run framework_bringup dash --logs       # verbose: all /rosout lines (--level warn, --node llm_bridge)
ros2 run framework_bringup dash --once       # one frame, for pipes/CI
```
Read-only. "Expected" nodes come from the table in `check_params`; BUSY only means bt_executor is running a tree.

## Launch
```
ros2 launch framework_bringup framework.launch.py [args]        # or: ros2 launch launch/framework.launch.py
   switches: enable_object/person/face, enable_audio, enable_sensors, enable_navigation (off by default),
             enable_arm/pose/odom/pid/social/nav (extensions, on by default)
   args:     image_topic, show, start_llama_server, llama_model, llama_hf_repo, llama_port, slm_model_path,
             dry_run (default true), auto_run (default false), slam:=slam_toolbox|cartographer|rtabmap|none,
             map:=<yaml>, initial_pose:=x,y,yaw, params_dir:=...
             cmd_vel_topic:=commands/velocity (mux output; goes to extensions.launch.py)
ros2 launch framework_bringup pipeline.launch.py                  # LLM/SLM/validator/executor only
ros2 launch framework_bringup nav_sim.launch.py initial_pose:=0.5,0.5,0.0   # Nav2 + simulated robot, no hardware
ros2 launch framework_bringup cluster.launch.py role:=agent|scheduler|both host_id:=pi roles:=core tags:=camera
```
Params: edit `src/framework_bringup/param/*.yaml` (no rebuild needed). Docs: `docs/navigation.md`, `docs/cluster.md`.

## Behaviour trees by hand
```
ros2 run bt_validator validate_bt --list-nodes [--labels a,b,c]
ros2 run bt_validator validate_bt tree.xml
ros2 run bt_orchestrator submit_bt tree.xml --run         # needs bt_validator + bt_executor running
ros2 service call /bt_executor/halt std_srvs/srv/Trigger
```
Examples: `src/framework_bringup/behavior_trees/{greet_person,enroll_person}.xml`.

## Integration checks without hardware (`tools/integration/`)
```
tools/integration/greet_test.sh greet_person.xml alice mytest        # fake perception + mock TTS
tools/integration/itest.sh live false '<spec json>' none              # full chain to a simulated robot
tools/integration/itest.sh halt false '{"goal":"x","steps":[{"node":"DriveDistance","ports":{"meters":"3.0"}}]}' 2.5
python3 tools/integration/stop_latch_test.py                          # bumper/cliff latch on the mux (own domain 77)
python3 src/cluster_core/test/integration/it_cluster.py               # cluster: 3 simulated hosts, failover timing
```
They use an isolated DDS domain (default 45), log to `/tmp/robocup_it`, and clean up on exit.
`tools/integration/bbox_probe.py` + `object_detection_node` check boxes on a photo;
`tools/strict_sub.py` is the "fail loudly if nothing matched" replace helper (see GOTCHAS).

## Detector training (boxes only, sized for 4 GB)
```
ros2 run object_detection train_detector --data my_items.yaml --base yolo11n.pt --epochs 100
# set object_detection_node.model in param/perception.yaml to the printed best.pt
ros2 run bt_slm make_bt_dataset --labels @labels.txt --out-dir bt_dataset
```

## Mapping / navigation on the real robot (not yet done; see docs/navigation.md)
```
# Kobuki driver comes from ~/ros2_ws:  ros2 launch kobuki_node kobuki_node-launch.py
ros2 launch framework_bringup sensors.launch.py
ros2 launch framework_bringup extensions.launch.py cmd_vel_topic:=commands/velocity   # safety mux -> Kobuki input
ros2 launch framework_bringup navigation.launch.py slam:=slam_toolbox
# then BT node SaveMap file="room1"  ->  navigate with map:=$HOME/maps/room1.yaml
```

## Local LLM + voice on this machine (set up 2026-11-14, GTX 1650 4 GB)
- **llama.cpp**: official prebuilt release b12375 (CUDA 12.8 + bundled cudart, no toolkit, no sudo) in `~/llama.cpp`,
  wrapper `~/.local/bin/llama-server`. Reproduce: `./setup.sh --only llamacpp --backend prebuilt`.
- **LLM = Gemma 4 E2B Q4_0** (`models/llm/gemma-4-E2B-it-Q4_0.gguf`, 2.6 GiB; E4B does not fit this GPU well):
  `llama-server -m models/llm/gemma-4-E2B-it-Q4_0.gguf --host 127.0.0.1 --port 8081 -c 6144 -np 1 -ngl 99 --reasoning off`
  then `ros2 launch framework_bringup pipeline.launch.py llama_url:=http://127.0.0.1:8081`.
- **`--reasoning off` is required**: Gemma 4 otherwise "thinks" for hundreds of tokens, exhausts `max_tokens` and the JSON is cut
  off (measured: a 60-token answer took 431 tokens; plans went from ~60 s to 1-10 s and the "reply is not a JSON object" errors stopped).
- **Constrained decoding**: `bt_validator/catalog` now returns a JSON schema (`GetCatalog.schema`) listing the allowed macros (the
  `whitelist:` in `param/macros.yaml`), their args, every leaf node and its real ports; `llm_bridge` sends it as
  `response_format: json_schema`, so call-syntax slips (`follow_person()`), invented macros/ports and non-string values cannot be
  produced. Before it: 1/10 valid plans from E2B; see `tools/integration/llm_eval.py` for the measurement.
- **TTS**: Piper `en_US-lessac-medium` in `models/tts/...`; use `audio.launch.py tts_engine:=piper piper_model:=$PWD/models/tts/en/en_US/lessac/medium/en_US-lessac-medium.onnx`.
- **STT**: faster-whisper `base.en` int8 on CPU (`models/stt`): 2.2 s of speech -> 0.6 s; Piper -> Whisper round trip returned the
  exact sentences (3/3). Keep STT on the CPU: the GPU is shared with the LLM.
- Disk is tight (about 8 GB free on `/`): do not add the E4B model here.
- **Gotcha that cost an afternoon (2026-11-14): JSON-schema key order.** llama.cpp emits object properties in the order the schema lists
  them. `nlohmann::json` sorts keys alphabetically, so `args` came before `macro` and E2B had to write the arguments before choosing the
  macro: it picked macros without arguments (`go_home` x12, `greet_person` for "go to the door"). Fixed with `nlohmann::ordered_json` in
  `llama_client.cpp` (gtest `SchemaKeepsPropertyOrderAndRepeatPenalty`). After the fix: 15/15 valid and sensible plans in 1-2 s
  (`tools/integration/llm_eval.py`); sampler settings (repeat/DRY penalty, temperature) made no difference, the cause was never sampling.
  `tools/integration/prompt_lab.py --body <captured request>` replays the bridge's exact prompt+schema (capture with a tiny logging proxy
  in front of llama-server); rebuilt prompts are only an approximation (the bridge adds e.g. "Known places" to NavigateToPlace).
- **Test hygiene gotcha:** processes started with `&` from a non-interactive shell ignore SIGINT, so `kill -INT` on a stray
  `ros2 launch` does nothing; use SIGTERM. Scope kills by the test's `ROS_DOMAIN_ID` (read `/proc/<pid>/environ`), never by name.
- **Verified end to end (isolated domain, mock TTS):** Piper-synthesised WAV -> `audio_capture` -> `vad_node` -> `stt_node`
  ("Go to the door and say hello.") -> `transcript_to_task` -> `llm_bridge` -> `go_and_say(door, hello)` -> valid plan. First
  request after start takes ~11 s (cold prompt cache), then 1-2 s. Real microphone/speaker not tried.

## Operator tools added 2026-11-14
- `ros2 run framework_bringup preflight [--no-robot]`: PASS/WARN/FAIL for key, SROS2, mux, executor dry-run, LLM, /scan, TF, clock sync, disk.
- `ros2 run framework_bringup teleop`: keyboard takeover (highest mux priority, halts the running plan on the first key).
- `ros2 run framework_bringup dash`: keys j/k select, r/R restart node, s/S stop system, w/W restart system, h halt task; shows Kobuki battery.
- `ros2 run cluster_core host_info`: CPU/RAM/GPU/VRAM/temperature/battery of each machine with a cluster agent.
- `tools/record_run.sh NAME [--images]`: rosbag2 recording of a run; `tools/systemd/install.sh [--enable]`: restart-always user units.

## SLM re-train on the 42-node catalog (2026-11-14)
- Data: `ros2 run bt_slm make_bt_dataset --out-dir bt_dataset_v2 --n 12000` (2 s). Rows are long now (median 737 tokens, 92 % <= 1024).
- GTX 1650 4 GB, batch 1: `--max-len 1024` runs out of memory; `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` + `--max-len 512`
  trains on only 12 % of the rows (472 of 4000): held-out valid 0.895 / exact 0.795 (old 3000-row run on the smaller catalog: 0.99 / 0.985).
  `--max-len 768` keeps ~60 % of the rows; result in the next entry if it finished. The LLM server and training cannot share the GPU
  (stop `llama-server` first, restart it afterwards). The SLM only runs on retries / odd specs (`rules_first`), so none of this blocks planning.

- **SLM (2026-11-14), `--max-len 768`, 2392 of 4000 rows:** held-out valid 0.975 / exact 0.915 vs the rule generator on 200 rows (`models/bt-slm-v3`,
  not wired into any launch default; it never sees `repeat` yet because it was trained before that grammar change). 512 tokens (12 % of rows) gave 0.895 / 0.795.
- **Planner crash incident (2026-11-14):** an experiment that built `dstar_lite_planner` with optimisation was installed into the shared `install/`
  and `planner_server` crashed for everyone using the workspace. Reverted, then root-caused (static core library without -fPIC linked into the shared plugin, see B-11) and fixed properly. Lesson: try builds with `colcon build --build-base X --install-base Y` and
  source `Y/setup.bash` on top (as done for the investigation); never install an experimental binary into the shared `install/`.
- **LLM re-check after the HRIC/doorbell/seat macros (31 whitelisted macros, 2026-11-14):** `llm_eval.py` 15/15 valid, right macro 8/10; the two "misses"
  were reasonable alternatives (`go_find_person` + `greet_person`; `go_find_person` + `guide_to`). Prompt growth is not hurting E2B so far.

## Sharing the fine-tuned SLM
The weights are 443 MB (`model.safetensors`): too big for git (GitHub rejects files over 100 MB; `models/` is gitignored on purpose). Two ways, both
need your decision and your login (nothing is uploaded automatically):
- **Hugging Face Hub (recommended):** `hf auth login`, then `python3 tools/slm/publish_hf.py --model models/bt-slm --repo <you>/<model-name> --valid <v> --exact <e>`
  (private by default; `--public` to share). Pick a neutral `<model-name>`: the model card is neutral too (no project name, no repo link unless you pass `--project-url`), and the weight files contain no project strings (checked with grep). Others get it with `SLM_WEIGHTS_REPO=<you>/<model-name> ./setup.sh --only slmweights`.
- **GitHub release asset:** `gh auth login`, `tools/slm/release_github.sh slm-v4` creates a DRAFT release with a tarball; publish it on github.com
  when you are happy. Extract with `tar -xzf bt-slm-<tag>.tar.gz -C models`.
- **License:** the base model is under the LFM Open License v1.0: redistribution of a fine-tune is allowed if you ship the license, mark the files as
  modified and keep the attribution (both scripts do), and commercial use is restricted above 10 M USD annual revenue. Check it fits your use.
- **Benchmark chart:** `docs/slm-benchmark.md` (table + `docs/img/slm-benchmark.png`, redraw with `tools/slm/plot_benchmark.py`); `publish_hf.py` puts the chart on the model card.
- **Published (2026-11-14):** the first fine-tuned SLM (`bt-slm-v3`, valid 0.975 / exact 0.915, older catalog) is on the Hugging Face Hub as **public**
  `larmkaixian/bt-sim-slm` (made public the same day at the owner's request; it was uploaded private first). Update it any time by running `publish_hf.py` again (each run is a new commit; old versions stay in the repo history and can
  be fetched with `--revision`). Visibility is changed on the repo's settings page.

## SLM v4 (2026-11-14 evening): trained on the 45-node catalog with Parallel/Repeat
Data `bt_dataset_v3` (12000 rows, 1724 with Parallel, 1302 with Repeat); first 6000 rows, 2835 usable at `--max-len 768`, 1 epoch, batch 1, about 50 minutes
including the evaluation. Held-out valid **0.995** / exact **0.935** (200 rows, against the rule generator on synthetic specs). Promoted to
`models/bt-slm` (auto-discovered by `bt_slm_node`, CPU, only used on retries); the earlier models are `models/bt-slm-v1..v3`. The public Hugging Face repo
still holds the first version (v3).

## GPSR-style baseline (2026-11-14): E2B + schema + SLM v4, 100 generated commands (`llm_eval.py --gen 100 --seed 1`)
100/100 plans valid; **86 % meet what the command asked** (navigation 30/35, people 36/41, refuse 10/12, vision 10/12). Misses seen: the doorbell command
planned `WaitForTranscript` (60 ms timeout) instead of `DetectSound`, "patrol N times" and "come with me" phrasings, one refusal that drove somewhere.
The command templates are ours (`tools/gpsr/commands.py`), not the official GPSR sentences. Next: look at all misses (`MISS` lines), fix prompt/examples/macro
descriptions first; fine-tune E2B on Colab (free T4) only if misses persist: the generator's gold plans are the training data.

### Update (2026-11-14, later): misses fixed without training
Baseline re-measured with the pose extension up (the 86 % run was missing it: `FindFreeSeat`/`CountPeople` plans were invalid): 90 %. Then, one change at a time
(`llm_eval.py --gen 100 --seed 1`): macro descriptions 94 %, + worked examples 99 %, + numeric-port rule, `-np 1 -c 6144`, `FindFreeSeat` "ONLY for empty seats",
`describe_person` macro 98 % (100/100 valid). Hand-written set (`tools/gpsr/handwritten.jsonl`, 44 commands in our own words, `llm_eval.py --jsonl`): 93 % -> 95 %, all valid.
Still missed: "take/put X to Y" sometimes also drives (refuse 10/12), question phrasings ("is there anyone in the right room" -> `DetectPerson` + `Speak`).
Findings: (1) `llm_eval.py` scores were shifted by one whenever a plan from an earlier run was still in flight (now drains 60 s first); starting `extensions.launch.py` twice
or leaving old nodes running exhausts the DDS participants of a domain (`error creating node`). (2) The prompt is 3.2-3.7k tokens: `llama-server` ran 4 slots sharing 4096, so
use `-c 6144 -np 1` (unit, runbook and operator guide updated). (3) The model wrote `min_conf: ">0.5"` by itself; the validator now rejects non-numbers and a knowledge line says so.
Not trained, so no HF update was needed.

## Robot mode, recovery and mission policy (2026-11-14, later)
`robot_mode_node` + `bt_orchestrator/mode.py`: init / safety / teleop / mission budget -> `robot/mode`; `RecoveryPolicy` picks retry / replan / go_home / ask_human / abort per failure
(budget = `max_repairs`, still 0 by default); `mode_gate` in `bt_executor` (off by default) refuses runs unless `robot/mode` is fresh and `run_ok`. Real-node tests:
`tools/integration/mode_test.py`, `tools/integration/recovery_test.py` (found two things: the executor's `bt/status` is only published once, so readiness is the `halt` service;
`go_to_place` already retries once itself, so one run makes two goals). Groot2: `groot2:=true` (see operator guide). Details: `docs/operator-guide.md`.

## SLM v5 (2026-11-14): long plans
`bt_slm make_bt_dataset --profile heavy|max` (normal rows never exceeded 768 tokens, heavy median about 1.4k, max up to 3.8k). v5 = v4 + 860 rows (150 normal / 500 heavy / 250 max,
`--max-len 2560`, Adafactor lr 2e-5, `--grad-ckpt`, about 40 minutes; AdamW + fp32 does not fit 4 GB at these lengths). 20 held-out rows per set, valid / exact:
normal 1.00 / 1.00 (v4 1.00 / 0.90), heavy 0.90 / 0.70 (v4 0.55 / 0.35), max 0.90 / 0.80 (v4 0.80 / 0.50). Promoted to `models/bt-slm` (v4 kept as `models/bt-slm-v4`);
`max_new_tokens` is now 3072. **Published** to the public `larmkaixian/bt-sim-slm` (2026-11-14, the earlier version stays in the repo history). On the usual 200 normal rows under today's validator: v4 0.970 valid / 0.935 exact, v5 0.975 / 0.980 (5 of those 200 reference trees are Parallel-on-one-extension, which the validator rejects since T-25, so valid cannot exceed 0.975; v4's original 0.995 predates that rule). Pruned the same day: `models/bt-slm-v1..v3`, the duplicate v5 dir and the old `bt_dataset*` dirs (kept: `bt-slm`, `bt-slm-v4` as rollback, `bt_dataset_v3` for the dataset publisher, `bt_dataset_v4_*`; the llama.cpp `cudart` folder is NOT a leftover, the `llama-server` wrapper needs it). Gotcha: training stdout is block-buffered into a log file, use `PYTHONUNBUFFERED=1`.

## Published dataset (2026-11-14)
`larmkaixian/bt-sim-dataset` (public, Apache-2.0): config `slm` = `bt_dataset_v3` (11,760 train / 240 validation rows: task spec -> BehaviorTree.CPP XML, validator-labelled) and
config `commands` = 1,000 GPSR-style commands with expectations and gold plans (`tools/gpsr/commands.py`, seed 1; plans stored as JSON strings). Re-publish with
`python3 tools/slm/publish_dataset_hf.py --repo larmkaixian/bt-sim-dataset --data-dir <dir>` (an existing repo keeps its visibility; new uploads of a public repo are public). The card is
neutral (no project name or links) and the data was grepped for paths/names before upload. Training used the first 6,000 train rows (2,835 under 768 tokens).
