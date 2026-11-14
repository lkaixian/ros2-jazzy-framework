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
  `llama-server -m models/llm/gemma-4-E2B-it-Q4_0.gguf --host 127.0.0.1 --port 8081 -c 4096 -ngl 99 --reasoning off`
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
