# Gotchas (each one cost real time)

## Editing and process discipline
- **Silent no-op edits.** A Python `str.replace` or a pattern edit that matches nothing changes
  nothing and reports success. This happened several times (the `enable_audio` hook in
  `framework.launch.py`, the SLM `normalize_spec` use, the first manifest republish) and was only
  caught later by audit/integration tests. Use `tools/strict_sub.py` (`from strict_sub import sub`),
  which raises when the match count is not exactly what you expect, and re-grep after editing.
  Files written through the generator helpers were dedented, so indentation guesses are often
  wrong (YAML 2 vs 4 spaces, module level vs class level): read the real text first.
- A launch file that *loads* (`--show-args`) proves nothing about behaviour. Start the nodes.
- **Kill test processes in a separate command, and never with a pattern that appears in your own
  command line** (`pkill -f fake_llama` inside a command that mentions `fake_llama` kills your own
  shell, exit 144). Use bracket patterns (`[f]ake_llama`) and run cleanup as its own call.
  `tools/integration/*.sh` have an exit trap that stops what they started.
- Tests must use an **isolated DDS domain** (`ROS_DOMAIN_ID=4x` + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)
  so they never talk to a real robot's graph; `check_params` uses 97.
- `git add -A` swept agent worktrees (`.claude/worktrees/*`) into a commit once. `.claude/` is ignored
  now; still check `git status` before committing.

## Python / environment
- `~/pyvenv.cfg` makes the `~/bin/python3` symlink behave like a venv that hides system packages
  (`empy`, numpy, ultralytics): CMake fails with `No module named 'em'`. Put `/usr/bin` first on PATH.
  After a wrong-Python configure, delete the package's `build/` and `install/` dirs (stale CMake cache).
- `ament_python` copies `.py` files into `install/` even with `--symlink-install`: **rebuild after
  Python edits** (ament_cmake data files such as `param/`, `launch/` are symlinked).
- pip's NumPy 2.4 in `~/.local` shadows apt NumPy 1.26. apt matplotlib (built for NumPy 1) then fails
  to import, which breaks Ultralytics *training* (detection still works). Fix: install matplotlib>=3.9
  with pip (`./setup.sh --only python`) or pin NumPy < 2. `train_detector` prints this. Nothing was
  changed in the user's Python by us.
- `rosdep check --from-paths src --ignore-src` currently reports all dependencies satisfied.

## BehaviorTree.CPP
- Port names `name` and `ID` are reserved and illegal; names must start with a letter. The catalog
  rejects them at load. (Found when `Speak name=...` failed only at execution time.)
- The CMake target is `behaviortree_cpp::behaviortree_cpp` in the Jazzy package (not `BT::...`).
- A condition returning RUNNING is only allowed for ActionNode-derived nodes; extension nodes are
  registered as actions.
- `RetryUntilSuccessful` retries immediately: wrap polling conditions as
  `Fallback(Sequence(...), ForceFailure(Sleep msec=200))` (see the example trees).

## ROS 2 / DDS
- An empty array in a ROS param YAML is unreliable (type inference): roles/tags/etc. are
  comma-separated strings.
- Unknown keys in a param file are ignored silently: run `check_params` after editing.
- Latched (transient_local) publication alone lost the race with discovery when many processes start
  together: extension manifests are re-published every 3 s and consumers register idempotently.
- When one node subclass creates `self.pub`, it overwrote the base class's manifest publisher
  (crash at the first republish): base attributes are underscore-prefixed (`_manifest_pub`).
- `ros2 launch` children can outlive a killed parent; check `ps` and the process's
  `ROS_DOMAIN_ID` (`/proc/<pid>/environ`) before killing so you never touch another session.
- Nav2 Jazzy chain: controller -> `cmd_vel_nav` -> velocity_smoother -> `cmd_vel_smoothed` ->
  collision_monitor -> `cmd_vel`; here the final output is remapped to `nav2/cmd_vel` for the mux.
  Kobuki frames: `odom`, `base` (not `base_link`); its input is `commands/velocity` with a 0.6 s timeout.

## Models and external tools
- Gemma 4 needs llama.cpp build >= b8637; repos may be gated (accept the licence, `hf auth login`).
- `llama-server -hf <repo>` downloads into llama.cpp's own cache (not the Hugging Face cache);
  `setup.sh models` uses that on purpose.
- The detector's labels come from the model (`model.names`); never hard-code COCO labels in BTs,
  datasets or prompts (the dataset generator takes `--labels`).
- COCO `yolov8n.pt` sits untracked in the repo root (weights are git-ignored).
- espeak `-q -w file` writes nothing; use `-w file` without `-q`.

## Process / user preferences
- No chat/session link in commit messages (user's explicit instruction).
- Mid-task user messages are refinements; acknowledge, fold in, and keep the verified/unverified
  split honest. Earlier summaries were wrong twice (audio include, republish); the correction
  was stated plainly. Keep doing that.

## Added 2026-11-14
- **JSON-schema key order:** llama.cpp emits properties in schema order; `nlohmann::json` sorts keys alphabetically (args before macro = the model
  picks macros without args). Use `ordered_json` for anything sent as a schema. Test: `LlamaClient.SchemaKeepsPropertyOrderAndRepeatPenalty`.
- **Gemma 4 thinking:** run `llama-server --reasoning off`, or replies are cut off by `max_tokens` ("not a JSON object").
- **Static library in a shared plugin needs `-fPIC`:** `dstar_lite_core` without it only worked unoptimised (Ubuntu defaults to PIE). Set
  `POSITION_INDEPENDENT_CODE ON`. The symptom was `planner_server` exit code -11 on the first goal.
- **Never install an experimental build into the shared `install/`:** it breaks everyone using the workspace. Overlay: `colcon build
  --build-base B --install-base I --packages-select X`, then `source I/setup.bash` after `install/setup.bash`.
- **Background jobs ignore SIGINT:** `cmd &` from a non-interactive shell cannot be stopped with Ctrl-C/`kill -INT`; use SIGTERM, and select
  processes by `ROS_DOMAIN_ID` in `/proc/<pid>/environ` (never `pkill -f <pattern>`: it can match your own shell).
- **Core dumps:** `core_pattern` pipes to apport; a crash lands in `/var/crash/*.crash` (one per executable until removed): `apport-unpack` it, then
  `gdb -batch -ex bt <exe> crashdir/CoreDump`.
- **ROS names:** a service/topic token must not start with a digit (a container id as host id broke `host_info/<id>`); sanitise with `host_service_name`.
- **Restricted SROS2 roles still need parameter-service replies**, or no rclpy node starts.
- **A denied DDS writer raises at `create_publisher`**, not silently later.
- **Extensions must be up before tasks are sent:** if the validator misses a manifest, the retry path drops unknown steps and can accept a shorter plan.
- **Latched `bt/tree`:** `dash` started after a run still shows the last tree; statuses are sticky (S/F stay) by design.
- **Python edits need `colcon build`** (symlink-install copies Python files here), including for `bt_common`; a stale install shows "cannot import name".
- **`python3` on PATH may be a different interpreter:** use `/usr/bin/python3` for scripts that import `yaml`/rclpy.
- **Disk is tight (about 8 GB free on `/`):** models live in `models/` (gitignored); do not add the E4B model here.
