# Architecture

## The idea: a harness around a slow brain and a fast brain
```
operator voice/text ──> bt/task (String)
                          │
   perception ──> scene/summary ─┐
   (YOLO11 boxes)                ▼
                     llm_bridge (C++)  ── HTTP ──> llama-server (LLM, "big brain", e.g. Gemma 4 E4B)
                          │  asks bt_validator/catalog for: all usable BT nodes + descriptions + detector labels
                          ▼
                   bt/task_spec (JSON: goal, steps, nested Sequence/Fallback groups, retry)
                          ▼
                   bt_orchestrator ──> bt_slm (fine-tuned ~230M SLM, "fast brain"; rule-based fallback now)
                          │                 ▲
                          │   errors fed back, up to max_attempts
                          ▼                 │
                   bt_validator (static checks) ─┘
                          ▼
                   bt/plan (BtPlan, valid=true only if validated)
                          ▼
                   bt_executor (C++, BehaviorTree.CPP)  ── dry_run=true, auto_run=false by default
                          │ goals to <extension>/execute (ExecuteNode action)
        ┌─────────┬───────┴───────┬────────────┬─────────────┐
   odom_ext   pid_ext        social_ext     nav_ext       arm_ext / pose_ext
        │         │                │             │
        └────> cmd_vel_mux ──> robot (commands/velocity)      Nav2 ──> cmd_vel_nav ... ──> nav2/cmd_vel (lowest priority)
```
The "harness" = validator (nothing unvalidated runs), dry-run/manual-run defaults, mux clamps and
watchdog, halt service, extension executors that reject bad ports, cluster fencing.
Hand-written BTs enter at `submit_bt` and take the same validator -> executor path.

Runtime failure of a BT node can feed back to the LLM (`max_repairs`, off by default); an LLM that gives no usable reply produces an invalid plan with the
reason (`llm/error` -> orchestrator) instead of silence.

## Key interfaces
- Topics: `bt/task`, `scene/summary`, `bt/task_spec`, `bt/plan`, `bt/status` (JSON string),
  `extensions/manifest` (latched + republished every 3 s), `objects`/`persons`/`faces`,
  `object_detection/classes` (latched), `detections/image`, `audio/raw`, `stt/utterance`,
  `stt/transcript`, `tts/say`, `tts/status`, `cluster/node_info|assignments|status`.
- Services: `bt_validator/validate`, `bt_validator/catalog`, `bt_slm/generate`, `bt_executor/run|halt`,
  `tts/speak|stop`, `face/enroll`, `llm/query`.
- Action: `<extension>/execute` (`bt_pipeline_msgs/action/ExecuteNode`: node_id + port names/values as
  strings; result success+message).
- BT nodes: core/built-in perception conditions live in C++ (`bt_executor/src/registry.cpp`):
  `DetectObject`, `DetectPerson`, `RecognizeFace`, `ExtractFaceFeature`. Everything else is an
  **extension node** declared in `<pkg>/config/<pkg>.yaml` and implemented by that extension.
  Catalog of core nodes: `bt_common/config/core_nodes.yaml`.
- Dashboard feed (`bt_executor`): `bt/tree` (latched JSON: uid, depth, parent, type, name, ports per node) at plan start and `bt/node_status`
  (`{"s": {uid: R|S|F|K|I}}` on change, sticky: S/F stay after a Sequence resets its children; K = halted while running).
- Voice: `stt/capture` (latched Bool; `WaitForTranscript` sets it so `transcript_to_task` does not turn the answer into a task), `audio/events`
  (`sound_event_node` -> `{"kind": "doorbell"}`), `llm/error`, `safety/battery`.
- Node outputs: an extension handler may return `(ok, msg, {"out_x": v})`; `ExecuteNode` results carry `output_names/values`; `bt_executor` writes them to
  the blackboard when the XML names that `out_` port.
- Machine health: each `cluster_agent` serves `host_info/<host_id>` (`cluster_msgs/srv/HostInfo`, sanitised name); `cluster/node_info` carries battery too.
- Velocity chain: teleop/odom_ext/pid_ext/nav2 publish to `*/cmd_vel` -> `cmd_vel_mux_node` (priority order
  teleop > odom > pid > nav2, clamp, 0.5 s watchdog, one zero Twist when all stale) -> `cmd_vel_topic`
  (`commands/velocity` on the Kobuki, which itself stops after 0.6 s).

## Package map (28)
- Messages: `object_detection_msgr`, `person_detection_msgr`, `face_recognition_msgr`, `audio_msgr`,
  `bt_pipeline_msgs`, `cluster_msgs`.
- Perception: `yolo11_common` (image source, model load), `object_detection`, `person_detection`,
  `face_recognition`.
- LLM/SLM/BT: `llm_bridge` (C++), `bt_slm`, `bt_validator`, `bt_orchestrator`, `bt_executor` (C++),
  `bt_common` (catalog, extension base classes, mux, launch utils).
- Extensions: `odom_extension`, `pid_extension`, `pose_extension`, `social_extension`, `arm_extension`,
  `nav_extension`.
- Audio: `audio_capture`, `stt_pipeline`, `tts_pipeline`.
- Navigation: `nav_sim` + launch/params/maps in `framework_bringup`.
- Distribution: `cluster_core`.
- Bring-up: `framework_bringup` (launch, param, behavior_trees, maps, scripts) and root `launch/`.

## Decisions and why
- **Validator is the single source of truth** for what planners may use (nodes, ports, labels). The
  SLM and LLM prompts and the executor registration derive from it / the same manifests.
- **Extension manifests** (YAML -> `extensions/manifest`) make capabilities pluggable; the base class
  `bt_common.extension_node` re-publishes them so DDS discovery races cannot lose a registration.
- **BT port names**: `name` and `ID` are reserved by BehaviorTree.CPP; the catalog rejects them.
- **Port values are strings** end to end; BT blackboard references `{x}` work (e.g.
  `ExtractFaceFeature out_identity="{who}"` then `Speak person="{who}"`). Extension nodes have no
  output ports (limitation).
- **SLM prompt format** is fixed (`bt_slm/prompt.py`, `prompt_for`) and guarded by a test that rebuilds
  training prompts the way inference does. Changing it means regenerating data and retraining.
- **Plain bounding boxes**, no segmentation (4 GB VRAM). The detector owns the label vocabulary.
- **Defaults are conservative**: dry-run, no auto-run, 0.2 m/s odom speeds, mux clamp 0.3 m/s, arm
  velocity scale 0.1, extensions refuse to move without configuration (arm).
- **One param folder** (`framework_bringup/param/`): package defaults < param files < launch args
  that are actually passed (unset args use a sentinel so they never clobber the files).
  `check_params` is the guard against silently ignored keys.
- **Nav stack choice**: slam_toolbox (2D, default), Cartographer (optional), RTAB-Map (optional,
  RGB-D), Nav2 with NavFn + Regulated Pure Pursuit (lighter than the stock MPPI for a Pi/Jetson).
  Frames are the Kobuki's: `odom -> base` (**`base`, not `base_link`**), lidar frame `laser`.
- **Cluster = control plane only**: scheduler publishes desired state, agents reconcile; agents run
  only jobs from their *local* catalog (assignments carry ids, never commands); hardware-bound jobs
  are pinned to the `core` role and never fail over; fencing by timeout (failover 5 s > agent orphan
  timeout 4 s).
- **Parallel agents**: big independent work was done in git worktrees on separate DDS domains
  (41 nav, 42 cluster) and merged; tests use isolated domains and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

## Added 2026-11-14 (decisions and why)
- **Macros + grammar, not bigger prompts:** the LLM calls whitelisted macros (`param/macros.yaml`), loops (`repeat`, `foreach`), `Parallel`; everything is
  expanded in plain code before the SLM/validator, so the SLM prompt format never changed. `docs/task-spec-grammar.md`.
- **Constrained decoding:** the validator builds a JSON schema (`GetCatalog.schema`) from the catalog + whitelist; `llm_bridge` sends it as
  `response_format`; property order is part of the contract (`ordered_json`). Source of truth stays the catalog.
- **Safety stays outside the planner:** guards live in the nodes that move the robot (`ScanGuard` in `odom_extension`/`pid_extension`, `battery_guard_node`,
  the mux latch), not in the LLM prompt; the executor stays dry-run by default.
- **Workspace roles** are enforced by DDS Security permissions in each member's enclave; revocation = re-key (new CA).
- **Experiments never go into the shared `install/`:** use `colcon --build-base X --install-base Y` and source `Y/setup.bash` on top.
