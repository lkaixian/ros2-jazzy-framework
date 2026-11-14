# Topics, services and actions

Names are relative to the workspace namespace (none by default). Message fields are in [Messages](messages.md). "Latched" means transient-local QoS.

## Topics

| Topic | Type | Publisher | Subscribers | Notes |
|---|---|---|---|---|
| `bt/task` | `std_msgs/String` | `transcript_to_task`, operator | `llm_bridge`, `orchestrator` | plain-text request |
| `scene/summary` | `String` | `scene_summarizer` | `llm_bridge` | people, distance, gestures, known faces |
| `bt/task_spec` | `String` (JSON) | `llm_bridge`, orchestrator (recovery) | `orchestrator` | goal + steps; may call macros |
| `bt/plan` | `BtPlan` | `orchestrator` | `bt_executor`, `plan_announcer` | `valid=true` only if validated |
| `bt/status` | `String` (JSON) | `bt_executor` | `orchestrator` | `result`, `failed_node`; published once per run |
| `bt/tree` | `String` (JSON, latched) | `bt_executor` | `dash`, Groot2 feed | uid, depth, parent, type, name, ports |
| `bt/node_status` | `String` (JSON) | `bt_executor` | `dash` | `{"s": {uid: R/S/F/K/I}}`, sticky S/F, K = halted while running |
| `bt/repair` | `String` | `orchestrator` | `llm_bridge` | failed node and reason for a replan |
| `task/state` | `String` (JSON, latched) | `orchestrator` | `dash`, `robot_mode_node` | lifecycle, repairs used |
| `robot/mode` | `String` (JSON, latched) | `robot_mode_node` | `bt_executor` (mode gate), `dash` | mode, `run_ok`, reasons, missing, mission_left_sec |
| `extensions/manifest` | `ExtensionManifest` (latched, republished every 3 s) | every extension | validator, `bt_slm`, executor | consumers register idempotently |
| `llm/status` | `String` | `llm_bridge` | `robot_mode_node`, `dash` | `up` / `down` per endpoint |
| `llm/error` | `String` | `llm_bridge` | `orchestrator` | turns silence into an invalid plan with a reason |
| `objects`, `persons`, `faces` | `ObjectArray`, `PersonArray`, `FaceArray` | YOLO11 nodes | executor conditions, `scene_summarizer` | boxes in pixels, top-left origin |
| `object_detection/classes` | `String` (latched) | `object_detection` | validator | the detector's label vocabulary |
| `detections/image` | `sensor_msgs/Image` | detectors | dash/rviz | annotated frames |
| `audio/raw` | `AudioChunk` | `audio_capture` | `stt_pipeline` | s16le PCM |
| `stt/utterance`, `stt/transcript` | `AudioChunk`, `Transcript` | `stt_pipeline` | `transcript_to_task`, `sound_event_node` | |
| `stt/capture` | `std_msgs/Bool` (latched) | `WaitForTranscript` | `transcript_to_task` | while true, speech is an answer, not a task |
| `audio/events` | `String` | `sound_event_node` | `DetectSound` | `{"kind": "doorbell"}` |
| `tts/say`, `tts/status` | `String`, `TtsStatus` | `plan_announcer`, `tts_node` | `stt_pipeline` mutes while SPEAKING | |
| `safety/stop_state` | `String` | `cmd_vel_mux_node` | `robot_mode_node` | `armed` or `latched: <reason>` |
| `safety/battery` | `String` | `battery_guard_node` | `robot_mode_node` | `{"level": "critical"}` below 13.2 V for 3 s |
| `teleop/cmd_vel` | `geometry_msgs/Twist` | `teleop` | mux, `robot_mode_node` | highest mux priority |
| `cluster/node_info`, `cluster/assignments`, `cluster/status` | `cluster_msgs` | agents, scheduler | `llm_bridge`, `dash` | ~2 Hz heartbeat incl. LLM endpoints and battery |

## Services

| Service | Type | Server | Purpose |
|---|---|---|---|
| `bt_validator/validate` | `ValidateTree` | validator | XML -> valid + errors |
| `bt_validator/catalog` | `GetCatalog` | validator | node listing, labels, JSON schema |
| `bt_slm/generate` | `GenerateXml` | `bt_slm` | spec (+ previous xml and errors) -> xml |
| `bt_executor/run` | `std_srvs/Trigger` | executor | start the current valid plan (refused by the mode gate when not `run_ok`) |
| `bt_executor/halt` | `Trigger` | executor | stop the tree and cancel the running extension goal |
| `robot/reset` | `Trigger` | `robot_mode_node` | a human acknowledges `init_failed` / `safe_stop`; refused while the cause persists |
| `safety_reset` | `Trigger` | mux | clear the bumper/cliff latch (refused while a sensor is still hit) |
| `llm/query` | `LlmQuery` | `llm_bridge` | raw LLM call |
| `tts/speak`, `tts/stop` | `Speak`, `Trigger` | `tts_node` | |
| `face/enroll` | `Enroll` | `face_recognition` | store a face under a name |
| `host_info/<host_id>` | `HostInfo` | each cluster agent | CPU, RAM, GPU, VRAM, temperature, battery |

## Actions

`<extension>/execute` (`bt_pipeline_msgs/action/ExecuteNode`): goal = `node_id` + parallel arrays `port_names` / `port_values` (strings); result = `success`, `message`, `output_names` / `output_values` (the executor writes them to the blackboard when the XML names that `out_` port); feedback = `status`. One goal at a time per extension, which is why the validator rejects a `Parallel` that starts two nodes of one extension.

## JSON payloads

```text
task/state    {"state": "running", "repairs": 1, "max_repairs": 3, "task": "go to the door", "detail": ""}
robot/mode    {"mode": "ready", "run_ok": true, "reasons": [], "missing": [], "task_state": "idle", "mission_left_sec": null}
bt/status     {"result": "FAILURE", "failed_node": "NavigateToPlace", ...}
```
