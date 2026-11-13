# ros2-jazzy-framework

ROS 2 Jazzy workspace: YOLO11 perception + an LLM -> SLM -> validator pipeline that produces
BehaviorTree.CPP v4 XML, with pluggable extension packages.

```
camera -> object_detection / person_detection / face_recognition (YOLO11)
                         |  (objects, persons, faces)
                 scene_summarizer  -> scene/summary
task  --bt/task--> llm_bridge (C++, llama.cpp server, e.g. Gemma 4 E4B) --bt/task_spec (JSON)-->
orchestrator -> bt_slm (fine-tuned ~230M SLM, e.g. LFM2.5-230M) -> bt_validator
        ^                                                              |
        +------------------- errors fed back, max_attempts ------------+
                         -> bt/plan (BtPlan, valid=true only if validated)
*_extension (arm, pose, odom, pid) --extensions/manifest--> validator / SLM / llm_bridge
```

| Package | Type | Role |
|---|---|---|
| `object_detection_msgr`, `person_detection_msgr`, `face_recognition_msgr` | ament_cmake | detection messages |
| `yolo11_common` | ament_python | shared image source (topic or webcam), model loading, OpenVINO export |
| `object_detection`, `person_detection`, `face_recognition` | ament_python | YOLO11 nodes (face: YOLO11 detector + YOLO11 embeddings, gallery in `faces/`) |
| `bt_pipeline_msgs` | ament_cmake | BtPlan, ExtensionManifest, LlmQuery/GenerateXml/ValidateTree |
| `llm_bridge` | **ament_cmake (C++)** | libcurl client for llama-server; task -> JSON task spec; `llm/query` service |
| `bt_common` | ament_python | node catalog (core + extensions), extension base class |
| `bt_slm` | ament_python | SLM XML generator (HF transformers) + rule-based fallback |
| `bt_validator` | ament_python | structural/port/cycle/limit checks on BT XML |
| `bt_orchestrator` | ament_python | generate -> validate -> retry loop; scene summarizer |
| `arm_extension`, `pose_extension`, `odom_extension`, `pid_extension` | ament_python | advertise BT nodes (interface only; executors not implemented yet) |
| `framework_bringup` | ament_cmake | `launch/`: `framework`, `perception`, `pipeline`, `extensions` |

## Build / run
Use the system Python first on PATH (a `~/pyvenv.cfg` can shadow it and break `empy`):
```
export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch framework_bringup framework.launch.py image_topic:=/camera/image_raw \
    start_llama_server:=true slm_model_path:=/path/to/finetuned-slm
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'fetch the cup'}"
ros2 topic echo /bt/plan
```

## Models
* LLM: Gemma 4 E4B via `llama-server -hf ggml-org/gemma-4-E4B-it-GGUF` (needs a llama.cpp build with Gemma 4 support).
* SLM: fine-tune `LiquidAI/LFM2.5-230M-Base` on (task_spec -> BT XML) pairs using the prompt format in `bt_slm/prompt.py`. Until `slm_model_path` is set, a deterministic rule-based generator is used. **Fine-tuning data/script are not included yet.**
* YOLO11 weights (`yolo11n.pt`, `yolo11n-pose.pt`) download on first use; face weights must be supplied (`yolov11n-face.pt`).

## Status
Verified: workspace builds; validator/SLM-fallback unit tests and C++ gtest pass; the pipeline ran end-to-end against a stub llama server (including a rejected-then-repaired attempt). Not verified: the YOLO nodes on a camera, a real llama-server, a real fine-tuned SLM.
