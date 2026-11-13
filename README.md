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
| `bt_executor` | **ament_cmake (C++)** | runs validated `bt/plan` with BehaviorTree.CPP; **`dry_run:=true` and `auto_run:=false` by default**; `bt_executor/run`, `bt_executor/halt`, `bt/status` |
| `odom_extension` | ament_python | NavigateTo, DriveDistance, RotateBy, IsAtGoal (odometry frame only) |
| `pid_extension` | ament_python | HoldHeading, FollowTarget (bbox-based, no depth), SetPidGains |
| `social_extension` | ament_python | `Speak` (TTS, `{person}` placeholder) and `EnrollFace` (face gallery) |
| `pose_extension` | ament_python | GetPersonPose, IsPersonWaving, IsPersonPointing from YOLO11-pose keypoints |
| `arm_extension` | ament_python | MoveArmToPose / OpenGripper / CloseGripper through MoveIt `MoveGroup` + `GripperCommand`; arm model is a parameter set |
| `bt_common` also ships | | `cmd_vel_mux_node`: priority + 0.5 s watchdog + hard clamps in front of the robot's `cmd_vel` |
| `audio_msgr` | ament_cmake | AudioChunk, Transcript, TtsStatus, Speak service |
| `audio_capture` | ament_python | microphone (`arecord`) or WAV file -> `audio/raw` |
| `stt_pipeline` | ament_python | energy VAD -> STT engine (faster-whisper / Vosk) -> `stt/transcript`; `transcript_to_task` (optional wake word) -> `bt/task`; mutes while the robot speaks |
| `tts_pipeline` | ament_python | queued TTS (espeak / Piper / Kokoro-82M), `tts/say`, `tts/speak`, `tts/stop`, `tts/status`; `plan_announcer` speaks `bt/plan` results |
| `framework_bringup` | ament_cmake | `launch/`: `framework`, `perception`, `pipeline`, `extensions`, `audio` |

## Quick start
```
./setup.sh --dry-run        # see what it would do; then ./setup.sh   (apt, rosdep, pip, llama.cpp, LLM + SLM base, build)
source env.sh               # system Python first, ROS, this workspace, ~/.local/bin
ros2 run framework_bringup check_params     # param files vs the real nodes
```
`setup.sh` steps: `apt rosdep python llamacpp models build` (+ optional `slm` = dataset and fine-tune). Use `--only`, `--skip`, `--minimal`, `--backend cuda|vulkan|cpu`. The LLM is fetched by `llama-server -hf` (Gemma 4 E4B), the SLM base `LiquidAI/LFM2.5-230M-Base` goes to `models/slm-base` for fine-tuning only.

## Parameters
Everything tunable lives in `src/framework_bringup/param/*.yaml` (`perception`, `pipeline`, `llama_server`, `extensions`, `audio`). Precedence: package defaults < these files < launch arguments (only arguments you actually pass override). Edit and relaunch; no rebuild needed. Another folder: `params_dir:=/path`. `check_params` starts every node with these files and reports typo'd keys and wrong types, which ROS would otherwise ignore silently. Per-package `launch/<pkg>.launch.py` files use the package's own `config/` defaults, not `param/`.

## Your own detector (bounding boxes only)
Plain YOLO11 boxes, no segmentation (a 4 GB GPU cannot afford SAM-class models). Verified on a GTX 1650: the nano detector runs ~10 ms/frame (~95 FPS) using ~100 MiB of VRAM, and a smoke fine-tune at batch 8 / 640 px peaked at **1.6 GB**.
```
ros2 run object_detection train_detector --data my_items.yaml --base yolo11n.pt --epochs 100   # defaults sized for 4 GB
# then in param/perception.yaml:  object_detection_node.model: <path to best.pt>
ros2 run bt_slm make_bt_dataset --labels @my_labels.txt --out-dir bt_dataset                   # SLM data with YOUR labels
```
The detector publishes its class names (`object_detection/classes`, latched). The validator then **rejects any `DetectObject label` the model cannot produce**, and the LLM prompt lists those labels plus every usable BT node with its description (`bt_validator/catalog`). Without a running detector set `bt_validator_node.labels` in `param/pipeline.yaml` or use `validate_bt --labels a,b,c`. Boxes: `ObjectArray` (`x_min,y_min,x_max,y_max` in pixels), annotated frames on `detections/image`.
Training needs matplotlib importable; if pip's NumPy 2 and apt's matplotlib clash, `train_detector` says so and `./setup.sh --only python` installs a compatible one.

## Hand-written behaviour trees
```
ros2 run bt_validator validate_bt --list-nodes          # every node + ports you may use
ros2 run bt_validator validate_bt my_tree.xml           # check offline, readable errors
ros2 run bt_orchestrator submit_bt my_tree.xml --run    # validate -> bt_executor (dry-run unless dry_run:=false)
```
Examples in `framework_bringup/behavior_trees/`: `greet_person.xml` (detect person -> `ExtractFaceFeature` -> `Speak "Hello {person}"`) and `enroll_person.xml` (`EnrollFace`). Port names `name` and `ID` are reserved by BehaviorTree.CPP and are rejected. Speak needs `tts_node` (`audio.launch.py enable_stt:=false`), EnrollFace/ExtractFaceFeature need `face_recognition_node` (publishes embeddings).

## Cluster (several machines)
`cluster_msgs` + `cluster_core`: one agent per machine, one scheduler, job catalog in `param/cluster_jobs.yaml`; places jobs by role/tags/capacity, fails movable jobs over, keeps hardware-bound jobs (`odom`, `pid`, `cmd_vel_mux`) on the `core` host, adopts state after a scheduler restart. Control plane only (data still flows directly over DDS). `ros2 launch framework_bringup cluster.launch.py role:=agent host_id:=pi roles:=core tags:=camera`, `ros2 run cluster_core cluster_status`, offline `plan_preview`. See `docs/cluster.md` (setup, failure semantics, security, measured numbers). Verified on one machine with simulated hosts only; not on a second machine or WiFi.

## Launch files
* `launch/` (workspace root): `framework`, `perception`, `pipeline`, `extensions`, `audio`, `cluster`; thin wrappers, e.g. `ros2 launch launch/framework.launch.py dry_run:=true`.
* `src/framework_bringup/launch/`: the real entry points the wrappers include (all arguments are documented there).
* Every runnable package has its own `launch/<package>.launch.py` to start it alone (`ros2 launch bt_validator bt_validator.launch.py`).

## Build / run
Use the system Python first on PATH (a `~/pyvenv.cfg` can shadow it and break `empy`):
```
export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch framework_bringup framework.launch.py image_topic:=/camera/image_raw \
    start_llama_server:=true slm_model_path:=/path/to/finetuned-slm
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'fetch the cup'}"
ros2 topic echo /bt/plan
# voice in/out (opens the microphone; needs an STT engine: pip install faster-whisper  or  vosk)
ros2 launch framework_bringup framework.launch.py enable_audio:=true wake_word:=robot
ros2 launch framework_bringup audio.launch.py tts_engine:=kokoro tts_voice:=<voice> \
    kokoro_model:=kokoro-v1.0.onnx kokoro_voices:=voices-v1.0.bin
```
Python edits need `colcon build` again: in this setup `--symlink-install` copies Python files into `install/`.

## Models
* LLM: Gemma 4 E4B via `llama-server -hf ggml-org/gemma-4-E4B-it-GGUF` (needs a llama.cpp build with Gemma 4 support).
* SLM: `LiquidAI/LFM2.5-230M-Base`, fine-tuned on synthetic pairs labelled by the validator:
  ```
  ros2 run bt_slm make_bt_dataset --out-dir bt_dataset --n 30000        # generate + repair rows
  pip install transformers
  ros2 run bt_slm finetune_bt_slm --data-dir bt_dataset --out bt_slm_model
  ros2 launch framework_bringup framework.launch.py slm_model_path:=$PWD/bt_slm_model
  ```
  Until `slm_model_path` is set, a deterministic rule-based generator (the dataset's ground truth) is used. The spec grammar is sequence/fallback groups with retries; richer structure (Parallel, SubTree) needs grammar and data extensions.
* YOLO11 weights (`yolo11n.pt`, `yolo11n-pose.pt`) download on first use; face weights must be supplied (`yolov11n-face.pt`).

## Executing plans (safety)
`framework.launch.py` starts `bt_executor` in **dry-run**: extension nodes are only logged and succeed, nothing moves. To command hardware you must pass `dry_run:=false`, and by default a plan still waits for `ros2 service call /bt_executor/run std_srvs/srv/Trigger` (`auto_run:=true` runs each valid plan immediately). `ros2 service call /bt_executor/halt ...` stops the tree and cancels the running extension goal.
All motion goes through `cmd_vel_mux_node` (clamps `max_linear`/`max_angular`; zero Twist if no input for `timeout_sec`=0.5 s, so a crashed extension coasts at most ~0.5 s at its last command). The mux is a software watchdog, **not** an e-stop: keep a hardware stop within reach.
Arm: pass `arm_group:=... arm_ee_link:=... arm_base_frame:=... gripper_action:=...` once the arm is chosen; with those empty `arm_extension` refuses to move. Velocity scale is capped at 0.1 by default.

## Audio engines
STT: `faster_whisper` or `vosk` (neither is installed by default; the node refuses to start without it). `mock` only tests the plumbing.
TTS: `espeak` works out of the box; `piper` needs `pip install piper-tts` plus a voice `.onnx`; `kokoro` needs `pip install kokoro-onnx` plus the model and voices files.

## Status
Verified: workspace builds; validator/SLM-fallback unit tests and C++ gtest pass; the pipeline ran end-to-end against a stub llama server (including a rejected-then-repaired attempt). Audio plumbing was verified with a synthesized WAV and mock engines (VAD -> transcript -> `bt/task`; TTS status).
Execution was verified in a **simulated** robot (a node integrating `cmd_vel` into `odom`): task -> stub llama -> SLM fallback -> validator -> `bt_executor` -> `odom_extension` -> mux -> robot, covering a live drive+turn (0.29 m / 0.47 rad requested 0.30 / 0.50), dry-run (no motion), halt mid-drive, and SIGKILL of the extension (mux stops the robot within its timeout). Controllers and PID have closed-loop unit tests; gesture rules are tested on synthetic skeletons only.
The greeting/enrolment trees were run through the real chain against a **fake perception node** and the mock TTS engine (spoke `Hello alice` / `Hello there`, enrolled via a stub service); the real YOLO face node, face embeddings and neural TTS were not exercised. Not verified: `setup.sh` apt/pip/llama.cpp/model steps (dry-run only; `--only build` was run), any real robot, MoveIt/arm execution (only goal construction is tested), gesture accuracy on real video, SLM fine-tuning (dataset generator is tested, `finetune_bt_slm` has only unit tests of its masking/collation and has never been run), the YOLO nodes on a camera, a real llama-server, a real fine-tuned SLM, real STT recognition (faster-whisper/Vosk), Piper and Kokoro playback, live microphone capture.
