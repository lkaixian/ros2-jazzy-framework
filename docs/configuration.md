# Configuration, launch and build

## Parameters
Everything tunable lives in `src/framework_bringup/param/*.yaml` (`perception`, `pipeline`, `llama_server`, `extensions`, `audio`). Precedence: package defaults < these files < launch arguments (only arguments you actually pass override). Edit and relaunch; no rebuild needed. Another folder: `params_dir:=/path`. `check_params` starts every node with these files and reports typo'd keys and wrong types, which ROS would otherwise ignore silently. Per-package `launch/<pkg>.launch.py` files use the package's own `config/` defaults, not `param/`.

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
