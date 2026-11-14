# Tutorial: your first plan

Goal: build the workspace, start the pipeline in **dry-run**, feed it a task spec by hand (so no LLM is needed), watch it become a validated tree and "run". Takes about 15 minutes after the build. Nothing moves.

> [!NOTE]
> The executor starts in dry-run: extension nodes are only logged and succeed. Nothing can move, whatever you send.

## 0. Prerequisites

* Ubuntu 24.04 with ROS 2 Jazzy (or Ubuntu 22.04 with Humble; see [Quick start](https://github.com/lkaixian/ros2-jazzy-framework#quick-start)). Python from `/usr/bin` must come first on `PATH` (a `~/pyvenv.cfg` can hide system packages).
* About 8 GB free disk for the build; models are optional here.

## 1. Build

```bash
git clone https://github.com/lkaixian/ros2-jazzy-framework.git && cd ros2-jazzy-framework
./setup.sh --dry-run          # shows apt / rosdep / pip / build steps without doing them
./setup.sh --minimal          # or: ./setup.sh   (adds llama.cpp, models)
source env.sh                 # system Python first, ROS, this workspace
ros2 run framework_bringup check_params     # every node starts with the param files; prints typos
```

Python edits need `colcon build` again: this setup copies Python files into `install/` even with `--symlink-install`.

## 2. Start the planning pipeline

Use a private DDS domain so you cannot disturb a real robot on the same network:

```bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch launch/framework.launch.py dry_run:=true start_llama_server:=false
```

You get the validator, the SLM node (rule generator only, no model), the orchestrator, the executor in dry-run, and the extensions. In a second terminal (same exports) check they are alive:

```bash
ros2 run framework_bringup dash --once
ros2 service call /bt_validator/catalog bt_pipeline_msgs/srv/GetCatalog   # the nodes the validator knows
```

## 3. Send a task spec (skipping the LLM)

The LLM normally writes this JSON onto `bt/task_spec`. You can write it yourself:

```bash
ros2 topic echo /task/state &                     # watch the lifecycle
ros2 topic echo /bt/plan --once &
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'go to the door and say hello'}"
ros2 topic pub --once /bt/task_spec std_msgs/msg/String \
  "{data: '{\"goal\": \"go to the door and say hello\", \"steps\": [{\"macro\": \"go_and_say\", \"args\": {\"place\": \"door\", \"text\": \"hello\"}}]}'}"
```

What happens, in order:

1. `task/state` goes `planning`.
2. The orchestrator expands the `go_and_say` macro into `NavigateToPlace` + `Speak` and asks `bt_slm` for XML.
3. `bt_validator` checks it. `bt/plan` arrives with `valid: true`, the XML and `attempts: 1`.
4. `task/state` becomes `ready`: the plan waits because `auto_run` is false.

## 4. Run it

```bash
ros2 service call /bt_executor/run std_srvs/srv/Trigger
```

In dry-run the executor logs `start nav_extension/NavigateToPlace(place=door)` and `start social_extension/Speak(text=hello)` and finishes with SUCCESS: `task/state` -> `done`. Nothing moved.

## 5. Make it fail on purpose

Send a spec the validator must refuse:

```bash
ros2 topic pub --once /bt/task_spec std_msgs/msg/String \
  "{data: '{\"goal\": \"bad\", \"steps\": [{\"node\": \"Fly\", \"ports\": {}}]}'}"
```

`bt/plan` comes back `valid: false` with an error naming the unknown node, after `max_attempts` (3). That message is exactly what the LLM would receive on a retry. Read [Validator rules](validator-rules.md) to see every check.

## 6. Hand-written XML

```bash
ros2 run bt_validator validate_bt --list-nodes
ros2 run bt_validator validate_bt src/framework_bringup/behavior_trees/greet_person.xml
ros2 run bt_orchestrator submit_bt src/framework_bringup/behavior_trees/greet_person.xml --run
```

## 7. Add the real LLM (optional)

```bash
llama-server -m models/llm/gemma-4-E2B-it-Q4_0.gguf --host 127.0.0.1 --port 8081 -c 6144 -np 1 -ngl 99 --reasoning off
ros2 launch framework_bringup pipeline.launch.py llama_url:=http://127.0.0.1:8081
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'go to the door and say hello'}"
```

`-c 6144 -np 1` matters: the planning prompt is 3.2-3.7k tokens, and four slots sharing 4096 truncate it. `--reasoning off` stops Gemma 4 from spending the whole token budget thinking.

## 8. Where next

* Read what the planner can say: [Task spec grammar](task-spec-grammar.md), [Macros](macros.md).
* Watch a run live: [Dashboard](dashboard.md), [Groot2](operator-guide.md#groot2-the-behaviortreecpp-gui).
* Before any real hardware: [Safety](safety.md) and [Robot mode and recovery](robot-mode-and-recovery.md).
* Something broke: [Troubleshooting](troubleshooting.md).
