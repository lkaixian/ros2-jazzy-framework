# Concepts and glossary

## The five ideas

1. **Behaviour tree (BT).** A tree of small steps. `Sequence` runs children in order and stops at the first failure; `Fallback` tries children until one succeeds; leaves are actions ("drive 1 m") and conditions ("is a person visible?"). The framework uses [BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML.
2. **Task spec.** A short JSON description of what to do (`goal` plus `steps`). It is what the LLM writes. It is far shorter than XML, so it is quick to generate and easy to constrain.
3. **Macro.** A named, parameterised list of steps (`go_to_place`, `patrol`, `meet_guest`). The LLM may only call whitelisted macros; the orchestrator expands them to real nodes.
4. **Validator.** The deterministic gatekeeper. A plan that does not pass is never executed; its errors go back to the generator.
5. **Extension.** A separate ROS 2 process that provides BT nodes (navigation, arm, speech ...) through a YAML manifest and an action server. Adding a skill never requires touching the executor.

## Glossary

| Term | Meaning |
|---|---|
| **LLM** | The large model that understands the request: Gemma 4 (E4B on a strong PC, E2B on the robot laptop) served by `llama-server`. |
| **SLM** | The small model (LFM2.5-230M, fine-tuned) that renders a spec to XML. Used on retries; rules go first. |
| **rules_first** | `bt_slm` tries the exact rule generator before the SLM (0.06 ms vs about 1.5 s). |
| **Catalog** | Every usable BT node with ports and description, served by `bt_validator/catalog`. |
| **Manifest** | An extension's YAML list of its nodes; republished every 3 s on `extensions/manifest`. |
| **Blackboard** | BehaviorTree.CPP's shared key-value store; `{name}` in a port reads it, `out_*` ports write it. |
| **Dry-run** | `bt_executor` default: extension nodes are only logged and succeed; nothing moves. |
| **auto_run** | If true, each valid plan runs immediately. Default false: you call `bt_executor/run`. |
| **Mux** | `cmd_vel_mux_node`: single gate in front of the robot's velocity input. |
| **Latch** | The mux's stop state after a bumper/cliff event; cleared only by `safety_reset`. |
| **Robot mode** | `init`, `ready`, `active`, `teleop`, `safe_stop`, `init_failed`, published on `robot/mode`. |
| **Mode gate** | Executor option that refuses to run unless `robot/mode` is fresh and `run_ok`. |
| **Recovery** | What happens after a failed run: retry, replan, go home, ask a human, abort. |
| **Repair** | A replan through the LLM that includes the failing node (`bt/repair`). |
| **Extension executor** | The base class (`bt_common.executor_node.ExtensionExecutor`) that turns handlers into an `ExecuteNode` action server. |
| **Place** | A named pose in the map frame (`param/places.yaml`), used by `NavigateToPlace`. |
| **GPSR** | RoboCup@Home task family: general-purpose service robot commands. Our eval commands are GPSR-style, not the official sentences. |
| **HRIC / PnPC** | RoboCup@Home tasks (human-robot interaction and cooperation; pick and place challenge). Rehearsal trees exist; arm parts are untested. |
| **Workspace** | An SROS2 keystore, a DDS domain, an LLM API key and a join token, created with `ws create`. |
| **Groot2** | The BehaviorTree.CPP GUI; the executor can serve the running tree to it (port 1667). |
| **Dashboard (`dash`)** | The terminal UI: nodes, LLM link, battery, task state, live plan tree. |

## Naming conventions

* Topics use `/` namespaces by area: `bt/*` planning and execution, `task/*`, `robot/*`, `safety/*`, `llm/*`, `stt/*`, `tts/*`, `audio/*`, `cluster/*`.
* `out_<name>` ports are outputs. A trailing `?` in the catalog means optional.
* Port values are **strings** end to end; extensions parse and range-check them.
* Frames are the Kobuki's: `odom`, `base` (not `base_link`), lidar frame `laser`.

Next: [Tutorial: first plan](tutorial-first-plan.md).
