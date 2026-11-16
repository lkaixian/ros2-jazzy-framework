# World In-Context Learning (task memory)

> [!WARNING]
> Sim-verified at most: a **2D kinematic** nav sim (no Nav2, no manipulation physics, no hardware). Off by default.

Inspired by Fang et al., *World In-Context Learning for Agentic Robot Manipulation* (arXiv:2611.12885). The model never changes: the planner tries a task in a simulator, learns from the feedback **in its context**, and a successful rehearsal is saved as a **Task Memory** (a JSON file). No fine-tuning, no gradient updates. Differences from the paper are in [notes/wicl_paper_notes.md](../notes/wicl_paper_notes.md).

```
rehearse (bt_rehearsal)                    deploy (bt_orchestrator, memory_enabled:=true)
task -> LLM plan -> validator -> sim       task -> look up memories (task text + scene objects)
   fail: reflect, feed lesson back, retry     -> inject top-k as examples for the LLM
   success: freeze as Task Memory   ------>   -> if a memory's XML validates NOW, use it as the plan
```

## Pieces

| Package | Role |
|---|---|
| `bt_memory` | `TaskMemory` (versioned JSON, one file per memory), store, lint, `TextSceneRetriever` (swap-in point for embeddings), cross-embodiment adapter |
| `bt_rehearsal` | `Simulator` protocol, `KinematicNavSim`, rehearsal loop, reflection, `wicl` CLI, `rehearsal_node` |
| `bt_orchestrator` | `memory_hook.py`: retrieval, behind `memory_enabled` (default **false**) |

## Run it

```bash
source env.sh
# rehearse against a llama-server endpoint (no endpoint = offline: reports planner_unavailable, plans nothing)
ros2 run bt_rehearsal wicl rehearse "go to the right room" --goal-place right_room --endpoint http://127.0.0.1:8080
ros2 run bt_rehearsal wicl list
ros2 run bt_rehearsal wicl show <id>
ros2 run bt_rehearsal wicl adapt <id> capabilities.yaml   # new memory for another robot, or a loud error
# deployment: retrieval on
ros2 launch framework_bringup pipeline.launch.py wicl_memory:=true
# ROS services: wicl/rehearse (RehearseTask), wicl/memories (ListMemories); node: ros2 run bt_rehearsal rehearsal_node
```
Memories live in `~/.local/share/robocup_fw/bt_memory` (override with `memory_dir`). Parameters are in `param/pipeline.yaml` (`memory_*`).

## Guarantees and their limits

- **A memory is a proposal.** The orchestrator re-validates its XML against the current validator before using it; a stale one is only shown as an example. `bt_executor` is untouched (dry-run default, validator-approved plans only). The velocity mux, watchdog and guards are not touched.
- **Data isolation.** The simulator returns `Observable` (scene summary, validator errors, executor-style result with failed node) and `Privileged` (pose, extras). `Privileged` has no text form; `Observable` accepts only allow-listed keys; the loop calls the simulator's `score()` for the pass/fail bit and passes nothing else on. A test plants a sentinel in the ground truth and asserts it appears in no prompt and no stored memory. Memories are also linted (no coordinates, no "ground truth").
- **Offline.** Rule generator and SLM are unchanged. With no LLM endpoint, rehearsal degrades (`planner_unavailable`), reflection falls back to a template lesson, and retrieval of existing memories works with no network. 4 GB VRAM: nothing here loads a model; the LLM is whatever endpoint you point at.
- **Not covered.** The kinematic sim models Sequence/Fallback/Retry/Repeat/Sleep/Speak and navigation (grid reachability) only. Any other node fails with "unsupported node", so a tree using perception or speech cannot be rehearsed yet. The ROS-driven simulator backend (real `bt_executor` + `nav_sim` + Nav2) is **not implemented**; the `Simulator` protocol is the seam for it.

## Cross-embodiment capability file

```yaml
embodiment: other_bot
actions:                       # source node id -> target spelling
  NavigateToPlace: {name: GoToPlace, ports: {place: target}, defaults: {speed: "0.2"}}
  Speak: {name: Say, ports: {text: message}}
```
BT.CPP built-ins (Sequence, Fallback, Sleep ...) pass through. Any other node missing from `actions` raises `AdaptError`; the adapted XML is re-validated against the target catalog.

## Benchmark

`tools/wicl_bench.py`, 12 navigation tasks: memories learned from one phrasing, evaluated on a paraphrase. Raw output: `docs/wicl-benchmark.json`. Planner: Gemma 4 E2B Q4_0 on the GTX 1650 through llama-server, max 3 attempts, one run, 12/12 training tasks rehearsed into memories.

| Arm | Sim success | First plan validates | Mean attempts to success |
|---|---|---|---|
| (a) no memory (raw feedback retries) | 7/12 | 12/12 | 2.14 |
| (b) memory retrieval | 11/12 | 12/12 | 1.0 |
| (c) memory + reflection | 11/12 | 12/12 | 1.0 |

What this does and does not show:
- **Memory helped here**, mostly by *reuse*: in (b) and (c) a stored plan that still validated was run first, with zero LLM calls. The evaluation tasks are paraphrases of tasks already rehearsed, so this is the best case for retrieval, not a general-planning gain.
- **Reflection added nothing measurable**: (b) equals (c). It mattered during *learning*: with the concrete template remedies (e.g. "leave out timeout_ms") rehearsal produced 12/12 memories, versus 1/12 earlier.
- **Hand-written knowledge is doing work.** The remedies in `bt_rehearsal/llm.py` (`_REMEDY`) are authored by a person, not learned. The 2B model's own lessons were vague ("verify the destination") and did not help alone.
- **I tuned this harness on its own results.** Four runs: 1/12 memories learned with a 600-token cap and a generic lesson, again 1/12 with a stale installed package (my mistake: the fixes were not in the run), 1/12 with better remedies but the LLM sentence replacing them, then 12/12 once the template remedy was always kept. Treat the final numbers as optimistic. 12 tasks, one run, one sample per call: a difference of one task (8 points) is noise, but 7/12 against 11/12 is four tasks.
- Sim-verified only (kinematic 2D sim). Nothing here is a Nav2 or hardware result.