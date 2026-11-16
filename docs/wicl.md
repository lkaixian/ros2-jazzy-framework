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

`tools/wicl_bench.py`, 12 navigation tasks: memories learned from one phrasing, evaluated on a paraphrase. Raw output: `docs/wicl-benchmark.json`. Planner: Gemma 4 E2B Q4_0 on the GTX 1650 through llama-server with the same JSON-schema constrained decoding as `llm_bridge`, max 3 attempts. 5 full repeats (learn + evaluate each time); memories learned per repeat: 10 to 11 of 12. Two splits: **reuse** (paraphrases of rehearsed tasks) and **held-out** (8 new goal compositions, never rehearsed, never used for tuning).

| Split | No memory | Memory | Memory + reflection |
|---|---|---|---|
| reuse | 35% (25 to 42) | 88% (83 to 92) | 92% (92 to 92) |
| held-out | 35% (25 to 50) | 92% (75 to 100) | 100% (100 to 100) |

(mean, min to max over 5 repeats.)

**Do not read this as "memory content helps".** A control (`zero-shot` vs one FIXED example that is unrelated to the held-out tasks, same 8 tasks, 5 repeats) gave **38% zero-shot and 100% with the single fixed example, all on the first try**. So almost all of the gain here is the example teaching the small model the output format (in particular, not to invent `timeout_ms`), which a static line in the prompt gives just as well. What the benchmark does show:
- With *some* example in context the 2B model plans these tasks reliably; without one it does not.
- Memory reuse of an already-validated plan is cheap (no LLM call) and was correct on the reuse split.
- It does **not** show that task-specific memory, or lessons from failures, beat a generic fixed example. Reflection's extra 4 to 8 points over memory-only are within what this setup can resolve (8 to 12 tasks).
- Learning (rehearsal) needed concrete lessons: the model's own were vague. With the concrete remedies rehearsal produced 10 to 11 of 12 memories; with only the model's lessons it produced 1 of 12 (earlier runs).

Practical consequence: the cheapest improvement for the small model is a better static prompt (one example plus "do not invent timeout_ms"), independent of this feature. Whether task memory adds anything beyond that needs a harder task set where one fixed example does not already give about 100%.

Limits: kinematic 2D sim only; 2B model only (a larger reflector was not tested: `--reflector-endpoint` exists but no larger model was available here); hand-written remedies in `llm.py`; one author-written task set.