# WICL design (draft, nothing implemented yet)

Branch: `feature/wicl`. Paper differences: see `wicl_paper_notes.md`. Verification ceiling: sim (2D nav). Not hardware.

## Integration points found in the code

| Where | What it does today | WICL use |
|---|---|---|
| `bt_orchestrator/orchestrator_node.py` `_on_spec` -> `_run` | spec -> macro `expand` -> `bt_slm/generate` (GenerateXml) -> `bt_validator/validate` (ValidateTree), up to `max_attempts`, errors fed back as `previous_errors` | Retrieval hook goes before `_run`. The retry loop already carries validator errors. We reuse it. |
| `bt_slm/fallback.py` `build_xml(task_spec, known_ids, errors)` | Pure Python rule generator. Raises on unknown steps in a retry (T-22 fixed). | Offline planner for rehearsal. Pure function: no ROS needed. |
| `bt_validator/validate.py` `validate_xml(xml, catalog, vocab)` | Pure Python, returns error strings | Used directly by rehearsal and by retrieval re-validation. |
| `bt_orchestrator/scene_summarizer_node.py` | Publishes `scene/summary` JSON: `objects`, `num_persons`, `people[]`, `known_faces` | The ONLY scene source for memory and planner context. Real-robot observable by construction. |
| `llm_bridge` (C++) -> `bt/task_spec`; `bt/repair` (text with failure) | Prompt = catalog + scene + task. Failover, schema-constrained JSON | Rehearsal planner and reflection call it through `llm/query` style service (`LlmQuery.srv`). Degrade to rules when no endpoint. |
| `task_fsm.repair_prompt` + `RecoveryPolicy` | Runtime repair by failure text | Same wording style for rehearsal feedback to the planner. |
| `nav_sim` (`kinematics.Robot`, `occupancy.OccupancyMap`, `/nav_sim/ground_truth`) | ROS node, plus ROS-free classes | Ground truth lives here already (`ground_truth` topic). This is the privileged tier. |
| `bt_executor` | dry-run default, `run`/`halt`, `bt/status` with `failed_node` | Not touched. Rehearsal never starts it live. |
| `bt_pipeline_msgs` | `BtPlan`, srvs | New: `RehearseTask.srv`, `ListMemories.srv` (phase f). |
| `framework_bringup/launch`, `param/` | all params and launches | New launch `wicl.launch.py`, `param/wicl.yaml`. `check_params` table needs entries. |

## What is added

- `bt_memory` (pure Python + thin ROS): Task Memory dataclass, YAML store, version gate, `Retriever` interface
  (`TextSceneRetriever` first), linter, adapter + capability file.
- `bt_rehearsal`: `Simulator` protocol, `NavSimKinematic` implementation, rehearsal loop, reflection, CLI + ROS service node.
- Orchestrator: optional hook `memory_enabled` (default false). When off, code path unchanged.

## Data isolation (enforced in code)

Two types, no shared base:
- `Observable` (scene summary, validator errors, executor-style outcome with failed node/step, step count).
- `Privileged` (ground-truth pose, goal distance, collision cell, success flag detail). Only the `Scorer` sees it.
`Simulator.run()` returns `(Observable, Privileged)`. The planner/reflector API accepts only `Observable`
(runtime `isinstance` check plus a key allowlist filter on any dict, as paper's linter). The scorer converts
`Privileged` to the pass/fail bit. Test: seed a sentinel value into the privileged data and assert it never appears in any
prompt or any stored memory. Also a memory linter rejects coordinates and pose-like numbers in lessons.

## Open design choices that have real trade-offs (asking you)

1. **Simulator backend.** A) in-process kinematic interpreter of the BT over `OccupancyMap` + `Robot`
   (no ROS, deterministic, fast, runs in CI, benchmark of 10+ tasks in seconds; does not run the real
   BehaviorTree.CPP executor). B) drive the real executor + nav_sim + Nav2 over ROS (more faithful; slow, needs a launch per
   attempt, flaky for a benchmark). Recommendation: A, with the `Simulator` interface so B can come later.
2. **What the memory holds.** The brief stores a concrete validated BT XML plus scene summary. The paper stores
   semantic knowledge and refuses scene-specific content. I'd keep the brief's BT-XML record (needed for "offer as
   first candidate"), but keep lessons semantic (linted) and always re-validate. OK?
3. **Who reflects.** The 230M SLM cannot write free-text lessons. Options: LLM via llm_bridge when reachable,
   otherwise a template-based lesson from the validator errors/failed step (deterministic, offline). Recommendation: that.
4. **Benchmark honesty.** With a rule generator that is already right for in-grammar specs, memory may show
   no gain. To make the comparison meaningful, the benchmark needs tasks where the first plan fails (e.g.
   a mocked LLM that proposes wrong places/ports until given feedback). I will say so in the results.
5. Unrelated untracked files (1.jpeg, 2.jpeg, mp4) are left alone, not committed.
