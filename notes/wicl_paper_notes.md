# WICL paper notes

Fang, Guo, Zhang, Du, Liang, Bai, "World In-Context Learning for Agentic Robot Manipulation", arXiv:2611.12885v1 [cs.CV] (9 Nov 2026), https://arxiv.org/abs/2611.12885. Code: https://github.com/ArcheOne-AI/RoboScript (not used here).
Read from the arXiv PDF (the HTML version does not exist). Section numbers refer to the paper.
The paper's own system is called RoboScript. Its world is RoboTwin 2.0 (dual-arm manipulation sim).

## The actual loop (Sec. 3, 4.2, App. C.4, Table 4)

Frozen model, context C. `C0 = I(d)` from ONE demonstration d. Then repeat: act in world on a rehearsal
instance e_k -> (trajectory, feedback r) -> `C_{k+1} = U(C_k, tau, r)`. Same frozen model does init and update.
After convergence C* is frozen and evaluated on disjoint instances with NO world feedback.

Two stages:
- **S1 imitation**: reader role writes the first memory from the demo; actor practises on the demo instance;
  reviewer revises after each attempt. Exit: 3 of the 5 latest no-hint attempts pass. Cap 60 attempts.
- **S2 rehearsal**: new training instances in order. Only the FIRST attempt per instance counts (3 tries).
  On failure the reviewer edits a CACHED COPY; the edit is merged into the shared memory only if a retry
  succeeds. Edits are typed, with a budget `max(1, round(4*alpha_k))`, alpha_k = max(0.2, 10/(9+k)), so the
  memory stabilizes. Early stop: 3 first-try successes in the 5 latest instances. Cap 60 instances.
  If the cap is hit, freeze the version with the best Wilson lower bound on first-try success.
- **Deployment**: frozen memory, scene anticipation (map memory roles onto the current scene, check
  reachability), failure recovery without reset, completion check against the demo's final state.

## Task Memory contents (Sec. 4.2, C.4)

Structured Markdown: instruction, object roles and how to find them, per-stage workflow with completion
criteria, lessons, notes. Separate typed parameter table (GraspSpec / PlaceSpec, relative to perceived object
parts). Every version stored and hashed. Linter before commit rejects: simulator names, evaluator terms,
instance identifiers, absolute positions, distances from the robot, fixed arm assignments.

## Data isolation (Sec. 4.2, App. C.6, Table 5)

Two tiers. Deployable (D): RGB-D, intrinsics/extrinsics, joint states, gripper opening, motion-planner
feedback, step count, instruction. Privileged (P): GT poses, meshes, contacts, GT segmentation, success
check, expert trajectories. Enforced by construction: agent code reaches the sim only through an interface
that returns D data; only the control plane holds P; tools per role derived from tier; deployment mounts no
P tool; outputs filtered. Tests: evaluation stages refuse privileged tools; a single module may touch
privileged state; no text shown to the model is task-specific. Memory linter is the second barrier.
IMPORTANT: privileged data IS used in learning, as supervision for the reviewer (a ground-truth diagnosis of
each failure, stated in object/role terms) and in S1 at the privileged tier. It reaches the actor only
through the lint-checked memory. The privilege-free ablation scored the same (86.5 vs 86.0).

## Reflection (Sec. 4.4, Eq. 3, 5.5)

Two different mechanisms, both called reflection-ish:
1. Reviewer inside S2: reads the attempt record and a GT diagnosis, makes typed edits to the memory.
2. Recursive self-improvement (RSI): after an evaluation block, a reflection agent reads episode records
   (deployable observations + success bit only, no failure reason) and writes a guidance document G_{j+1}
   for the NEXT disjoint block. Entries have status (active/probation/retired) and helped/harmed counters.
   Result: net +18, +8, +4 rescued episodes on the 17 weak tasks over 3 rounds; +18/+7/+1 on all 50.

## Cross-embodiment (Sec. 5.7)

A "lightweight adapter" remaps coordinate conventions, arm kinematics and gripper parameters. Workflow,
object roles and relative specs are untouched. No re-demo, no re-rehearsal. ALOHA (sim) -> PiPER (real).
Full action sequence planned offline, no online replanning. 4 tasks x 10 episodes: 10/7/7/6. The paper gives
no adapter details beyond this (not a capability-file mechanism).

## Numbers worth knowing

83.4% mean on 50 tasks (GPT-6.1-Sol), 77.9 to 78.1 for weaker models. Ablation: memory without world
interaction 69.0 vs 78.5 with. Cost USD 308 to 1,911 for learning plus 1,000 eval episodes at list API price.
Evaluation episode: about 55 model calls, 2.3M tokens. All with frontier hosted models, "high" reasoning.

## Differences from the task brief (paper wins)

| Brief | Paper |
|---|---|
| Task Memory has task text, scene summary, validated BT, lessons, outcome, versions | Memory is semantic Markdown (roles, workflow, typed params, lessons). It deliberately holds NO scene-specific data: no positions, no instance ids. A "scene summary at rehearsal time" and a stored concrete BT are not in the paper's memory. |
| Rehearse until success, then store | Rehearse on MANY instances (about 11 to 15 per task), learn from first-try success rates, edit with decaying budgets, merge edits only if a retry passes. A single success is not the stop rule. |
| Data isolation: GT only for scoring | GT is also fed to the reviewer as a diagnosis (stated in role terms), and the memory linter filters it. Paper says its privilege-free variant is as good. Brief is stricter, which is fine. |
| Reflection writes a lesson from each failure | Closest match is the S2 reviewer. The separate RSI guidance loop works across disjoint blocks, not per failure. |
| Retrieve top-k memories by task/scene overlap | No retrieval: one memory per task, picked by task. Retrieval is our addition. |
| Offer a retrieved BT as first candidate | Not in paper. Paper's agent re-plans from the memory on every scene (anticipation). |
| Small local models | Paper relies on frontier models. It reports a weaker model works but nothing near 230M parameters. Expect much weaker reflection from our SLM. |
| Cross-embodiment through a capability file | Paper's adapter is described only in one paragraph. Capability file is our design. |

Also: the 2D nav BT setting has no vision, so the paper's evidence (manipulation, RoboTwin) does not carry
over. Whatever we measure is our own evidence.
