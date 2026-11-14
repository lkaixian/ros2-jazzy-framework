# Task spec grammar

What the LLM (or you) writes, and what happens to it. The spec is JSON; `bt_orchestrator` expands macros and loops, `bt_slm`
renders it to BehaviorTree.CPP v4 XML, `bt_validator` checks the XML, `bt_executor` runs it.

```
SPEC  = {"goal": str, "retry": N?, "steps": [STEP, ...]}          # at most 12 steps per list
STEP  = {"macro": name, "args": {arg: "string", ...}}              # a whitelisted macro (docs/macros.md)
      | {"node": Id, "ports": {port: "string", ...}}               # one BT node from the catalog
      | {"group": G, "retry": N?, "repeat": N?, "success_count": N?, "failure_count": N?, "steps": [STEP, ...]}
      | {"foreach": [v1, v2, ...], "as": "x", "steps": [STEP using ${x}]}      # task specs only, 1-8 values
G     = Sequence | Fallback | ReactiveSequence | ReactiveFallback | Parallel
```
All port/arg values are strings (`"0.5"`, `"door"`). A value `"{name}"` is a blackboard reference (see outputs below).

| Construct | Meaning | Becomes |
|---|---|---|
| `Sequence` | run children in order, stop at the first failure | `<Sequence>` |
| `Fallback` | try children in order, stop at the first success ("try A, otherwise B") | `<Fallback>` |
| `Reactive*` | like the above but re-checks earlier children on every tick | `<ReactiveSequence>` ... |
| `Parallel` | run children at the same time (e.g. speak while driving); `success_count` = how many must succeed (default 1), `failure_count` likewise | `<Parallel success_count=..>` |
| `retry: N` | run the group again if it fails, up to N attempts (outermost wrapper) | `<RetryUntilSuccessful num_attempts=N>` |
| `repeat: N` | run the group N times in a row (loop); fails if one round fails | `<Repeat num_cycles=N>` |
| `foreach` | unrolled at plan time: the steps once per value, `${x}` replaced | N copies of the steps |
| `macro` | named step list from `macros.yaml`, expanded before everything else | its steps |

## Outputs (node results into later steps)
A node with an optional `out_<name>` port (see the catalog line, e.g. `WaitForTranscript(timeout_ms?,out_text?)`) writes its result to the
blackboard when the XML names a variable: `{"node":"WaitForTranscript","ports":{"out_text":"{heard}"}}`, then a later step reads `"{heard}"`.
`Speak` only substitutes `{person}`, so to say a variable use `{"text":"I heard {person}","person":"{heard}"}`. In dry-run nothing runs, so
no output is produced (the downstream port is just omitted).

## Limits and rules
- Validator limits (`param/pipeline.yaml`): tree depth 12, 160 nodes (a macro such as `hric_round` expands to about 100).
- An unknown node/macro, a missing required port, an empty `{...}`-less port, or a name the detector cannot produce is rejected with a message
  that is fed back to the LLM (up to `max_attempts`).
- `Parallel` children that use the same extension action server conflict: an extension runs one Action at a time and rejects a second.
- `foreach` is not allowed inside macro bodies (a macro takes parameters instead).
- When the LLM server supports it, `llm_bridge` constrains the reply to a JSON schema built from this grammar (only whitelisted macros, real
  node ids and real port names, string values); see `notes/RESEARCH.md`.
- **Known hazard:** if the validator has not yet received every extension's manifest, the retry path drops steps whose node it does not know yet
  and can accept a shorter plan (seen once in a rehearsal). Start extensions before sending tasks (`bt_validator/catalog` lists what it knows).

Hand-written trees skip all of this: `ros2 run bt_orchestrator submit_bt <file.xml>` still goes through the validator.
