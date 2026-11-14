# Behaviour trees, macros and the task lifecycle

## Hand-written behaviour trees
```
ros2 run bt_validator validate_bt --list-nodes          # every node + ports you may use
ros2 run bt_validator validate_bt my_tree.xml           # check offline, readable errors
ros2 run bt_orchestrator submit_bt my_tree.xml --run    # validate -> bt_executor (dry-run unless dry_run:=false)
```
Examples in `framework_bringup/behavior_trees/`: `greet_person.xml` (detect person -> `ExtractFaceFeature` -> `Speak "Hello {person}"`) and `enroll_person.xml` (`EnrollFace`). Port names `name` and `ID` are reserved by BehaviorTree.CPP and are rejected. Speak needs `tts_node` (`audio.launch.py enable_stt:=false`), EnrollFace/ExtractFaceFeature need `face_recognition_node` (publishes embeddings).

## Macros and the task lifecycle
`param/macros.yaml` holds 36 named, parameterised step lists (`go_to_place`, `go_and_say`, `patrol`, `wander`, `meet_guest`, `receive_guest`, `scout_room`, ...; full list in `docs/macros.md`, generated; only the `whitelist:` ones may be called by the LLM/SLM). Task specs can also use `repeat`, `Parallel` groups and `foreach` loops (`docs/task-spec-grammar.md`). The LLM can write `{"macro": "go_to_place", "args": {"place": "door"}}`; the orchestrator expands it to real nodes before the SLM/rule generator, and a test expands and validates every macro against the real node catalog. Bad calls refuse the plan with a message naming the problem. Add your own by adding an entry with `example_args`. The rule-based generator is exact for every spec in the grammar and was measured at 0.06 ms against 1.5 s for the fine-tuned 230M model (GTX 1650), so `bt_slm` tries the rules first and the model only handles retries or specs the rules cannot render. The spec grammar still has no Parallel/loops/blackboard planning.
`task/state` (latched) follows planning -> ready -> running -> done/failed/halted. With `bt_orchestrator.max_repairs > 0` a failed run is re-planned through the LLM with the failing node (`failed_node` in `bt/status`); default 0. Every Action/Condition node is tagged `sync` or `async` in the LLM's node list (derived from how `bt_executor` really runs it).
