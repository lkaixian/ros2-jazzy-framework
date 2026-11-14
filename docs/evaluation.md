# Evaluation: how plan quality is measured

Two different questions, two different tools.

| Question | Tool | Needs |
|---|---|---|
| Does the **SLM** render specs correctly? | `finetune_bt_slm` held-out eval (valid / exact vs rules) | GPU or CPU, no LLM |
| Does the **whole pipeline** plan what a command asked? | `tools/integration/llm_eval.py` | a running stack with a real LLM |

The second is what matters on the robot. It sends commands to a live `llm_bridge -> orchestrator -> validator` chain and scores each plan on **what the command asked**, not just on validity.

## Running it

```bash
# own DDS domain, extensions up BEFORE the pipeline (pose extension included, or FindFreeSeat / CountPeople plans are invalid)
export ROS_DOMAIN_ID=82
llama-server -m models/llm/gemma-4-E2B-it-Q4_0.gguf --host 127.0.0.1 --port 8081 -c 6144 -np 1 -ngl 99 --reasoning off &
ros2 launch framework_bringup extensions.launch.py enable_arm:=false &
ros2 launch framework_bringup pipeline.launch.py llama_url:=http://127.0.0.1:8081 &
python3 tools/integration/llm_eval.py --gen 100 --seed 1                      # 100 generated GPSR-style commands
python3 tools/integration/llm_eval.py --jsonl tools/gpsr/handwritten.jsonl    # 44 hand-written commands
python3 tools/integration/llm_eval.py                                         # the 15 built-in commands
```

Each command prints `ok`/`MISS`, validity, attempts, latency and the steps; a summary gives scores per category. Exit code 0 means at least 70 % of plans were valid and right (a measurement, not a guarantee).

## What "right" means

`tools/gpsr/commands.py` generates commands from templates with an **expectation** (checker) and a **gold plan** per command; categories are `navigation`, `people`, `vision` and `refuse`. A plan is right when it is valid **and** satisfies the checker: for example "go to the door and say hello" must contain a navigation step to `door` and a `Speak` with that text; a refusal command ("open the door" on a robot without an arm) must contain **no** driving. An invalid plan counts as a miss.

The command wording is ours, not the official GPSR sentences. `tools/gpsr/handwritten.jsonl` holds 44 commands written by hand (ids 1000+) to avoid flattering the model with template phrasing; add the real GPSR wording when you have it. A test validates every gold plan against the real catalog (`bt_validator/test/test_gpsr_commands.py`).

## Results (Gemma 4 E2B Q4_0, JSON-schema constrained, SLM v5)

| Set | Result |
|---|---|
| 100 generated commands, seed 1 | **99 %** right, 100 / 100 valid (navigation, people, vision, refuse) |
| 44 hand-written commands | **95 %** right, all valid |
| Refusals | 12 / 12 and 7 / 7 after the deterministic no-arm guard |

History of the generated set: 86 % baseline, 90 % once the pose extension was running (the 86 % run was missing it), 94 % after macro description edits, 99 % after worked examples and prompt rules. Remaining misses are question phrasings ("is there anyone in the right room", "do you see a laptop anywhere").

## The order of fixes (cheapest first, nothing trained)

1. **Macro and node descriptions** (`macros.yaml`, extension manifests): the LLM chooses by description. "Never picks up", "greeting only", "ONLY for empty seats" fixed whole classes of misses.
2. **Worked examples** (`param/llm_knowledge.md`): each example is test-validated against the catalog. Add the failing phrasing with its correct plan.
3. **Rules in the knowledge file**: for example "numeric ports take a plain number such as 0.5, never >0.5".
4. **System prompt** (`llm_bridge_node.cpp`).
5. **Deterministic guards** for things the model must never get wrong: `bt_orchestrator/policy.py` refuses tasks that need an arm when none is configured (regex of carrying verbs, with an escort exclusion; a `shortcut:` heuristic).
6. Only then consider fine-tuning.

After any change re-run both sets and `tools/integration/prompt_lab.py`: the small model is prompt sensitive (T-21).

## Pitfalls that produced wrong numbers

* **Scores shifted by one command** when a plan from an earlier run was still in flight: the script now drains for 60 s first.
* **Leaked nodes** from an earlier run (or starting `extensions.launch.py` twice) exhaust DDS participants (`error creating node`): stop everything and use a fresh `ROS_DOMAIN_ID`.
* **Context too small**: the planning prompt is 3.2-3.7k tokens; four slots sharing 4096 truncate it. Use `-c 6144 -np 1`.
* **Thinking on**: Gemma 4 spends `max_tokens` thinking and the reply is cut ("not a JSON object"). Use `--reasoning off`.
