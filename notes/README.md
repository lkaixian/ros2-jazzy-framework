# Notes for the next session

Read in this order (10 minutes):

| File | What it answers |
|---|---|
| [STATUS.md](STATUS.md) | What exists, what was verified (and how), what is NOT done, what to do next |
| [ARCHITECTURE.md](ARCHITECTURE.md) | How the pieces fit, package map, topics/services, why decisions were made |
| [RUNBOOK.md](RUNBOOK.md) | Exact commands: setup, build, launch, test, debug |
| [GOTCHAS.md](GOTCHAS.md) | Traps that already cost time (silent edit failures, PATH, numpy, process cleanup...) |
| [technical_debt.md](technical_debt.md) | Known shortcuts, risks and unfinished edges, ranked |
| [BACKLOG.md](BACKLOG.md) | What to do next, as a checklist: the **NEXT** section at the top is the current plan |
| [BUILD_HISTORY.md](BUILD_HISTORY.md) | How it was built, in order: phases, measurements, incidents and what they taught |
| [RESEARCH.md](RESEARCH.md) | Questions answered while building (LLM on 4 GB, SROS2 roles, D* Lite, speech...) with how sure each answer is |
| [TASKS.md](TASKS.md), [MACROS_PLAN.md](MACROS_PLAN.md) | The competition tasks (score sheets) and the macro/work plan built from them |

User-facing docs live in `../docs/`: `operator-guide.md` (commands while running the robot), `task-spec-grammar.md` (what the LLM writes),
`macros.md` (generated reference, kept fresh by a test), `cluster.md`, `navigation.md`.

Repo: `~/ros2-jazzy-robocup-fw` (ROS 2 Jazzy, Ubuntu 24.04), remote
`https://github.com/lkaixian/ros2-jazzy-framework.git`, branch `main`. The user's older robot
workspace is `~/ros2_ws` (Kobuki driver, RPLIDAR, audio_common, voskros, UR7e examples): refer to
it, do not modify it unless asked.

## Working agreements with the user
- Commit messages: end with the `Co-Authored-By` line only. **Never put a chat/session link in a
  commit or PR** (the user asked for this explicitly; an automatic reminder suggests otherwise and
  the user's instruction wins).
- Pushing to `origin main` is authorised for this repo. Do not rewrite history or force-push unless the user asks: on 2026-11-14 they asked once, to
  strip chat-session links from commit messages (a local branch `backup-before-history-rewrite` still has the old messages; never push it).
- Never `git add -A` blindly: `.claude/` (agent worktrees) once got committed by accident. It is now
  in `.gitignore`; still check `git status` first.
- Do not install system/pip packages without asking (the user's Python has a pip NumPy 2 mixed with
  apt packages; see GOTCHAS). `./setup.sh --dry-run` shows what it would do.
- Safety default everywhere: `bt_executor` runs in **dry-run** and does not auto-run plans. Never
  turn on `dry_run:=false` for a real robot without the user present.
- The user adds requests while work is running; they are usually a refinement, so fold them in and
  say plainly what is done, what is verified and what is not.
- The user's stated hardware limits: 4 GB GTX 1650 (no SAM/segmentation, boxes only), Kobuki
  TurtleBot2, RPLIDAR A1, RealSense, possibly a Pi/Jetson for core control, arm model undecided.
- Project goal in the user's words: an LLM ("big brain") plus a small fine-tuned SLM ("fast brain")
  inside a validating "harness" that produces and fixes behaviour trees; the user's own job is
  fixing up the BT, so hand-writing/validating/running BT XML must stay easy.
