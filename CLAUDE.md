# CLAUDE.md

ROS 2 Jazzy workspace: natural-language task -> LLM task spec -> BT XML (rules first, SLM on retry) -> `bt_validator` -> `bt_executor` (BehaviorTree.CPP v4) -> extensions -> `cmd_vel_mux` -> robot. `README.md` has the architecture diagram and package table; `docs/` the details. Picking this up cold: read `notes/README.md`, `notes/BACKLOG.md`, `notes/technical_debt.md`, `docs/status.md` (what is and is not verified).

## Build / test

```bash
source env.sh                                   # system Python first, ROS, install/, CycloneDDS config
colcon build --packages-select <pkg>            # then re-source env.sh
colcon test --packages-select <pkg> && colcon test-result --verbose
python3 -m pytest src/<pkg>/test -q             # fastest loop for one Python package
ros2 run framework_bringup check_params         # param files vs the real nodes
ros2 launch framework_bringup framework.launch.py dry_run:=true
```

- Integration sims live in `tools/integration/` (each on its own `ROS_DOMAIN_ID`, localhost discovery). CI is `.github/workflows/ci.yml`; see `docs/testing-and-ci.md` before writing a sim test (kill/reset/discovery-race gotchas).
- C++ packages: `llm_bridge`, `bt_executor`, `dstar_lite_planner`. The rest are ament_python or message packages.
- Jazzy only on the robot; Humble builds in CI (unit tests only). Do not mix distros on one DDS domain.

## Safety rules for edits

- `bt_executor` defaults to `dry_run:=true`, `auto_run:=false`; keep it that way. Nothing here has run on a real arm, second machine or real mic audio.
- All motion goes through `cmd_vel_mux` (priorities, clamps, 0.5 s watchdog, bumper/cliff latch). The software mux is not an e-stop.
- Do not relax a limit/latch/threshold because a run failed; find what it measures first. Nav2 params (`scan_mask_node`, 0.25 m min range) exist because of the robot's own standoffs.
- Every ROS process is a DDS participant; Cyclone's limit is raised to 200 in `tools/net/cyclonedds_local.xml`.

## Conventions

- A new BT node: add to the `bt_common` catalog (tagged sync/async), the extension's manifest, `docs/node-catalog.md`, and make sure `test_macro_library.py` / `test_validate.py` still pass.
- Parameters live in `framework_bringup/param/`; `check_params` must stay green.
- Commits: short `area: what and why` subject; end with the Co-Authored-By line only (no session link).
- Untracked media in the repo root (`*.jpeg`, `*.mp4`, `.codegraph/`) is not part of the project; do not commit it.
