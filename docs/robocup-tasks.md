# RoboCup@Home Malaysia Open 2026 (3-7 Dec): one launch file per task

Sources: meeting minutes 2026-11-25 (`~/Downloads/MEETING MINUTES.pdf`: Stage I = Help Me Carry, GPSR, Receptionist, Storing Groceries; 3 Dec = inspection
and team leaders' meeting) and the score sheets, draft 2027 revision 2026-11-24 (https://robocupathome.github.io/RuleBook/scoresheets/master.pdf; the
numbers match `notes/TASKS.md`, which was copied from the 2026-10-14 revision). Both are drafts: check them again after the 3 Dec meeting.

```bash
ros2 launch framework_bringup robocup.launch.py          # EVERYTHING on (audio, sensors, OAK, Nav2, all perception, arm)
ros2 launch framework_bringup gpsr.launch.py             # one file per task, below
```

| Launch file | Task | When | Starts besides the core (LLM, validator, executor, mux) |
|---|---|---|---|
| `help_me_carry` | Help Me Carry | Stage I | person + depth, Nav2, sensors, audio |
| `gpsr` | General Purpose Service Robot | Stage I | audio, objects, person + pose + depth, Nav2, sensors |
| `receptionist` | HRI Challenge "Receptionist" | Stage I | audio (doorbell), faces, person + pose + depth, objects, Nav2, sensors |
| `storing_groceries` | Storing Groceries | Stage I | objects, arm + OAK, Nav2, sensors (no audio) |
| `clean_the_table` | Clean the Table | Stage II | objects, arm + OAK, Nav2, sensors (no audio) |
| `egpsr` | Enhanced GPSR (also the Final round format) | Stage II / Final | audio, objects, person + pose + depth, arm + OAK, Nav2 |
| `restaurant` | Restaurant | Stage II | audio, objects, person + pose + depth, Nav2 (no arm) |
| `give_me_a_hand` | Give Me a Hand | Stage II | audio, objects, person + pose + depth, arm + OAK, Nav2 |
| `inspection` | 3 Dec robot inspection (gate -> goal in 3 min) | 3 Dec | sensors + Nav2 + mux only |

Each is `framework.launch.py` with that task's switches (printed at start-up, `bt_common.launch_utils.task_launch`). Anything given on the command line
wins (`enable_arm:=true`, `enable_navigation:=false`); `dry_run` stays true until you pass `dry_run:=false`. **The GAPS of each task are in the docstring at
the top of its launch file** (and the score-sheet tables below for the two scored on the 2027 sheet). Arm launches need the arm driver first (`tools/arm/arm_up.sh`).

## GPSR (max 1490, 7 min)
| Score | Needs | Launch | Gap |
|---|---|---|---|
| understand the command 3x80 (-30 rephrase, -50 bypassing ASR) | mic -> VAD -> STT -> `bt/task` | audio on | real-mic accuracy unverified |
| show the plan 3x100 | `plan_announcer` speaks `bt/plan` | `announce_plans` (default on) | none known |
| solve 3x250, interleaved bonus 200 | LLM -> validator -> executor, Nav2, objects, people, pose | all on, depth-ranged persons | command-generator grammar vs the real LLM; no arm (pick/give commands) |

## Receptionist / HRIC (max 1450, 7 min)
| Score | Needs | Launch | Gap |
|---|---|---|---|
| doorbell 2x30 | `sound_event_node` | audio on | untested with a real bell |
| open the door 2x200, bag handover 50, drop bag 50 | arm | off (`enable_arm:=true`) | `receive_guest` asks a person to open the door; arm side not ready |
| look at the speaker 2x50, look at the right guest 2x50 | person + depth, turn the base | on | no head: the base turns |
| free seat 2x100 | objects (chair/sofa) + persons, macro `offer_seat` | on | exists, untested with real chairs |
| attribute of guest 1 to guest 2 (+-20 x4), name + drink 4x30, no needless confirmations 4x15 | face + person, STT/TTS, macros `meet_guest`, `describe_person`, `introduce_guest`, `hric_round` | on | only the shirt colour is described (glasses, hair, jacket are not) |
| follow the host to the bag area 200 | `FollowTarget approach` + depth | on | rediscovery / losing the host |

Plan read-out is off (`announce_plans=false`): this test does not score it. Wrong memorised information is -40 each, not acknowledging people -200 each.

## Known blockers seen on 2026-11-25 (not caused by the launch files)
- `person_detection_node` died on its first frame: ultralytics' tracker needs the Python package `lap`, which is not installed.
- `face_recognition_node` needed face weights: now `yolo26s-face.pt` (akanametov/yolo-face release 1.0.0, GPL-3.0, pickle scanned, tested on a real photo) in the repo root (gitignored: `*.pt`; download it again on another laptop).
- `cmd_vel_mux` stays stopped without `kobuki_ros_interfaces` (source `~/ros2_ws`): by design, a safety input it cannot read keeps the robot stopped.

## `~/Downloads/task1..3_homedu.py`
Another team's / earlier scripts (Help Me Carry and Receptionist), not used here: they publish straight to `/cmd_vel` (no mux, no limits, no bumper latch),
hard-code paths, and task1 has a syntax error at line 286.

## Open on every task (the cross-cutting gaps)
- The arm chain (grasp, object 3D pose, hand-eye calibration, force limit, door / dishwasher): `notes/ARM_BRINGUP.md`. Five of the nine launch files depend on it.
- The competition maps, places (`start`, `inspection`, shelves, dishwasher, dustbin, the door) and detector classes for the real objects come from the 3 Dec
  mapping and dataset slots; none exist yet.
- Depth obstacles stay off until the RealSense mount is measured (`tools/camera/floor_plane.py`), so obstacle avoidance is lidar-only (one plane).
- Stage II tasks and the Final are known only from the minutes (no score sheet): scoring and details come from the 3 Dec team leaders' meeting.
