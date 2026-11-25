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

## Help Me Carry and Receptionist: what is built, what is not (2026-11-26)
Built (nodes tested in simulators with fake persons / scans / transcripts: `tools/integration/follow_test.py`, `phrase_test.py`; macros validated, never run on a robot):
- `FollowTarget until_cancelled=true` keeps following after arriving; `lock=true` stays with the first person (track id); an obstacle that is not the person makes it turn toward the freer side (`ScanGuard.freer_side`, reactive, no detour planning).
- `WaitForPhrase` (social_extension) ends a long action by voice; `Speak` got `detail` and `skip_if_unknown`.
- Macros `follow_until_told` (Parallel of FollowTarget and WaitForPhrase, 3 searches for a lost person) and `help_me_carry` (follow, stop word, `return_home`).
- `hric_round` now: doorbell, door, welcome, face the guest, name + drink, how they look (`note_look`), lead to the living room, introduce to the host (`introduce_to`), offer a seat with its side ("chair on my left"), guest 2 introduced to guest 1 and told how guest 1 looks (`tell_look`). 146 of the validator's 160 nodes.
Still open (also in each launch file's GAPS): door opening, bag handover / drop (arm); a real host (found, recognised) and checking the guest follows; looking at the right guest (only the nearest person); no path retrace outside the arena map, no detour planning, the lock is lost when the tracker changes the id; seat finder on synthetic boxes only; all of it unproven on the robot and with real audio.

## The four arm tasks: what is built (2026-11-26)
Software only, on top of the guarded arm stack (`arm_extension`: moves nothing live until `live: true` AND `dry_run:=false`; arm places are still an empty `places` parameter). Needs `tools/arm/arm_up.sh` and `tools/arm/arm_eye.sh` (-> `/arm_eye/objects`) running.
- `NextObject` (pose_extension) picks the next object the arm eye sees for a task (`clean_table`, `groceries`, `rubbish`) from `param/object_categories.yaml` (label -> group, named place to drive to, arm place to put it) and prefers another label than the last one. Placeholders: COCO labels, dustbin / dishwasher / shelf names, none of them mapped or taught yet.
- Macros: `clean_the_table_round`, `store_groceries_round` (go to the table, then per item NextObject -> `pick_object` -> drive -> `put_down` -> say it -> back; up to 6; a failed pick does not stop the round), `dispose_rubbish`, `egpsr_round` (rubbish, then `assist_waving_person`), `give_me_a_hand_round`.
- `give_me_a_hand_round`: opens the gripper at the handover point (ESTIMATE x y z), waits for the person to say "ready" (`WaitForPhrase`), closes (`CloseGripper` succeeds only on a stall), asks where it goes and parses table / shelf / bin (`ExtractAnswer kind=place`), drives there and puts it down. The PERSON triggers the grasp: the robot never closes on a hand by itself, but that also means the rule "no contact with the hand" is not met.
- Planning (Gemma E4B, `llm_eval.py --gen 100 --seed 1`, with these macros and templates): 100/100 plans valid, 94/100 right; the five new macros were chosen for their commands (clean 3, dispose 3, egpsr 3, hand 1, store 1). Misses: two ambiguous phrasings ("put the groceries ... on the shelf" is also a fair `move_object`; "let me hand you something" got a plain reply; both dropped from the templates), "grab the scissors" -> `find_object` (an old miss), "welcome the guest" -> only `wait_for_doorbell` once.
- Tested: `tools/integration/arm_task_test.py` runs the plans in a live executor against fake extensions with the real manifests, the real object table and answer parser (15 checks: call order, values passed from NextObject to pick / drive / place, loops ending, a failed pick, the gripper closing only after the voice cue, no placing on an answer that names no place). No arm, no robot, no real object.
- Open (also in each launch file's GAPS): grasping any real object (hand-eye calibration, object pose, grip force: `notes/ARM_BRINGUP.md`), arm places taught for table / shelf / bin, shelf levels and looking at the shelf, the dishwasher (not an arm place; door and rack not opened), real object classes and the real rubbish / groceries (3 Dec), hand detection for a contact-free handover, the venues' maps.

## EGPSR, Restaurant, Inspection, GPSR: what is built (2026-11-26)
- **EGPSR / Final:** macro `assist_waving_person` = `find_waving_person` (turns on the spot ~2 turns until `IsPersonWaving`), `approach_person`, "How can I help you?"; the answer is heard as the next task by the normal speech -> plan -> run path. Not built: watching for waves while another task runs, rubbish (no detector class, arm), memory of done tasks.
- **Restaurant:** macros `take_order` (asks, repeats it back, needs a yes, 3 tries), `tell_order` and `restaurant_round` (two customers: wave -> approach -> order -> back to the counter `start` -> tell it). The order is the whole heard sentence (no menu); serving needs the arm; the venue has no map.
- **Inspection:** `tools/integration/inspection_test.py` runs the simulator (nav_sim + Nav2/AMCL + mux, bumper/cliff events off because the simulator has no Kobuki messages) from the start through the door to the right room: SUCCEEDED in 44.9 s of the 180 s limit, final error 0.24 m. Simulator only: nothing is placed in the way (no avoidance check), not the real lidar, arena or robot; the arena map and the `inspection` place come on 3 Dec.
- **GPSR on Gemma E4B** (`llm_eval.py --gen 100 --seed 1`, own DDS domain, llama-server E4B Q4_0, median 0.7 s per plan, max 2.1 s). Run 1 had the arm extension OFF (setup error: every plan with arm nodes was invalid): 89 % right. With it ON: 100/100 valid, 95/100 right (manipulate 7/8, navigation 32/32, vision 12/12, refuse 8/8, people 36/40). After new prompt examples and templates for the new macros (a different command mix): 99/100 valid, 95/100 right; the two manipulate misses were "I cannot see a remote / bottle" because the evaluation scene has no object detector running (arguably correct). Real problems found and fixed: "follow me around" -> `wander`, "receive the next guest" -> only `wait_for_doorbell`, and "receive both guests and introduce them" chaining `receive_guest` twice around `hric_round` (209 nodes > the 160 limit: its description now says to use it alone). Not re-run after that last description change. The command templates are ours, not the official GPSR sentences. Prompt limit raised 4800 -> 5400 bytes; the arm-verb refusal policy (only when `has_arm` is false; default true) no longer treats "help me carry" / "take the orders" as arm work.

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
