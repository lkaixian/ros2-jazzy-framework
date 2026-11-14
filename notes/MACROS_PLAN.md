# Macro plan (basic -> competition)

A macro = a named, parameterised step list in `param/macros.yaml` (sequence/fallback groups + retry; no loops,
parallel or decorators until B-03). Status: **now** = only existing nodes, **node** = needs the named new primitive
first, **grammar** = needs B-03. Tasks: G=GPSR, H=HRIC, P=PnPC, L=Laundry, F=Final demo (see `TASKS.md`).
Existing nodes: Speak, EnrollFace, NavigateToPlace/Pose, ClearCostmaps, SetInitialPose, IsNearPose, DriveDistance,
RotateBy, FollowTarget, DetectPerson, DetectObject, RecognizeFace, ExtractFaceFeature, GetPersonPose, IsPersonWaving,
IsPersonPointing, MoveArmToPose, OpenGripper, CloseGripper, Sleep, AlwaysFailure.

## A. Basic (exist: say, go_to_place, go_and_say, patrol, turn_around, greet_person, approach_person, wave_back, report_object)
| Macro | Steps | Status | For |
|---|---|---|---|
| return_home | go_to_place(start) | now | all |
| look_around | RotateBy x4 (pi/2) | now | G,F |
| find_object(label) | Detect; else rotate+Detect up to 4x; say not found | now | G,P,F |
| find_person | same with DetectPerson | now | G,H,F |
| go_find_object(place,label) | go_to_place, find_object | now | G |
| go_find_person(place) | go_to_place, find_person | now | G,H |
| go_find_and_say(place,text) | go_find_person, approach_person, say | now | G |
| say_what_i_see | scene summary spoken | now (needs summary port) | G |
| meet_and_remember(name) | approach_person, EnrollFace(name), say | now | G,H |
| follow_person(distance) | find_person, FollowTarget | now | G,H |
| guide_to(place) | say "follow me", go_to_place, say "we are here" | now | G,H |
| escort_person_to(place) | find_person, guide_to | now | G,H |
| wait_for_wave | retry IsPersonWaving, say hello | now | H,F |
| point_at_place(place) | say direction (no arm) | now | G |
| stop_and_wait(sec) | Sleep | now | all |
| recover_nav(place) | ClearCostmaps, back up (DriveDistance -0.2), retry | now | all |

## B. Speech and people (need listening and attributes)
| Macro | Needs | For |
|---|---|---|
| ask_name_and_drink | node **WaitForTranscript(timeout)** + ask/confirm policy: one question, no needless confirm (H scores that) | H |
| introduce_guest(name,drink) | now: say name + drink | H |
| introduce_guests_to_each_other | now, plus node **LookAtPerson** (gaze/head) for the 2x50 points | H |
| tell_guest_attribute | node **DescribePerson** (clothes, hair, glasses) | H |
| answer_question | LLM answers from the scene; no node, but a plan type for Q&A | G |
| tell_time / tell_name / tell_count | node **CountPeople**, clock via Speak | G |
| count_objects(label) | node **CountObjects** | G |
| acknowledge_person | LookAtPerson + say hello (H penalises -200 x2 for ignoring) | H |

## C. Reception (HRIC)
| Macro | Steps | Needs |
|---|---|---|
| wait_for_doorbell | listen for bell | node **DetectSound(bell)** |
| open_door_for_guest | go to door, open, step back | node **OpenDoor** (arm or base push) |
| receive_guest | wait_for_doorbell, open_door_for_guest, greet, ask_name_and_drink, meet_and_remember | all above |
| offer_seat | find free seat, point/guide, say "please sit" | node **FindFreeSeat** |
| take_bag | handover from guest | node **ReceiveHandover** (arm) |
| follow_host_to(place) | follow_person with rediscovery, no touch/asks | FollowTarget upgrade: lost-target recovery |
| drop_bag(place) | go_to_place, release | node **PlaceObject** |
| hric_round | receive_guest x2, introduce, offer_seat, take_bag, follow_host_to, drop_bag | grammar: repeat 2x written out, fine |

## D. Manipulation (PnPC, Laundry, Final)
Primitives first (none exist): **GraspObject(label)**, **PlaceObject(place)**, **OpenDoorHandle**, **PushPullRack**,
**PourInto(target)**, **OpenContainer**, **FoldCloth**, **HandOver(person)**.
| Macro | Steps |
|---|---|
| pick(label) | find_object, approach, GraspObject |
| place_at(place) | go_to_place, PlaceObject |
| pick_and_place(label,place) | pick, place_at |
| bring_to_person(label,person) | pick, find_person, HandOver |
| clear_table_to(place) | pick_and_place per object (written out; loops need B-03) |
| load_dishwasher_item | open door, pull rack, PlaceObject, push rack |
| serve_breakfast | pick bowl, spoon, cereal, milk; open milk, pour, place (4 pours/placements) |
| tidy_floor_item | pick(from floor) bonus +30 |
| take_clothes_from_machine | open machine door, GraspObject cloth, drop in basket |
| carry_basket(place) | GraspObject(basket), go_to_place, PlaceObject (300 for using the basket) |
| fold_garment | pick from basket, FoldCloth, stack |
| stack_folded | PlaceObject on stack (6x100) |
| do_laundry | take_clothes_from_machine, carry_basket, fold_garment x(1+5), stack_folded |

## E. Final demonstration
| Macro | Needs |
|---|---|
| find_problem_and_say | scene scan + say what is wrong; a "problem" list (open door, dirty table, ...) |
| solve_problem(kind) | dispatch to a macro by kind (LLM picks) |
| open_apartment_door / close_dishwasher / move_basket | OpenDoorHandle, push, carry_basket |

## Order to build
1. **A + B(partial) now (about 20 macros):** only existing nodes; each is covered by the existing macro validation test.
2. **Primitive nodes next, in this order of points per effort:** WaitForTranscript, LookAtPerson, DetectSound,
   FindFreeSeat, DescribePerson, FollowTarget recovery (HRIC and GPSR). Then the arm (B-25) with GraspObject and PlaceObject:
   they unlock all of D (PnPC 3515 and Laundry 4415 points).
3. **Grammar B-03** (loops, parallel) for "for each object" macros.

## Rules for every macro
- Params are strings; add `example_args` so the test validates it against the real node catalog.
- Update `param/llm_knowledge.md` when a capability appears (it says "never plan grasping"; the drift test checks it).
- Keep each macro one scoring idea so a failure shows which score was lost.

---
# Work plan (how the macros land in the existing framework)

Everything below reuses the shipped pattern: a BT node = an entry in `<pkg>/config/<pkg>.yaml` (manifest) + a handler in
that package's `ExtensionExecutor` (`handlers = {'Name': fn}`; ports are strings; result = success + message over
`<ext>/execute`). The validator, LLM node list, SLM dataset sampler and macro test all derive from the manifests, so a
new node is visible everywhere once its manifest is in. Size: S < 0.5 day, M ~ 1-2 days, L > 3 days.

## Per-node checklist (every new node/macro, in this order)
1. Manifest entry (required/optional ports, one-line description the LLM reads).
2. Handler in the extension; bad ports -> `PortError`, honour `ctl` cancel (halt must stop it).
3. Param entry in `framework_bringup/param/*.yaml` + the node in `check_params.NODES` (dash then lists it).
4. New process? launch file with `respawn`, `cluster_jobs.yaml` entry (pinned to `core` if it touches hardware).
5. Unit test of the pure logic + a macro entry with `example_args` (validated against the real catalog).
6. `param/llm_knowledge.md`: state the new capability (and remove "never plan grasping" only when the arm works).
7. `bt_slm/dataset.py` `value_for` rule for the new ports, so synthetic specs stay valid (retrain later; rules_first
   keeps working in the meantime).
8. README package table + `notes/STATUS.md`.

## Phase 0 - foundation: node outputs (B-01 / T-09), M
Blocks WaitForTranscript, DescribePerson, Count*, FindFreeSeat: they must hand a value on.
- Follow the core-node convention already in `registry.cpp`: an optional port `out_<name>` that, when present in the XML,
  is written to the blackboard (`ExtractFaceFeature out_identity="{who}"`).
- `bt_pipeline_msgs/action/ExecuteNode`: add `string[] output_names`, `string[] output_values` to the result.
  Extension generic BT node in `bt_executor/src/registry.cpp` writes each output whose `out_` port is in the XML.
- Handlers return `(ok, message)` or `(ok, message, {name: value})`; `ExtensionExecutor` copies it into the result.
- Manifest: `outputs: [text]` -> catalog exposes `out_text` as an optional port, validator accepts it, LLM node list shows it.
- Tests: executor round trip with a fake extension; validator accepts/rejects `out_` ports; a macro chains
  one node's output into `Speak`.
Rebuild order: `bt_pipeline_msgs` -> `bt_common` -> `bt_executor` -> extensions.

## Phase 1 - macro batch A, S (no new nodes)
Add the "now" macros of table A to `param/macros.yaml` with `example_args`. Limits to design around: no loops (write
`find_object` as a Fallback of "detect | rotate+detect" x4), params are strings, max depth 4. Count after: ~25 macros.
`test_macro_library.py` already checks each; extend `test_macros_only_name_places_that_exist` for new place params.
Also add an `acceptance` test per macro that the dry-run of the plan equals the expected node sequence.

## Phase 2 - HRIC / GPSR primitives (needs Phase 0), ordered by points per effort
| Node | Where it lives | Notes |
|---|---|---|
| WaitForTranscript(timeout, out_text) | `social_extension` (listens on `stt/transcript`) | **Conflict:** `transcript_to_task` turns every transcript into `bt/task`. Add a latched `stt/capture` flag (set by this node, honoured by `transcript_to_task`) so an answer to "what is your name" does not start a new task. Reuse the existing mute-while-speaking path. M |
| LookAtPerson | `pid_extension` next to `FollowTarget` | no head: rotate the base toward the person's bearing from `persons`; stop when centred. S-M |
| FollowTarget lost-target recovery | `pid_extension` | param `lost_timeout_sec`; spin-search, then fail with a clear message (HRIC penalises asking/waiting). M |
| DetectSound(kind, timeout) | new `sound_event_node` in `stt_pipeline` publishing `audio/events`; BT node in `social_extension` | doorbell = template/band-energy detector on `audio/raw`; needs real recordings to tune (B-23). M |
| DescribePerson(out_text) | `person_detection` adds clothing colour (+ glasses/hair if cheap) to the `persons` message (`person_detection_msgr`); handler in `pose_extension` | measured on synthetic frames only until real video. M |
| CountPeople / CountObjects(label, out_text) | `pose_extension` (already reads persons/objects) | S |
| FindFreeSeat(out_place) | `pose_extension`: chair/couch boxes (COCO labels) minus boxes overlapped by a person; bearing + depth from `person_depth_node` | needs depth camera; returns a pose usable by `NavigateToPose`. M |
| OpenDoor | `nav_extension` / base push first (no arm): drive through when the door is unlocked, announce otherwise | arm version later. M |
Then macros: B (ask_name_and_drink, introduce_guests_to_each_other, tell_guest_attribute, acknowledge_person, counts) and C
(wait_for_doorbell, receive_guest, offer_seat, follow_host_to, hric_round). `hric_round` is the HRIC scoresheet as one plan.

## Phase 3 - grammar B-03 (loops/foreach), L
Needed for "each object", "repeat until N", `clear_table_to`. Must change **together**: spec grammar + validator,
`bt_slm/fallback.py` (`build_xml`), `bt_slm/dataset.py` (`sample_steps`), and `prompt_for` (a prompt change means regenerate
data and retrain, see ARCHITECTURE). Until then macros write loops out by hand (fixed 12 objects = 12 `pick_and_place` calls).

## Phase 4 - arm (B-25 / T-11), L, gated on a hardware decision
Decide the arm model first (the macros, MoveIt config, grasp planner depend on it). Then in `arm_extension`:
GraspObject(label) (depth target from detection + `person_depth_node`-style depth), PlaceObject(place), HandOver(person),
OpenDoorHandle, PushPullRack, PourInto, OpenContainer, FoldCloth. Sim first (arm sim + the existing `nav_sim` style fake
objects); real arm only with a spotter and the velocity cap (0.1) kept. Then macros D and E, and update `llm_knowledge.md`.

## Phase 5 - acceptance rehearsals, M (after each phase)
Mirror `tools/integration/itest.sh`: isolated DDS domain, fake perception + fake LLM that returns the sheet's commands.
One script per task (`gpsr_test`, `hric_test`, later `pnpc_test`, `laundry_test`), each asserting the executed node sequence
and scoring it against `TASKS.md` lines (e.g. HRIC: both guests introduced = 4x30 + 2x50). Wire into `ci.yml`.

## Phase 6 - preflight and safety additions
Extend `preflight` per phase (arm configured, depth camera publishing, mic/VAD alive, stt/capture flag cleared) and keep
dry-run default, mux limits and the hardware e-stop note (B-04). New motion nodes (LookAtPerson, OpenDoor, arm) go through
`cmd_vel_mux` / the existing clamps; none may publish `cmd_vel` directly.

## Order and dependencies
```
P0 outputs ──┬─> P2 speech/people nodes ──> macros B,C ──> P5 gpsr/hric rehearsals
P1 macros A ─┘                                   │
P3 grammar (parallel, independent) ──────────────┘
arm decision ──> P4 arm nodes ──> macros D,E ──> P5 pnpc/laundry rehearsals
```
First deliverables: P1 (S) and P0 (M) can start immediately and in parallel; P2's WaitForTranscript is the first real
node because it unlocks the whole "ask the guest" part of HRIC and GPSR.

## Risks and open questions
- **Depth camera: decided, Intel RealSense D435** (`realsense2_camera` is already in `sensors.launch.py`; use aligned depth-to-colour, `person_depth_node` consumes it). D435 limits to design around: useful range about 0.3-3 m (nothing closer than ~0.3 m), so grasping needs the arm to look from >= 30 cm (or a wrist camera later); needs hand-eye calibration before any grasp; depth is noisy on shiny/dark/transparent items.
- **Arm model is undecided and is the big risk**: P4 and everything in tables D/E (PnPC 3515 + Laundry 4415 points) depend on it. The Kobuki base is the constraint (see the arm decision below).
- **Turn-taking** (WaitForTranscript vs `transcript_to_task`, wake word, mute while speaking) is the most fragile part of HRIC.
- **Prompt size**: ~40 nodes + ~60 macros grow the LLM prompt; check latency on the real model (B-21) and keep descriptions short.
- **SLM**: with a larger catalog the fine-tuned model is out of date; rules_first covers it, retrain after P3.
- Sheet text is a **2027 draft** (rev 2026-10-14); re-read the rulebook before locking the plans.

## Arm decision (open)
The base limits the choice more than the arm does: a Kobuki/TurtleBot2 carries about 5 kg and is low (about 0.4 m), while tables,
shelves, the dishwasher and the washing machine need reach at 0.5-1.0 m and the arm + mast + laptop + camera easily exceed
5 kg. Specs below are from memory, verify against datasheets before buying.
| Option | Gives | Costs |
|---|---|---|
| A. Keep Kobuki, no arm | GPSR (navigation/perception/speech parts), HRIC without handover/door, Final-demo partial | PnPC, Laundry, door, handover points are lost |
| B. Light arm (<= ~3 kg) on a mast, Kobuki kept | pick light items (cups, cutlery, bag), some handover | tiny payload, no dishwasher rack/door, no folding; weight budget is tight |
| C. New base with lift/mast (>= 15-20 kg payload) + 6 DoF arm of 3-5 kg payload (xArm 6 / Kinova Gen3 class, both have ROS 2 + MoveIt configs) | PnPC fully, door/dishwasher, folding at slow speed | money, new base driver and TF, re-tune Nav2 footprint |
Either way the software plan does not change: `arm_extension` keeps `MoveGroup` + `GripperCommand`, only the param set differs.

### Arm in hand (from photos 1.jpeg / 2.jpeg, 2026-11-14; model NOT confirmed)
Black serial arm of Dynamixel-style servos with a two-finger parallel gripper, about 40-45 cm long, plus a ROBOTIS **OpenCR 1.0**
board (Arduino-compatible, STM32F746). The supervisor named the DYNAMIXEL2Arduino library, which is the firmware route for OpenCR.
To confirm: arm model, number of joints, servo model (read the label: XM430 / XL430 / AX-12...). If it is OpenManipulator-X class:
about 4 DoF + gripper, ~0.4 kg payload, ~0.4 m reach (from memory). That means top-down grasps of light objects only, which is
option B above: no dishwasher door/rack, no folding, no full-size milk carton. Plan for it: PnPC subset (light items, cutlery),
HRIC bag handling only if the bag is light, and treat Laundry as out of reach until a bigger arm.
Two integration routes: (1) PC-side `ros2_control` Dynamixel hardware interface + MoveIt (OpenCR as USB-to-Dynamixel bridge or a
U2D2), least new code; (2) firmware on OpenCR with DYNAMIXEL2Arduino + a ROS link (micro-ROS or a serial protocol), more code
(kinematics/trajectory/hardware plugin) but matches the supervisor's suggestion. Ask the supervisor if (2) is required.

### Correction from the user (2026-11-14): the arm has 3 servos and a camera on top of the gripper
Wrist camera (model unknown) + about 3 joints + gripper. Consequences for P4:
- 3 joints cannot reach an arbitrary pose, so MoveIt/KDL with full 6D goals is the wrong tool. Use joint-space presets
  (`stow`, `look_down`, `grasp_low`, `carry`, `place_high`) through `MoveArmToPose`, or a small analytic planar IK
  (shoulder/elbow/wrist pitch), and let the **Kobuki yaw supply the missing base rotation**. Drop MoveIt for this arm.
- Grasping = visual servoing with the wrist camera: YOLO on the wrist image, centre the bbox by turning/driving the base,
  lower the arm to `grasp_low`, close the gripper. The D435 on the mast finds the object; the wrist camera finishes it
  (this also removes the D435 0.3 m minimum-range problem).
- Realistic scope: light, small objects from a table edge. No handover, dishwasher, door or folding. Max honest PnPC target
  is a few objects (about 12 x (10 + 50 + 40) = 1200 points is the ceiling, bonuses excluded).
Questions to the user: do the 3 servos include the gripper? what camera is on the gripper (USB webcam? resolution)?

### Update from teammate video VID-20261023-WA0048.mp4 (22 s, 4 frames viewed; supersedes the "3 servos" estimate above)
Arm sits on a table (not on the robot yet), powered from a LiPo, driven by an OpenCR (LEDs on) while it moves. Visible chain:
rotating base block -> shoulder -> elbow -> wrist servo(s) -> wide two-finger gripper. That is probably **4-5 servos plus the
gripper, with a base yaw**, i.e. OpenManipulator-X-like, not a 3-DoF arm: the Kobuki-yaw trick is not needed. On top of the gripper
is a black two-lens module that looks like a **stereo/depth camera** (model unknown; could be a RealSense or similar).
STILL UNCONFIRMED: exact servo count, servo model, camera model. Ask the teammate to read them off the hardware.

## STATUS 2026-11-14
- **Arm: everything above about the arm is UNCERTAIN** (user: "i totally not sure about it"). Servo count, model and camera are
  unconfirmed; the photos and the video conflict with each other. Do not design or build arm code until the hardware is read off
  and confirmed. Work on what exists now: P1 and P0.
- **P1 done:** 13 macros added to `param/macros.yaml` (return_home, look_around, find_person, find_object, go_find_object,
  go_find_person, go_find_and_say, meet_and_remember, follow_person, guide_to, escort_person_to, wait_for_wave, stop_and_wait);
  `test_macro_library.py` validates each against the real catalog. Total 22. Not tried against a real LLM or robot.
- **P0 done:** `ExecuteNode` result has `output_names/output_values`; extension handlers may return `(ok, msg, {"out_x": v})`; the C++ `ExtNode` registers `out_*` manifest ports as BT output ports and writes them to the blackboard when the XML names them. `tools/integration/outputs_test.py` (fake extension -> live bt_executor -> next node) passes locally and is in CI. In dry-run outputs are not produced (the downstream port is just omitted).
- **P2 step 1 done: `WaitForTranscript(timeout_ms, out_text)`** in `social_extension`; it sets the latched `stt/capture` flag so
  `transcript_to_task` keeps the answer out of `bt/task`. `tools/integration/listen_test.py` (4 checks, also in CI) passes locally.
  Macro `ask_and_repeat(question)` uses it. Still open: pulling a NAME out of the sentence ("my name is Alice" -> alice; `EnrollFace`
  accepts only A-Za-z0-9_-), so `ask_name_and_drink` needs a small extraction step (LLM or rule) before it can feed `EnrollFace`.
- **P2 step 2 done: `CountPeople(min_conf, out_count)` and `CountObjects(label, min_conf, out_count)`** in `pose_extension`
  (fail only when the detector is silent; 0 is a valid count). Pure helper tested; not run against live detections.
- **Host health service done (user request):** `cluster_msgs/srv/HostInfo`, served by each `cluster_agent` as `host_info/<host_id>`,
  plus `ros2 run cluster_core host_info [host_id]` (CPU, RAM, GPU util + VRAM, temp, battery). `NodeInfo` gained `battery_pct` and
  `charging`. Verified on this laptop: cpu 2% x12, ram 7.1G/15.0G, gpu 9% 0.4G/4.0G, 75C, battery 100% charging.
- Next in P2: LookAtPerson, FollowTarget recovery, DescribePerson, FindFreeSeat, DetectSound. Then: macro whitelist for the LLM/SLM
  (an allow-list in `macros.yaml`/launch so only listed macros are offered and accepted), P3 grammar, P5 rehearsals, P6 preflight.

## STATUS 2026-11-14 (later)
- **Macro whitelist done:** `whitelist:` at the top of `param/macros.yaml` (all 27 macros today). The validator's catalog, the JSON
  schema sent to the LLM and the orchestrator's `expand(..., allowed)` honour it; a test checks whitelist == defined macros.
- **Constrained decoding done** (see `RUNBOOK.md` "Local LLM + voice"): E2B can only emit whitelisted macros/nodes with real ports.
- **New:** `ExtractAnswer(kind=name|drink, text, out_value)` (rule-based, no model), macros `meet_guest(slot)`,
  `introduce_guest(slot)`, `look_at_person`, `acknowledge_person`, improved `follow_person` (lost-target retry via Fallback), and
  `ask_and_repeat`. `LookAtPerson` is NOT a node: it is `FollowTarget` without `approach` (macro `look_at_person`).
- **LLM measurements on E2B** (`tools/integration/llm_eval.py`, 10-15 commands): before schema 1/10 valid; schema + `--reasoning off`
  8/10; adding the macro-first prompt made E2B repeat `return_home` (looping); `repeat_penalty 1.1` and worked examples in
  `llm_knowledge.md` (guarded by a test that each example is a valid plan) are the fix being measured.
- Still open in P2: DescribePerson, FindFreeSeat, DetectSound (need real data/camera). P4 (arm) blocked on the hardware. P3 grammar,
  P5 rehearsal scripts, P6 preflight extensions, B-03..B-12 not started.

## STATUS 2026-11-14 (evening)
- **P3 slice done: `repeat: N` on groups** (spec grammar, `fallback.py` -> `Repeat num_cycles`, validator catalog, LLM schema + prompt,
  dataset sampler, macro `patrol_laps`). Verified in the real executor (dry-run: a repeated step ran twice). NOT done: `Parallel`,
  `SubTree`, blackboard "for each" loops. The SLM must be retrained to see `repeat` in training data (not yet).
- **P5 done for 4 cases:** `tools/integration/rehearsal.sh` (go_and_say, patrol_laps, meet_guest, refuse-to-bring) checks the exact
  node sequence through orchestrator -> expansion -> validator -> executor in dry-run; in CI. Not yet: HRIC door/doorbell/handover and
  PnPC/Laundry rehearsals (blocked on the missing nodes and the arm). `llm_eval.py` is the real-LLM counterpart (not in CI: needs a GPU model).
- **P2: `DetectSound(kind=doorbell, timeout_ms)` done** (`stt_pipeline/sound.py` + `sound_event_node` -> `audio/events`; BT node in
  `social_extension`; macro `wait_for_doorbell`). Verified: unit tests with synthetic tones plus negative controls (white noise, quiet
  tone, short blip, gliding pitch, one Piper speech clip), and a synthetic ding-dong WAV through `audio_capture` -> `sound_event_node`
  -> `audio/events` (883 Hz). NOT verified: real doorbell recordings, the BT handler through the executor, noisy rooms. Heuristic
  thresholds are guesses until real recordings exist (B-23).

## STATUS 2026-11-14 (final for this round)
- **36 macros**, all whitelisted (`docs/macros.md` is generated from `macros.yaml`). New since the last status: `wander`, `wander_until_person`, `scout_room`,
  `announce_and_go` (Parallel), `receive_guest`, `hric_round`, `offer_seat`, `wait_for_doorbell`, `patrol_laps` (repeat).
- **P3 done:** `repeat`, `Parallel`, `foreach` in the grammar (`docs/task-spec-grammar.md`); `SubTree` dropped because macros expand inline.
- **P5 done:** 9 rehearsals in `tools/integration/rehearsal.sh` (GPSR go_and_say/patrol_laps/refuse, HRIC meet_guest/receive_guest, walk_wander, foreach_tour,
  announce_and_go, scout_room), all in CI; the real-LLM counterpart is `llm_eval.py`.
- **P6 done:** preflight checks LLM, mux, executor dry-run, lidar/TF, clock offset (chrony) and disk.
- **Open:** `DescribePerson` (needs pixels), P4 arm (deferred by the user), HRIC door opening/handover, PnPC/Laundry rehearsals.
