# RoboCup@Home Education: making Carry My Luggage, Find My Mates and Receptionist viable

> **Update 2026-11-28 (same day):** steps 1 of section 5 are built: `edu_luggage` / `edu_mates` / `edu_reception` launch files, the spoken start signal, macros `luggage_round`, `mates_round` (+ `mates_visit`, `mates_report`),
> the Receptionist door wait (`DetectPerson max_distance`), and the speech vocabulary (`param/vocab.yaml`, snapping, `kind=age`). Answers from the user: the start signal is a phrase we say ("start the task");
> "description" in Find My Mates means what the robot asks and sees at each *checkpoint* (name, age, how they look) and reports with the location back at the operator; the bag is a backpack or a grocery bag with handles;
> the arena is expected to match github.com/robocupmalaysia/at-home-2025/tree/main/maps (known only from 2 Dec); the luggage area replaces the car; the event is 3-7 Dec on the same score sheet.
> `reception_round` was dropped (macros may nest 4 deep): the Receptionist runs `hric_round` directly. Open: sections 3 (C2-C8) below.
> **Later the same day:** C2 (`PeopleBook`, `FindPerson who=`, `look_at_guest`), C6 (`DescribeUnique`, `DescribeLocation`, `describe_guest`, `mates_report` against everyone seen), the guest-follows check
> (`CheckBehind` in `guide_to`), seat facing (`FindFreeSeat out_radians`), C3 as an opt-in (`FollowTarget trail=true`, on in `follow_until_told`; sim-tested) and `PointedObject` (C7 tier 1: which bag) are built and unit-tested.
> The validator's `max_nodes` went 160 -> 260 because `hric_round` is now about 210 nodes. Still open: C4 (depth obstacles need the camera tilt measured; crowd manners), gaze while listening, a bag grasp (C7 tier 2),
> retrace/queue, age and gender models, and every real-robot, real-microphone and real-speaker test.

Written 2026-11-28. Sources: the Education rules PDF (robocupmalaysia/at-home-2025, titled "Rules 2024", which defers the task text to the
2022 @Home rulebook) and sections 5.1, 5.4, 5.6 of the official 2022 rulebook (athome.robocup.org/wp-content/uploads/2022_rulebook.pdf, revision
2022-06-30). Code facts were read from this repository on the same day. Everything below about the real robot is a plan, not a result: see
`docs/status.md` for what has run on hardware. Items marked **(unverified)** are my reading, not something I tested.

## 1. What the rules ask for (condensed, with the points)

| Task | Main goal | Score items (2022 sheet; the Education sheet splits them finer) | Time |
|---|---|---|---|
| **Carry My Luggage** (navigation) | carry a bag to a car parked outside, following the operator | pick the correct bag 100; follow the operator to the car 300; avoid crowd / small object / hard-to-see object / barrier area 50 each; bonus re-enter the arena 100, join the queue 300; dropping the bag -50; losing the operator -50 / -100 / -200 by how it is found again | 5 min |
| **Find My Mates** (vision) | find and report the description and location of 2 guests (3rd is a bonus); the operator knows only their names | location 2x100, a unique location feature 2x50, description 2x150; bonus 3rd guest 150 + 250; Education adds the correct name (50 per guest); penalties for being waved at / told where they are (-75) / walking up to the robot (-150); **points only count when the robot goes back to the operator and reports in time** | 5 min |
| **Receptionist** (speech) | take two new guests to the living room, introduce them and offer a free seat, with correct gaze | per guest 250 (name, drink, empty seat, pointing/facing the seat); look at the person talking / being described / the direction of travel 50 each; wrong name or drink -50 each; bonus: open the door 2x100, describe guest 1 to guest 2 (at least 4 characteristics) 150; asking for help to identify people -200 per guest | - |

Rules that shape the design:
- **Carry My Luggage:** the operator *points* at one of at least two bags within 2 m. The robot must tell the operator it is ready to follow. The car location is
  unknown, so it can only be reached by following. Four obstacles come in any order: a small object on the floor, a hard-to-see object, a crowd of 3-4
  people, an area closed with retractable barriers. The queue is made of the same people that formed the crowd.
- **Find My Mates:** a description must be *unique* among the 4 guests (clothing, age, height, complexion, hair, gender); the location must be relative to
  the surroundings ("sitting on the couch", "beside the lamp"). Guests stay where they are.
- **Receptionist:** host name and drink are announced 2 h before. Guests arrive one at a time and may swap seats after being seated. Not understanding and asking
  again is free; continuing with a wrong name or drink costs 50. The door is open by default.
- **Education extras:** incremental scoring, a **skip rule** (move on, never retry the same subgoal) and a **simplify rule** (a team member in a predefined
  coloured shirt, or a self-supplied object, at 50% of the points). Open Platform objects are placed within the arm's working envelope.
- Not in the PDF (ask the organisers): the start signal (section 3.4.4), what "description 1" means on the Education sheet, whether a hand-over of the bag
  counts as picking it up, and the dates of the Education event versus the Malaysia Open (3-7 Dec per the 25 Nov minutes).

Our robot: Kobuki base, RPLIDAR (mounted facing backwards: `scan_yaw_offset` = pi), RealSense D435 on the base at about 0.8 m (colour 640x480 at 15 fps),
OpenMANIPULATOR-X (4 joints, gripper opens to about 72 mm) at about 0.9 m, no head (the base turns to look at someone), laptop with an RTX 3080.

## 2. Where we stand, item by item

"Built" means code exists and is tested in simulation or unit tests; nothing below has been scored on the real robot.

| Item | What exists | What is missing |
|---|---|---|
| Follow the operator (300) | `FollowTarget` (lock, approach, until_cancelled), lost-person search, `follow_until_told`, `help_me_carry`; stable ids from `IdentityBook` (re-identification) and the learned `TargetModel`, tried on the real camera with 3 people but not while driving | follow the operator's *path* (it drives straight at them and cuts corners); real-base drive test; crowd behaviour; the "I am ready, please walk" announcement exists in `follow_until_told` |
| Four obstacles (4x50) | lidar `ScanGuard` stop and turn-to-freer-side; depth obstacles exist but are off (camera tilt not measured) | small object below the lidar plane; a glass/dark object; a crowd (it should wait, say "excuse me", then continue); retractable-barrier belts (thin, may not show on a lidar plane) **(unverified)** |
| Pick the pointed bag (100) | `IsPersonPointing` (left/right in the image), `DetectObject`, guarded arm, one real pick of a bottle on 27 Nov, `give_me_a_hand_round` hand-over | choosing which bag is pointed at; grasping a bag handle; carrying it while driving; releasing it to the operator at the car |
| Re-enter, queue (400 bonus) | `return_home` (Nav2 on the arena map) | no map outside the arena, no breadcrumb retrace, nothing joins a queue |
| Detect guests, stand in front (Mates) | `find_person`, `approach_person`, `FollowTarget`, `BackUpForFace` (new, untested on the base) | a sweep of the room that remembers every person it saw |
| Name of a guest (Mates, Edu) | `register_person` / `greet_or_register` (ask, repeat back, enrol, greet) | name list and fuzzy matching: `extract_name` is regex only, no vocabulary |
| Description of a guest (Mates 2x150, Receptionist bonus) | `DescribePerson` (top colour and now legs colour), `DescribeAttributes` (glasses, hair colour, hat, earrings from a face-parsing model), `describe_features` (new) | **uniqueness among the 4 guests**; age and gender; the face-parsing model file must be present (`models/face_parsing/`) |
| Location of a guest (Mates 2x100 + 2x50) | named places in `places.yaml`, `FindFreeSeat` (seat objects), persons carry `distance_m` | a person position in the map frame; a relative phrase ("on the couch", "near the kitchen table"); persons are in the camera frame only today |
| Report back to the operator (Mates) | `go_to_place start`, `Speak` | remembering the operator's position and the 5-minute budget |
| Doorbell / guest arrival (Receptionist) | `wait_for_doorbell` (`DetectSound`), `receive_guest` | a guest who just steps in front of the robot (rules allow both): `receive_guest` fails if no bell rings within 60 s |
| Name and drink (Receptionist) | `meet_guest` (asks twice, regex extraction, `EnrollFace`) | vocabulary biasing and fuzzy matching; the host's name and drink as parameters; no confirmation when the answer is clean (time) |
| Lead the guest to the living room | `guide_to` ("please follow me", drive, "we have arrived") | checking the guest actually follows (it does not look back); `escort_person_to` leaves the guest behind if they stop |
| Introduce guest to host and others | `introduce_to`, `hric_round` | looking at the *right* person (it uses the nearest person, not the named one); guests may swap seats: identify people by name, not position |
| Free seat + point/face it (100 + 50) | `FindFreeSeat` -> `offer_seat` says "Please sit on the chair on my left" | face the seat (no angle output: only an image offset), or point with the arm; the sits-on-it check is by image boxes only |
| Gaze (3x50, -50 penalties) | `look_at_person` (turns the base, 5 s); driving forward is already "looking where it goes" | look at the speaker *while listening* (it does not run in parallel with `WaitForTranscript`); alternate gaze between two people while describing |
| Describe guest 1 to guest 2 (150) | `tell_look` (speaks the stored look) | at least 4 characteristics: today top colour or glasses/hair, not four |
| Open the door (2x100) | `receive_guest` asks a person to open it | not feasible with a 4-joint arm: skip (door is open by default) |

## 3. Cross-cutting changes (build once, used by all three tasks)

**C1. Speech that survives a noisy room (Receptionist and Mates; the largest single risk).** Nothing has run on a real microphone.
- Give the recogniser the likely words: `stt_pipeline/stt_node.py` already passes a Whisper `initial_prompt` built from the place names (`vocab.hint_prompt(place_phrases())`, param `hint_places`); extend `vocab.py` to add the announced host name, a name list and a drink list.
- Add `social_extension/vocab.py`: a name list file and a drink list file under `framework_bringup/param/`; `ExtractAnswer kind=name|drink` snaps the heard word to the
  closest list entry (edit distance plus a phonetic key) when it is close enough, otherwise keeps what was heard and asks again once. A wrong name costs 50, asking again costs nothing.
- Keep the yes/no confirmation only when the match was not an exact list hit (saves about 15 s per guest).
- Test with `tools/integration/phrase_test.py` style synthetic transcripts, then with real recorded speech before the event.

**C2. A "who is who" book (Receptionist and Mates).** The detector now publishes stable person ids and the face node publishes named faces; nothing joins them.
- New small node (or a section of `pose_extension`): when a face box with a gallery identity lies inside a person box (face centre inside the person box,
  persons and faces share the image), record `name -> person id` and the person's attributes (top colour, legs colour, hair colour, glasses, sitting or standing from the
  keypoints, last map position). Persist it on the blackboard through `out_` ports.
- New BT node `LookAtPerson name=...` (turn the base toward the person whose book entry matches; fall back to the face recogniser's name when the id changed).
  This fixes "look at the right guest" and "guests switch seats" without asking for help (-200).
- The same book feeds C6 (unique description) and C3 (the operator's id).

**C3. Follow the operator's path, not just the operator (Carry My Luggage, 300 + 200).**
- Today the robot drives straight at the person's current position. Add a breadcrumb trail to `FollowTarget`: record the operator's odom position every 0.25-0.3 m
  (depth + odom pose are already used in `Memory.update`), steer to the oldest breadcrumb that is about 1 m ahead, and stop dropping crumbs when the operator is out of view.
  This keeps the robot on the route the operator walked (around barriers, through the door) and gives the lost-operator search a path to retrace instead of one point.
- Use the stable id and `TargetModel` score to decide who is the operator; a second person in front of the operator must not become the target (the existing 5-frame vote and
  the colour/size check cover most cases **(unverified on a moving robot)**).
- Pure logic with unit tests (`pid_extension/trail.py`), then `follow_sim_test.py` with a path that bends around an obstacle.

**C4. Obstacles the lidar plane misses (Carry My Luggage, 200).**
- Measure the RealSense tilt with `tools/camera/floor_plane.py` (notes say it is hand-measured, tilt unknown), then switch on `depth_obstacles:=true` with a spotter. This is the
  only route to the small floor object and the hard-to-see object.
- Crowd: when the guard stops the robot with people in front, say "Excuse me, may I pass?", wait, and only then turn to the freer side. The operator passes first, so the
  trail (C3) shows where a gap is. Add a `blocked_by_people` flag to `ScanGuard` consumers if the persons topic shows people at that range.
- Retractable barriers are thin belts: expect the lidar to miss them **(unverified)**; the trail keeps the robot on the operator's side of them, and the depth camera helps at the posts.
- Keep every limit in `cmd_vel_mux`; do not raise speeds to make up time.

**C5. One launch file and one macro per Education task, started without the LLM.** The planner path adds latency and a failure mode we do not need for three known tasks.
- `edu_luggage.launch.py`, `edu_mates.launch.py`, `edu_reception.launch.py` built with `task_launch` like the existing ones (switches: audio, persons + depth + pose, faces, objects, Nav2,
  arm only for luggage if the bag is attempted). Macros `luggage_round`, `mates_round`, `reception_round` in `macros.yaml`; started by publishing the task text on `bt/task` or
  by a one-line command, still passing through the validator and `cmd_vel_mux`.
- Parameters for the day: host name/drink, operator wait spot, `places.yaml` poses from the venue map.
- A 5-minute watchdog macro wrapper that makes the robot go and report (Mates) or return (Luggage) when the time is nearly out, so partial scores are kept.
- `dry_run` stays true by default; the real run is `dry_run:=false` with the operator in control of the mux.

**C6. Describe a person so it is unique and say where they are (Mates, Receptionist bonus).**
- `unique_description(book, person_id)` (pure function, `pose_extension/describe.py`): rank attributes (top colour, legs colour, hair colour, glasses, hat, sitting/standing, plus age and gender
  if available) by how many other known people share them, and say the fewest attributes that no other person shares, at least two, up to four for the Receptionist bonus. If no
  combination is unique, say so by adding a location feature rather than guessing.
- To get the other people, the robot must have *seen* them: add a `survey_room` macro (turn about 360 degrees in 4 stops, running `DescribePerson` + `DescribeAttributes` into the book) before reporting.
  With 4 people in a room it must see all of them, or "unique" is a guess.
- `DescribeLocation` node: map-frame position of a person = robot pose (TF map -> base) + bearing from the box + `distance_m`; the phrase is "sitting on the <seat>" when a seat object overlaps
  the person box and the keypoints say sitting, "near the <object>" for the nearest detected object, else the nearest named place in `places.yaml` ("near the coffee table"). Only
  phrases backed by a detection or place are spoken. COCO has couch, chair, bench, tv, potted plant, bed, dining table; no lamp.
- Age and gender: no model is wired in. A small face-attribute model (e.g. an InsightFace genderage ONNX) could be added behind `DescribeAttributes`. The 2022 score sheet has no penalty
  for a wrong description, only a bonus for naming 4 characteristics, so a moderately accurate guess costs little **(my reading of the sheet; confirm with the organisers)**.

**C7. The bag (Carry My Luggage, 100 + the drop penalty).** Three tiers; pick by what the arm can do on the day.
- **Tier 0 (skip rule):** skip the bag, go straight to following. Keeps the 300 + 200 points that do not depend on the arm.
- **Tier 1 (simplify rule, hand-over):** ask the operator to hand the bag over with the existing hand-over flow (`give_me_a_hand_round`: open, wait "ready", close on a stall), hold it folded
  in the rest pose, hand it back at the car on "thanks". 50% of the pick points under the simplify rule.
- **Tier 2 (grasp it):** `PointedObject` (new, `pose_extension`): the pointing ray from elbow to wrist in the image, compared with the suitcase / handbag / backpack boxes
  (COCO classes) to choose the bag; then `PickObject` using the RealSense as the target source (`target_source: base_camera`, already how the 27 Nov pick worked). Limits: the gripper opens to about 72 mm,
  so the bag needs a thin handle at a reachable height; the payload of this arm is small. Ask the organisers for the bag type before investing.

**C8. Tests and the hardware ladder.**
- Unit tests for every new pure function (trail, vocabulary snap, unique description, location phrase, pointed object, book association).
- Simulation: extend `fake_perception.py` / `nav_sim` actors to several people with names so `mates_round` and `reception_round` run through the real executor like
  `arm_task_test.py` does; `follow_sim_test.py` gets a bending path and a crowd.
- Hardware ladder (see `references/testing.md` levels): L2 unit and sim -> L3 stack running on the real robot with the mux killed -> L4 each node on real people with the base stationary -> L5 base
  moving slowly with a spotter and a hand on the stop -> L6 a timed full run. No step is skipped; a failed step is fixed at its own level.

## 4. The three tasks as behaviour trees (all inside the current macro engine)

**Receptionist** (`reception_round`, replaces `hric_round`; host and drink are parameters)
1. `wait for guest`: Fallback [`DetectSound doorbell` (short timeout) | `find_person` at the door] -> say "Welcome".
2. `look at guest` -> `greet_or_register`-style intake with C1: ask the name, ask the drink, snap to the lists, enrol the face; record guest in the book (C2) with attributes.
3. `lead`: "Please follow me", drive to `living_room`, and check the guest follows (rear lidar sector for a person-sized blob within 2 m; if not, wait up to 10 s and say "please follow me").
4. `introduce`: `LookAtPerson host`, say host name + guest name + drink; for guest 2 also `LookAtPerson guest1` and say guest 1's name + drink, with `Parallel` gaze while speaking.
5. `seat`: `FindFreeSeat` -> turn to the seat bearing (needs `out_radians`, add to `FindFreeSeat`), say "Please sit on the <seat>".
6. After guest 2: describe guest 1 to guest 2 with `unique_description` (4 characteristics) while `LookAtPerson guest1` (looks at the person being described).
7. Skip rules: if intake fails twice, move on with what was heard; never continue with an unconfirmed name that the list could not match (-50).

**Find My Mates** (`mates_round`)
1. Operator gives the task; say "I will find your mates". Record the operator's place (current pose).
2. `survey_room`: go to the room place, four 90-degree stops, record every person and attribute (C2, C6).
3. For two guests (then a third if time remains): `approach_person` -> `BackUpForFace` -> ask their name (C1) -> `describe_features` into the book -> `DescribeLocation`.
4. Go to the operator place (`go_to_place` with the stored pose) and report: "<name> is <unique description>, <location>" for each guest. Time budget: survey 60 s, 80 s per guest, 40 s report.
5. Deus ex machina avoidance: the robot never asks guests to wave or come to it; if nobody is found it widens the sweep.

**Carry My Luggage** (`luggage_round`)
1. Ask "which bag?" is not allowed to cost points: use `IsPersonPointing` and `PointedObject` (C7), say "I will take the <colour> bag" (the colour from the box crop).
2. Bag per tier (C7), say "I am ready, please walk to the car" (the rule asks for this).
3. `follow_until_told` with the trail (C3) and depth obstacles (C4) until the operator says "stop"/"we are here"; a lost operator is searched along the trail and by `TargetModel`.
4. At the car: hand the bag back on request (tier 1) or release it; say "Here is your bag".
5. Bonus only if time remains: retrace the trail to the door (odom), then Nav2 on the map to `start`; the queue is not attempted.

## 5. Order of work and what to cut (the event is days away; the day-by-day depends on the real dates)

| Order | Work | Why first | Cost (rough) |
|---|---|---|---|
| 1 | C5 launch files + macros skeletons; C1 vocabulary | Receptionist and Mates both depend on speech; the launch files make every later test one command | 1 day |
| 2 | C2 who-is-who book + `LookAtPerson`; `FindFreeSeat out_radians`; guest-follows check; no-bell guest arrival | Closes the Receptionist gaps (the most complete task, 1000 points) | 1 day |
| 3 | C6 unique description + `DescribeLocation` + `survey_room` + `mates_round` | New task; mostly glue over existing nodes | 1.5 days |
| 4 | C3 trail following + C4 depth obstacles and crowd manners; real-base drive test | Carry My Luggage points (500) need only the base, no arm | 1.5 days + test time |
| 5 | C7 tier 1 hand-over, then tier 2 only if the bag allows | 100 points and a penalty; the arm is the least proven part | stretch |
| cut | opening the door, queue joining, age/gender model unless time remains | no arm path; 300 bonus with an unknown route; low penalty either way | - |

Go / no-go gates: after step 1 the robot must say the plan and hear a name from a real mic; after step 4 it must follow a person for 20 m on the real base without a touch on the stop.

## 6. Risks and open questions
1. **Real microphone and speaker have never been tested** (docs/status.md). Everything in Receptionist and Mates names rides on it.
2. **Camera framing:** at 0.8 m the camera sees torsos up close; `BackUpForFace` reverses the base using the rear lidar, untested on hardware.
3. **Identity quality:** the re-identification works on 3 people in one room; two similarly dressed people (the crowd, the queue) are not told apart. Do not let it pick the operator without a distinct feature check.
4. **Time:** 5-minute limits per task and 15 fps colour. Measure the real latencies (STT, TTS, the face and parsing models) before fixing the budgets above.
5. **Not in the PDF:** start signal, the Education sheet's description line items, bag type, door and arena layout, dates. Please ask the organisers or share the Education score sheets.
6. **Platform:** the 2022 rules assume a robot that can pick a bag from the floor; this arm cannot. The skip and simplify rules are the legal route; confirm how the referees will count them.
