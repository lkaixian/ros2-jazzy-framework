# What is still missing (outside the arm and the models) - 2026-11-27

Estimates from reading the code and docs, not measurements. Nothing here has run on the real robot except the e-stop chain and sensor enumeration.

## Three blockers
1. **Navigation on the real robot**: lidar `/scan` (spins, publishes nothing), a supervised drive test, a real map and places, then the items below.
2. **Voice in and out**: USB audio adapter for the Comica mic, real speech and speaker tests in a noisy room, face registration with real people.
3. **Full-stack real run**: `dry_run:=false` from boot, a pre-run self-test, a hardware (power-cut) e-stop, battery thresholds, venue Wi-Fi.

## Navigation gaps: state after the 2026-11-27 sim work (docs/navigation.md; tests in tools/integration/)
| Gap | Done in the simulator | Still open (needs the robot or is a known limit) |
|---|---|---|
| Person following | `FollowTarget`: lost person -> drive to last seen, sweep, re-id by colour + position, ease off near things. `follow_sim_test.py`, `help_me_carry_sim_test.py` (whole BT path) | reactive only: no walls-aware route, no motion prediction, similar clothes not told apart; real detector track ids, real walking speed |
| Dynamic obstacles | `nav_dynamic_test.py crossing`, `door_crowd` (walkers, crowd in a doorway) | no velocity estimate of people; real lidar noise and real people |
| Depth obstacles | `overhead` (avoided) vs `overhead_off` (touched: the control) | camera tilt unmeasured (tools/camera/floor_plane.py), left/right of the arm, ghost points, CPU; off by default |
| Recovery behaviours | patient Nav2 tree, no BackUp; goal heading now honoured (`rotate_to_goal_heading`) | Spin on the real robot; the BT layer's RecoveryPolicy table untuned |
| Robot radius | - | `robot_radius` 0.18 vs `scan_mask` 0.27: measure the widest layer |

Bugs the simulator found: places with a heading never finished (shim `rotate_to_goal_heading: false`); `pid_extension` `max_linear` 0.15 lost a person walking at 0.2 m/s; Nav2's default tree aborted a blocked doorway after 19 s.

## Other
- Arena `places.yaml` are placeholders until the venue map on 3 Dec; rehearse the map workflow on a room now.
- Rule changes on 3 Dec not checked; score sheets in `TASKS.md` are 2027 drafts.
- `docs/robocup-tasks.md` updated 2026-11-27 (depth obstacles: overlay exists, off by default).

## Estimated progress per task (software built / proved on robot)
Inspection 85/10, GPSR 75/10, Help Me Carry 55/5, Receptionist 55/5, Storing Groceries 50/10, Clean the Table 50/10, EGPSR 50/5, Restaurant 55/5, Give Me a Hand 40/5.
