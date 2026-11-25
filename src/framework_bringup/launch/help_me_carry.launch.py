"""RoboCup@Home Malaysia 2026 Stage I, Help Me Carry (minutes 4.2; no 2027 score sheet): follow the operator inside the arena with obstacle avoidance, follow
on to a randomly chosen destination outside, two bags; bonus for navigating back to the starting point.

Starts: person detection + depth ranging, lidar + RealSense, Nav2, audio. No arm, no objects, no faces. max_runtime_sec 900 (a follow outlasts
the default 300 s run limit).
Plan: task "help me carry" / "follow me until I say stop" -> macro help_me_carry: say, follow_until_told (FollowTarget until_cancelled + lock,
in a Parallel with WaitForPhrase "stop / we are here / we have arrived"; a lost person is searched for 3 times), say, return_home.
Built: keeps following after arriving; locked to the first person (track id); an obstacle ahead makes it turn toward the freer side (reactive, no
path planning); the spoken stop ends the follow; then it drives to place `start`. Tested in simulators only (tools/integration/follow_test.py,
phrase_test.py); the macros are only validated, never run on a robot.
GAPS: `start` in places.yaml must be a real pose of the arena start; the way back is Nav2 on the arena map: outside it the robot is probably not
localised and nothing retraces the path (no breadcrumbs); the turn-away avoidance can dither at doors and does not plan a detour; a lost track id
(occlusion, tracker reset) loses the lock and re-locks onto the biggest person; stop words are not heard over robot noise / in a crowd; nothing carries
the bags (the operator does, or ask on 3 Dec); depth obstacles off until the camera TF is measured (the lidar sees one plane).
Any enable_* below can be overridden on the command line; dry_run stays true until you pass dry_run:=false. Notes: docs/robocup-tasks.md.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'false',
        'enable_person': 'true',
        'enable_pose': 'false',
        'enable_face': 'false',
        'enable_depth_ranging': 'true',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'false',
        'enable_oak': 'false',
        'persons_topic': 'persons_ranged',
        'max_runtime_sec': '900'})
