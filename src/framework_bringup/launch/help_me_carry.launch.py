"""RoboCup@Home Malaysia 2026 Stage I, Help Me Carry (minutes 4.2; no 2027 score sheet): follow the operator inside the arena with obstacle avoidance, follow
on to a randomly chosen destination outside, two bags; bonus for navigating back to the starting point.

Starts: person detection + depth ranging (FollowTarget keeps a real distance), lidar + RealSense, Nav2 (obstacle avoidance, return home), audio (spoken
commands). No arm, no objects, no faces.
GAPS: a spoken "stop"/"we arrived" is not wired to stop following and go home (say it to the planner: "follow me", then "return home");
`start` in places.yaml must be a real pose of the arena start; the outside destination is off the map (no map outside the arena: mapping slot on 3 Dec);
following through a doorway / lost-person recovery is untested; depth obstacles are off until the camera TF is measured (lidar only sees one plane);
carrying a bag: nothing on the robot holds one (the operator carries, or it is not required: ask on 3 Dec).

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
        'persons_topic': 'persons_ranged'})
