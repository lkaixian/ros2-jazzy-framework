"""Stage II Give Me a Hand (minutes 5.4): the operator takes an object from a bag and offers it; the robot takes it from the hand WITHOUT touching the
hand, then is told where it goes (table or dustbin) and places it; single room, starts outside the arena.

Starts: speech, LLM pipeline, objects, people + pose + depth, arm + OAK, Nav2.
GAPS: handover is not built at all (hand detection, approach, gripper close on the object at a safe distance, no contact with the hand: needs a force/current
limit and the OAK depth); destination words (table / dustbin) need places; PlaceObject / pick exist but are unverified on a real object; no force sensor.

Any enable_* below can be overridden on the command line; dry_run stays true until you pass dry_run:=false. Notes: docs/robocup-tasks.md.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'true',
        'enable_person': 'true',
        'enable_pose': 'true',
        'enable_face': 'false',
        'enable_depth_ranging': 'true',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'true',
        'enable_oak': 'true',
        'persons_topic': 'persons_ranged'})
