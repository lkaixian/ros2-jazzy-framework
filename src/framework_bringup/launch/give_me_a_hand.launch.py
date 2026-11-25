"""Stage II Give Me a Hand (minutes 5.4): the operator takes an object from a bag and offers it; the robot takes it from the hand WITHOUT touching the
hand, then is told where it goes (table or dustbin) and places it; single room, starts outside the arena.

Starts: speech, LLM pipeline, objects, people + pose + depth, arm + OAK, Nav2.
Plan: "take this from my hand" -> macro give_me_a_hand_round: say what to do, open the gripper and move to the handover point (x y z, ESTIMATE, teach it),
wait for the person to say "ready", close (succeeds only if the gripper stalled on an object), ask where it goes (table, shelf or bin; ExtractAnswer
kind=place), drive there, put it down. The PERSON decides when the gripper closes: the robot never closes on a hand by itself.
Needs the ARM running first (tools/arm/arm_up.sh) and the arm eye detecting (tools/arm/arm_eye.sh -> /arm_eye/objects). arm_extension does nothing live
until `live: true` (extensions.yaml) AND dry_run:=false; arm places (table, shelf, bin, ...) are "name:x,y,z" in its `places` parameter, still empty.
Objects -> destinations: param/object_categories.yaml (placeholders, COCO labels; the node NextObject). tools/integration/arm_task_test.py runs the plans
against fake extensions (order of calls, values passed on, loops, failures): no arm, no robot, no real object has been touched.
GAPS: with that voice cue the person puts the object between the fingers and the gripper may touch the hand: the "no contact with the hand" rule is NOT met (needs hand
detection and depth at the gripper; not built); the handover point is a guess (the arm refuses it outside the taught range); no check that the object is
really in the gripper beyond the stall; table / shelf / bin must exist as places in places.yaml AND as arm places; nothing but the voice triggers the grasp.
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
