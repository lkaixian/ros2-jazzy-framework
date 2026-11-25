"""Stage II Enhanced GPSR (minutes 5.2), and the Final round (expected to follow this format): detect rubbish and bin it, spot a person who needs help (waving),
go to them, ask what they need, do it; 10 minutes, more than three tasks allowed, repeating a task is penalised.

Starts: speech, LLM + validator + executor (plan read aloud), objects, people + pose (waving), depth, arm + OAK, Nav2.
Plan: "bin rubbish and help whoever needs it" -> macro egpsr_round = dispose_rubbish (NextObject task=rubbish -> pick -> dustbin -> put down) then
assist_waving_person (find_waving_person, approach, "How can I help you?"; the answer is heard as the next task by the normal speech -> plan -> run path).
Needs the ARM running first (tools/arm/arm_up.sh) and the arm eye detecting (tools/arm/arm_eye.sh -> /arm_eye/objects). arm_extension does nothing live
until `live: true` (extensions.yaml) AND dry_run:=false; arm places (table, shelf, bin, ...) are "name:x,y,z" in its `places` parameter, still empty.
Objects -> destinations: param/object_categories.yaml (placeholders, COCO labels; the node NextObject). tools/integration/arm_task_test.py runs the plans
against fake extensions (order of calls, values passed on, loops, failures): no arm, no robot, no real object has been touched.
GAPS: rubbish is a guess from COCO labels (bottle, cup, banana, sandwich), real rubbish needs a detector class (3 Dec); the arm is unproven on real objects; nothing watches for
waving while another task runs; waving is hand-tuned geometry on synthetic skeletons (T-13); no memory of done tasks (repeat penalty); 10 min and more than three tasks
are allowed, the round does one rubbish item and one person; the Final's format is open until 3 Dec.
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
