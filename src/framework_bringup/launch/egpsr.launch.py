"""Stage II Enhanced GPSR (minutes 5.2), and the Final round (expected to follow this format): detect rubbish and bin it, spot a person who needs help (waving),
go to them, ask what they need, do it; 10 minutes, more than three tasks allowed, repeating a task is penalised.

Starts: speech, LLM + validator + executor (plan read aloud), objects, people + pose (waving), depth, arm + OAK, Nav2.
GAPS: nothing watches for a waving person while idle (macro wait_for_wave exists: the planner has to start it, and the spec grammar needs "when someone
waves"); rubbish is not a detector class; picking and the dustbin need the arm; no memory of tasks already done (repeat penalty); the Final's format is open until 3 Dec.

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
