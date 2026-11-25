"""Stage II Enhanced GPSR (minutes 5.2), and the Final round (expected to follow this format): detect rubbish and bin it, spot a person who needs help (waving),
go to them, ask what they need, do it; 10 minutes, more than three tasks allowed, repeating a task is penalised.

Starts: speech, LLM + validator + executor (plan read aloud), objects, people + pose (waving), depth, arm + OAK, Nav2.
Plan: "help the person who waves" -> macro assist_waving_person (find_waving_person: turns on the spot, ~2 turns, until IsPersonWaving; approach_person; asks how it
can help; the answer is heard as the next task by the normal speech -> plan -> run path). Macros validated, never run on a robot.
GAPS: it only looks while that macro runs (nothing watches for waving while another task runs, and the macro starts only when a task asks for it, e.g. the
operator says "look for someone who needs help"); waving is hand-tuned geometry tested on synthetic skeletons (T-13); rubbish is not a detector class and
binning it needs the arm; no memory of tasks already done (repeat penalty); the Final's format is open until 3 Dec.
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
