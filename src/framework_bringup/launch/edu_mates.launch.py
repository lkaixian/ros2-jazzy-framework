"""RoboCup@Home Education 2025/26, Find My Mates (vision, 5 min; notes/EDU_PLAN.md): visit checkpoints in the living room, find a guest at each, walk up, ask their
name and age, remember how they look, then go back to the operator and report each guest with where they are and how they look (points only when it reports).

  ros2 launch framework_bringup edu_mates.launch.py                # robot idle; add dry_run:=false when it may move
  say "start the task"                                             # the only thing that starts it
  ros2 launch ... start_spec:='{"goal":"edu mates task","steps":[{"macro":"mates_round","args":{"cp1":"couch","cp1_words":"on the couch"}}]}'   # other checkpoints

Starts: audio, faces (names, attributes), objects, person + pose + depth, Nav2, sensors, social extension. No arm. The checkpoint names are places in param/places.yaml
(cp1 / cp2 / cp3 and their spoken phrase cp1_words ...): the placeholders there are NOT the venue's; set them from the real map on the preparation day, and `start`
must be where the operator stands. Spoken progress ("I am finding your mates now", "I am scanning for people here") is part of the plan on purpose.
Plan: macro mates_round = for each checkpoint mates_visit (go, find a person, approach 1 m, ask name, ask age, back away until the face shows, remember the look,
enrol the face) under a Fallback so an empty checkpoint is skipped, go to `start`, then mates_report per guest. The third checkpoint is the bonus guest.
Built since: the pose extension keeps a book of who it has seen (stable ids), so at the end each guest is described with the fewest features that nobody else it has
seen shares (colour of top and bottoms, hair, glasses, sitting or standing: only what was seen at least twice), against everyone seen, and the location is measured from
detected objects ("sitting on the couch", "next to the tv") when there is one, besides the checkpoint's phrase.
GAPS: nobody checks the guest is real (a poster would be approached); only COCO objects name a location; age and gender are only what the guest says (age asked, gender not);
5 minutes is tight with three visits. Never run on a robot.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true',
        'announce_plans': 'false',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'true',
        'enable_person': 'true',
        'enable_pose': 'true',
        'enable_face': 'true',
        'enable_depth_ranging': 'true',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'false',
        'enable_oak': 'false',
        'persons_topic': 'persons_ranged',
        'max_runtime_sec': '360',
        'start_phrase': 'start the task,start task',
        'start_spec': '{"goal":"edu mates task","steps":[{"macro":"mates_round","args":{}}]}',
    })
