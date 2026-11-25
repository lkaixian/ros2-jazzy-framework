"""Stage II Restaurant (minutes 5.3): new location outside the arena (maybe a real restaurant), two customers, receive and handle both orders.

Starts: speech, LLM pipeline, people + pose + depth (customer calls the robot), objects, lidar + RealSense, Nav2. No arm (enable_arm:=true if it carries items).
Plan: "restaurant round" -> macro restaurant_round: per customer find_waving_person, approach_person, take_order (asks, repeats it back, needs a yes, 3 tries),
return_home (the counter = place `start`), tell_order (speaks it for the barman). Macros validated, never run on a robot.
GAPS: the venue has no map (mapping slot, slam:= on navigation.launch.py) and `start` must be the counter there; the order is the whole heard sentence (no menu,
no item extraction, no check against what exists); fetching and serving the items needs the arm; both customers are handled one after the other (no queue, no
customer who calls from another table); waving customers rely on the T-13 geometry; unfamiliar room: lidar-only avoidance, depth obstacles off until the camera TF is measured.
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
        'enable_arm': 'false',
        'enable_oak': 'false',
        'persons_topic': 'persons_ranged'})
