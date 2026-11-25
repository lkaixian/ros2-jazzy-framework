"""Stage II Restaurant (minutes 5.3): new location outside the arena (maybe a real restaurant), two customers, receive and handle both orders.

Starts: speech, LLM pipeline, people + pose + depth (customer calls the robot), objects, lidar + RealSense, Nav2. No arm (enable_arm:=true if it carries items).
GAPS: the venue has no map: needs a mapping slot (slam:= on navigation.launch.py); taking an order is only the generic ask/remember macros (no order
slot, no menu, nothing reads the order back or checks it); fetching and serving items needs the arm; waving customers need wait_for_wave; unfamiliar
environment: lidar-only obstacle avoidance, depth obstacles off until the camera TF is measured.

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
