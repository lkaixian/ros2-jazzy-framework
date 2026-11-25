"""Everything on, for any task or for trying the whole robot: audio, lidar + RealSense + OAK, Nav2, objects / people / faces / pose with depth,
the arm. Switch parts off on the command line (enable_arm:=false ...). Per task: gpsr, receptionist, help_me_carry, storing_groceries, clean_the_table,
egpsr, restaurant, give_me_a_hand, inspection.

  ros2 launch framework_bringup robocup.launch.py dry_run:=true


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
        'enable_face': 'true',
        'enable_depth_ranging': 'true',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'true',
        'enable_oak': 'true',
        'persons_topic': 'persons_ranged'})
