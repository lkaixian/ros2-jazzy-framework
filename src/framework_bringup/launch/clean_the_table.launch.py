"""Stage II, Clean the Table (minutes 5.1): two beverage bottles; identify, pick, place random objects; decide the place per object (waste -> dustbin,
kitchenware -> dishwasher).

Starts like storing_groceries (objects, arm + OAK, Nav2, no audio).
GAPS: everything manipulation (see storing_groceries); object -> destination rule (waste vs kitchenware) has no table yet; dishwasher / dustbin places
are not in places.yaml; opening the dishwasher (door / rack) is not automated; bottles need a detector class and a grip-force limit (no force sensor:
notes/ARM_BRINGUP.md item 1).

Any enable_* below can be overridden on the command line; dry_run stays true until you pass dry_run:=false. Notes: docs/robocup-tasks.md.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'false',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'true',
        'enable_person': 'false',
        'enable_pose': 'false',
        'enable_face': 'false',
        'enable_depth_ranging': 'false',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'true',
        'enable_oak': 'true'})
