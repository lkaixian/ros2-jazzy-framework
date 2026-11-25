"""Stage II, Clean the Table (minutes 5.1): two beverage bottles; identify, pick, place random objects; decide the place per object (waste -> dustbin,
kitchenware -> dishwasher).

Starts: objects, arm + OAK arm eye, lidar + RealSense, Nav2; audio off.
Plan: "clean the table in <place>" -> macro clean_the_table_round (per item: NextObject -> pick_object -> drive to the dishwasher or dustbin -> put_down ->
say it -> back to the table; up to 6 items).
Needs the ARM running first (tools/arm/arm_up.sh) and the arm eye detecting (tools/arm/arm_eye.sh -> /arm_eye/objects). arm_extension does nothing live
until `live: true` (extensions.yaml) AND dry_run:=false; arm places (table, shelf, bin, ...) are "name:x,y,z" in its `places` parameter, still empty.
Objects -> destinations: param/object_categories.yaml (placeholders, COCO labels; the node NextObject). tools/integration/arm_task_test.py runs the plans
against fake extensions (order of calls, values passed on, loops, failures): no arm, no robot, no real object has been touched.
GAPS: everything arm (see storing_groceries); the dishwasher is not an arm place and its door / rack are not opened (PlaceObject refuses it today: the plan fails
there with a spoken apology); what a beverage bottle is (waste or kitchenware) is a guess; "Mickey Mouse" style objects have no class; places `dishwasher`,
`dustbin` and the table are not mapped (3 Dec); one object per trip (no tray).
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
