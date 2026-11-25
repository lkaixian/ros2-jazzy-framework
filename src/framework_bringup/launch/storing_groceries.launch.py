"""RoboCup@Home Malaysia 2026 Stage I, Storing Groceries (minutes 4.5): recognise grocery items on a table (outside the arena, venue unconfirmed), pick
and place each on the right shelf by category; objects random, not disclosed.

Starts: objects, the arm (arm_extension) and the OAK arm eye, lidar + RealSense, Nav2; audio off (autonomous task).
Plan: "store the groceries from <table place>" -> macro store_groceries_round (drive to the table, then per item: NextObject -> pick_object -> drive to
the shelf -> put_down -> say which group it belongs to -> back to the table; up to 6 items; a failed pick does not stop the round).
Needs the ARM running first (tools/arm/arm_up.sh) and the arm eye detecting (tools/arm/arm_eye.sh -> /arm_eye/objects). arm_extension does nothing live
until `live: true` (extensions.yaml) AND dry_run:=false; arm places (table, shelf, bin, ...) are "name:x,y,z" in its `places` parameter, still empty.
Objects -> destinations: param/object_categories.yaml (placeholders, COCO labels; the node NextObject). tools/integration/arm_task_test.py runs the plans
against fake extensions (order of calls, values passed on, loops, failures): no arm, no robot, no real object has been touched.
GAPS: grasping a real object has never been done (hand-eye calibration, object 3D pose, grip force: notes/ARM_BRINGUP.md); one `shelf` arm place only, no shelf
levels, no looking at the shelf to put an item next to similar ones; the arm places and the places `shelf` / the table are not taught or mapped (3 Dec);
labels are the 80 COCO classes (real groceries need a dataset, 3 Dec); the venue (outside the arena) has no map; items are picked from where the arm eye sees them
(the robot does not search the table or move along it); no items left or dropped detection beyond "the pick failed".
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
