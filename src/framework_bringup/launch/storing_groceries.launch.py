"""RoboCup@Home Malaysia 2026 Stage I, Storing Groceries (minutes 4.5): recognise grocery items on a table (outside the arena, venue unconfirmed), pick
and place each on the right shelf by category; objects random, not disclosed.

Starts: objects, the arm (arm_extension) and the OAK arm eye, lidar + RealSense, Nav2; audio off (autonomous task). Needs the arm driver running first
(tools/arm/arm_up.sh).
GAPS: the whole manipulation chain (PickObject / PlaceObject exist, grasp planning, object 3D pose, hand-eye calibration are open: notes/ARM_BRINGUP.md);
category -> shelf rule (a places.yaml / LLM prompt entry per shelf, shelf contents seen by the camera); detector classes for the real groceries (dataset
collection on 3 Dec); the venue is not on any map; arm mounted ~80 cm high, reach limits unknown.

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
