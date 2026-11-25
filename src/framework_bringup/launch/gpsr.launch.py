"""RoboCup@Home GPSR only (score sheet 2026-11-24: understand 3x80, show the plan 3x100, solve 3x250, interleaved bonus 200; 7 min).

  ros2 launch framework_bringup gpsr.launch.py                 # add dry_run:=false when the robot may move; any enable_* below can be overridden

Speech in (STT -> bt/task), LLM + validator + executor, the plan read out loud (plan_announcer = "demonstrate a plan has been generated"),
Nav2 + lidar + RealSense, object / person / pose detection with depth ranging. No face recognition (add enable_face:=true if the
commands name people) and no arm (add enable_arm:=true with the arm driver running). Task notes: docs/robocup-tasks.md.
GAPS: command-generator grammar vs the real LLM (E4B once planned "drive forward" as a doorbell wait); real microphone accuracy unverified; no arm, so
pick / give commands fail; no interleaved-command test (+200); depth obstacles off until the camera TF is measured.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true', 'enable_sensors': 'true', 'enable_navigation': 'true',
        'enable_object': 'true', 'enable_person': 'true', 'enable_pose': 'true', 'enable_face': 'false',
        'enable_depth_ranging': 'true', 'persons_topic': 'persons_ranged',
        'enable_odom': 'true', 'enable_pid': 'true', 'enable_nav': 'true', 'enable_social': 'true', 'enable_arm': 'false'})
