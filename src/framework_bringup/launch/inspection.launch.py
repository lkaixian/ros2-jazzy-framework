"""3 Dec robot inspection (minutes 3.2): enter by the gate, navigate to a predefined place in the arena with obstacle avoidance within 3 minutes. Also
check the emergency stop and the safety (hardware: nothing here).

Starts: lidar + RealSense, Nav2 and the safety mux only (no audio, no perception, no arm). The LLM pipeline still starts (framework.launch.py always includes it).
Send the goal: ros2 action send_goal /navigate_to_pose, or the planner: "go to the inspection point".
GAPS: the arena map does not exist until the 3 Dec mapping slot; place `inspection` is not in places.yaml; a hardware e-stop on the 12 V line is still open;
depth obstacles are off until the camera TF is measured; the 3-minute run has not been timed.

Any enable_* below can be overridden on the command line; dry_run stays true until you pass dry_run:=false. Notes: docs/robocup-tasks.md.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'false',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'false',
        'enable_person': 'false',
        'enable_pose': 'false',
        'enable_face': 'false',
        'enable_depth_ranging': 'false',
        'enable_odom': 'false',
        'enable_pid': 'false',
        'enable_nav': 'true',
        'enable_social': 'false',
        'enable_arm': 'false',
        'enable_oak': 'false'})
