"""RoboCup@Home Receptionist (HRI challenge) only (score sheet 2026-11-24, 1450 max, 7 min): doorbell 2x30, open the door 2x200, look at the
speaker 2x50, offer a free seat 2x100, tell guest 1's attribute to guest 2 4x20, introduce name + drink 4x30, look at the right guest 2x50,
take the bag 50, follow the host to the bag area 200, drop the bag 50; penalties for wrong info (-40) and ignoring people (-200).

  ros2 launch framework_bringup receptionist.launch.py         # add dry_run:=false when the robot may move; any enable_* below can be overridden

Microphone + sound-event (doorbell) + STT/TTS, face recognition (who is who, acknowledge people), person + pose + depth (look at the speaker, follow the
host), objects (free chair / sofa), Nav2 + lidar + RealSense. No arm: door opening and the bag handover need it (add enable_arm:=true with the
arm driver running). Plan read-out is off: this test does not score it. Task notes: docs/robocup-tasks.md.
GAPS: the door is NOT opened by the robot (receive_guest asks a person to; arm); no bag handover / drop (arm); only the shirt colour is described
(describe_person; glasses, hair, jacket are not extracted); no head, the base turns to look; offer_seat / hric_round exist but are untested with real chairs
and guests; losing the host while following (200) is untested; a real doorbell is untested.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true', 'announce_plans': 'false', 'enable_sensors': 'true', 'enable_navigation': 'true',
        'enable_object': 'true', 'enable_person': 'true', 'enable_pose': 'true', 'enable_face': 'true',
        'enable_depth_ranging': 'true', 'persons_topic': 'persons_ranged',
        'enable_odom': 'true', 'enable_pid': 'true', 'enable_nav': 'true', 'enable_social': 'true', 'enable_arm': 'false'})
