"""RoboCup@Home Receptionist (HRI challenge) only (score sheet 2026-11-24, 1450 max, 7 min): doorbell 2x30, open the door 2x200, look at the
speaker 2x50, offer a free seat 2x100, tell guest 1's attribute to guest 2 4x20, introduce name + drink 4x30, look at the right guest 2x50,
take the bag 50, follow the host to the bag area 200, drop the bag 50; penalties for wrong info (-40) and ignoring people (-200).

  ros2 launch framework_bringup receptionist.launch.py         # add dry_run:=false when the robot may move; any enable_* below can be overridden

Microphone + sound-event (doorbell) + STT/TTS, face recognition (who is who, acknowledge people), person + pose + depth (look at the speaker, follow the
host), objects (free chair / sofa), Nav2 + lidar + RealSense. No arm: door opening and the bag handover need it (add enable_arm:=true with the
arm driver running). Plan read-out is off: this test does not score it. Task notes: docs/robocup-tasks.md.
Plan: task "receptionist" / "hric round" -> macro hric_round (host = name, living_place = where the host and seats are): per guest wait_for_doorbell, go to
the door, welcome, face the guest, ask name and drink, remember how they look (DescribeAttributes, else the top colour); lead them to the living room,
introduce them to the host (name, drink), offer a free seat ("on my left"); guest 2 is also introduced to guest 1 and told how guest 1 looks.
Built and validated as a tree; tested in simulators only for the nodes (never the whole round, never on a robot).
GAPS: the door is NOT opened by the robot (a person does; arm); no bag handover or drop (arm; follow_until_told can follow the host to the bag area);
the host is just a name argument (not found or recognised) and the guests are told to follow, not checked; "look at the right guest" is only "look at the
nearest person"; seat finder = COCO chair / couch / bench boxes no person covers, biggest box first, no depth, tested on synthetic boxes (a bag or coat
on a seat is not seen); only certain attributes are told (glasses, hair colour, hat, earrings; else shirt colour), never length / style / jacket; the
doorbell and the speech recognition of names are untested with real audio; a real round is longer than the 7 min limit if every step is slow.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true', 'announce_plans': 'false', 'enable_sensors': 'true', 'enable_navigation': 'true',
        'enable_object': 'true', 'enable_person': 'true', 'enable_pose': 'true', 'enable_face': 'true',
        'enable_depth_ranging': 'true', 'persons_topic': 'persons_ranged',
        'enable_odom': 'true', 'enable_pid': 'true', 'enable_nav': 'true', 'enable_social': 'true', 'enable_arm': 'false'})
