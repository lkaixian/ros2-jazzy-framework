"""RoboCup@Home Education 2025/26, Receptionist (speech; notes/EDU_PLAN.md): welcome two guests at the door, lead them to the living room, introduce each to the host
and the other guest (name and favourite drink), offer a free seat, and describe guest 1 to guest 2; gaze at the speaker and in the direction of travel.

  ros2 launch framework_bringup edu_reception.launch.py            # robot idle; add dry_run:=false when it may move
  say "start the task"                                             # the only thing that starts it
  ros2 launch ... start_spec:='{"goal":"edu reception task","steps":[{"macro":"hric_round","args":{"host":"John","living_place":"living_room"}}]}'   # the host's real name

Starts: audio (microphone, doorbell sound event, STT, TTS), faces, objects (seats), person + pose + depth, Nav2, sensors, social extension. No arm: the door is open by
default and the robot does not open it. Plan: macro hric_round (it says it is starting; per guest: go to the door and wait for a person within 2.5 m or the bell, welcome,
face them, ask name and drink, remember the look; lead to the living room, introduce to the host, offer a seat; guest 2 is introduced to guest 1 with name and drink and
told how guest 1 looks).
Flow (macro hric_round; host, host_drink, host_pron are the facts announced before the test, passed in start_spec): at the door it waits for a person within 2.5 m or the bell,
backs away until the face shows, asks "Hi, may I know what is your name?", says "Hi <name>, welcome!", asks the favourite drink and says it back, enrols the face, reads he / she and
the age from a vision-language model on the planner's llama-server (DescribeWithVLM; Gemma 4 E4B has an mmproj; the face model EstimateGender is the fallback), says "please follow me", checks by the lidar behind that the guest follows, then in the living room
looks at the host (the face enrolled under the host's name with the enroll_host macro, else the nearest person) and says "Hi <host>, this is <guest>, he/she likes to drink <drink>", "Hello everyone",
then tells the guest who the host is and what the host likes; it faces the free seat (couch/bench: the free part beside whoever sits there), points at it with the arm when enable_arm is on
(ArmPoint at point_rate, then ArmPark) and says which seat. Guest 2 is also introduced to guest 1 and the other way round, and guest 1 is described to guest 2 (colours of top and bottoms, hair, glasses,
gender, age; at least the features nobody else shares).
Setup the day before / 2 h before: run macro enroll_host (the host in front of the camera) so the host has a fixed face and name.
GAPS: nothing looks at the speaker WHILE listening (only before each question); a wall behind the robot satisfies CheckBehind; seat finder = COCO chair/couch/bench boxes only (a coat or bag on a seat
is not seen; a host in the middle of a sofa leaves the ends only when the sofa is about three people wide); the gender/age model is a non-commercial research model with unmeasured accuracy on this camera
(it says they when unsure); no real microphone, doorbell or speaker test yet.
"""
from bt_common.launch_utils import task_launch


def generate_launch_description():
    return task_launch({
        'enable_audio': 'true',
        'announce_plans': 'false',
        'enable_sensors': 'true',
        'enable_navigation': 'true',
        'enable_object': 'true',
        'enable_person': 'true',
        'enable_pose': 'true',
        'enable_face': 'true',
        'enable_depth_ranging': 'true',
        'persons_topic': 'persons_ranged',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'false',
        'enable_oak': 'false',
        'max_runtime_sec': '600',
        'start_phrase': 'start the task,start task',
        'start_spec': '{"goal":"edu reception task","steps":[{"macro":"hric_round","args":{}}]}',
    })
