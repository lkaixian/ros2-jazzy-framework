"""RoboCup@Home Education 2025/26, Carry My Luggage (navigation, 5 min, max 1050; notes/EDU_PLAN.md): take the bag the operator points at, follow the operator
outside to the luggage area through four obstacles (small object, hard-to-see object, crowd, retractable barriers), hand the bag back; bonus: re-enter, join the queue.

  ros2 launch framework_bringup edu_luggage.launch.py             # robot idle; add dry_run:=false when it may move
  say "start the task"                                            # the only thing that starts it (spoken, any time after the launch)
  ros2 launch ... start_spec:='{"goal":"edu luggage task","steps":[{"macro":"luggage_round","args":{"distance":"1.2"}}]}'   # other arguments

Starts: audio (microphone + STT + TTS), lidar + RealSense, Nav2, objects, person + pose + depth, odom/PID/nav/social extensions. No faces. The arm is OFF: add
enable_arm:=true (arm driver running, `tools/arm/arm_up.sh`) for the bag handover. While the start phrase is configured nothing the operator says becomes a task
(transcript_to_task_node); the spoken phrase publishes the task spec directly (no LLM) and the plan runs at once (auto_run; dry_run still keeps the hardware still).
Plan: macro luggage_round = say, optional bag handover (skipped when the arm or the handover fails), follow_until_told (stop words stop / we are here / we have
arrived / this is the car), give the bag back on "thank you", return_home (bonus, fails outside the map without ending the task).
Flow (macro luggage_round): "Which bag should I carry? Please point at it." -> PointedObject (the extended arm's line against backpack / handbag / suitcase boxes) -> "I will take the red backpack on my
left" (type from the detector, colour from the bag's crop) -> optional handover -> "I am ready. Say follow me" -> only the phrase "follow me" starts the follow (after 90 s it starts anyway) -> follow_until_told with
trail=true -> it stops only when the followed person says stop / we are here OR holds a hand up for a second AND stands still (WaitForStop in pose_extension): a stop from anyone else or while the operator walks is ignored.
The hand sign is five extended fingers on the raised hand (hand-landmark ONNX models in models/hand/, see docs/perception.md; a raised hand alone when the models are missing).
GAPS (notes/EDU_PLAN.md): the operator still hands the handle over (no grasp from the floor), nothing carries the bag in a safe pose while driving, depth obstacles are off (camera tilt unmeasured), nothing waits
for or asks a crowd to move, no queue. The way back is RetraceTrail (along the path the operator walked, odometry only, no map outside the arena) and then Nav2 to the start. None of it has run on the real base.
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
        'enable_face': 'false',
        'enable_depth_ranging': 'true',
        'enable_odom': 'true',
        'enable_pid': 'true',
        'enable_nav': 'true',
        'enable_social': 'true',
        'enable_arm': 'false',
        'enable_oak': 'false',
        'persons_topic': 'persons_ranged',
        'max_runtime_sec': '420',
        'start_phrase': 'start the task,start task',
        'start_spec': '{"goal":"edu luggage task","steps":[{"macro":"luggage_round","args":{}}]}',
    })
