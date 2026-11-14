#!/bin/bash
# record_run.sh [NAME] [--images]: record everything needed to replay or debug a run (rosbag2, mcap).
#   tools/record_run.sh gpsr_try1          # -> ~/robocup_bags/gpsr_try1_<time>/
#   tools/record_run.sh gpsr_try1 --images # also the camera images (large: about 30 MB/s raw)
# Latched topics (task/state, llm/status, tf_static, ...) are recorded with their transient_local QoS so a replay
# re-latches them. Replay:  ros2 bag play <dir> --clock   (and run consumers with use_sim_time:=true)
set -euo pipefail
NAME="${1:-run}"; IMAGES=0; [[ "${2:-}" == "--images" ]] && IMAGES=1
OUT="${BAG_DIR:-$HOME/robocup_bags}/${NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$OUT")"
TOPICS=(/rosout /tf /tf_static /bt/task /bt/task_spec /bt/plan /bt/status /task/state /llm/status /llm/error /scene/summary
        /persons /objects /faces /stt/transcript /tts/say /tts/status /cmd_vel /odom /scan /safety/stop_state
        /events/bumper /events/cliff /sensors/core /cluster/status /cluster/node_info)
[[ $IMAGES -eq 1 ]] && TOPICS+=(/camera/color/image_raw /camera/depth/image_rect_raw /detections/image)
QOS="$(mktemp --suffix=.yaml)"; trap 'rm -f "$QOS"' EXIT
for t in /task/state /llm/status /tf_static /cluster/status; do
  printf '%s:\n  history: keep_last\n  depth: 1\n  reliability: reliable\n  durability: transient_local\n' "$t" >> "$QOS"
done
echo "recording ${#TOPICS[@]} topics to $OUT (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$OUT" --qos-profile-overrides-path "$QOS" "${TOPICS[@]}"
