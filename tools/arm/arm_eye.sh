#!/usr/bin/env bash
# Arm eye: the OAK driver (camera pose on the gripper from param/oak.yaml) plus object detection on its image, turned upright.
#   tools/arm/arm_eye.sh            then:  ros2 topic echo /arm_eye/objects     (boxes are in the UPRIGHT image)
# Topics: /arm_cam/rgb/image_raw, /arm_cam/stereo/image_raw (raw, upside down), /arm_eye/objects, /arm_eye/image (annotated, upright).
# Ctrl+C stops both. Needs the camera free (a leftover component_container from an unclean stop holds it).
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
# shellcheck disable=SC1091
[ -f "$repo/env.sh" ] && source "$repo/env.sh" >/dev/null 2>&1
set -u
if [[ ${CYCLONEDDS_URI:-} == file://* && ! -f ${CYCLONEDDS_URI#file://} ]]; then
  echo "warn: ignoring CYCLONEDDS_URI, file missing: ${CYCLONEDDS_URI}"; unset CYCLONEDDS_URI
fi
pgrep -f "[c]omponent_container" >/dev/null && { echo "a camera driver is already running: stop it first"; exit 1; }

setsid ros2 launch framework_bringup sensors.launch.py enable_lidar:=false enable_camera:=false enable_oak:=true >/tmp/arm_eye_camera.log 2>&1 &
cam=$!
for _ in $(seq 1 40); do
  ros2 topic list 2>/dev/null | grep -q "arm_cam/rgb/image_raw$" && break
  sleep 1
done
mkdir -p ~/.cache/arm_gui && cd ~/.cache/arm_gui || exit 1          # YOLO weights download here, not into the repo
setsid ros2 run object_detection object_detection_node --ros-args -r __node:=arm_eye_detection \
  -r objects:=/arm_eye/objects -r detections/image:=/arm_eye/image \
  -p image_topic:=/arm_cam/rgb/image_raw -p rotate:=180 >/tmp/arm_eye_detection.log 2>&1 &
det=$!
echo "arm eye running (logs: /tmp/arm_eye_camera.log, /tmp/arm_eye_detection.log). Ctrl+C to stop."
trap 'kill -TERM -- "-$det" "-$cam" 2>/dev/null; sleep 3; kill -KILL -- "-$det" "-$cam" 2>/dev/null; exit 0' INT TERM
wait
