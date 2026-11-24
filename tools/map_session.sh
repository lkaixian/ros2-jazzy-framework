#!/usr/bin/env bash
# One-terminal mapping session: driver + lidar + safety mux + slam_toolbox in the background, teleop in front.
#   tools/map_session.sh room2     drive; quit teleop (Ctrl-C) to save ~/maps/room2 and stop everything
#   tools/map_session.sh --nav room2 [x,y,yaw]   navigate on ~/maps/room2 instead: same driver/lidar/mux, AMCL + Nav2.
#                                  Put the robot where the mapping run started (initial pose 0,0,0) or pass x,y,yaw.
# Logs: /tmp/map_session/*.log. Come back to the start spot before you quit (loop closure).
NAV=""; [ "${1:-}" = "--nav" ] && { NAV=1; shift; }
NAME="${1:?usage: map_session.sh [--nav] <map name> [x,y,yaw]}"; POSE="${2:-0,0,0}"
cd "$(dirname "$0")/.." && . env.sh
[ -f "$HOME/ros2_ws/install/setup.bash" ] && . "$HOME/ros2_ws/install/setup.bash"   # kobuki + sllidar drivers
[ -z "$NAV" ] && [ -e "$HOME/maps/$NAME.yaml" ] && { echo "$NAME already exists in ~/maps; pick a new name"; exit 1; }
mkdir -p /tmp/map_session "$HOME/maps"
# every process: output to its log AND the terminal with its own colour prefix (QUIET=1: log only)
start() {
  local n=$1 c=$2; shift 2
  if [ -n "${QUIET:-}" ]; then
    setsid "$@" > "/tmp/map_session/$n.log" 2>&1 &
  else
    { setsid "$@" 2>&1 | tee "/tmp/map_session/$n.log" | sed -u "s/^/\x1b[${c}m[$n]\x1b[0m /"; } &
  fi
  echo "started $n"; sleep "${DELAY:-3}"
}
# the driver's stock config says /dev/ttyUSB0, which is the lidar here: pin both by udev name
sed 's#^kobuki_ros_node:#/**:#; s#device_port: .*#device_port: /dev/kobuki#' \
  "$HOME/ros2_ws/install/kobuki_node/share/kobuki_node/config/kobuki_node_params.yaml" > /tmp/map_session/kobuki.yaml
start kobuki 36 ros2 run kobuki_node kobuki_ros_node --ros-args --params-file /tmp/map_session/kobuki.yaml
start sensors 33 ros2 launch framework_bringup sensors.launch.py enable_camera:=false
start mux 32 ros2 launch framework_bringup extensions.launch.py cmd_vel_topic:=commands/velocity
if [ -n "$NAV" ]; then
  [ -f "$HOME/maps/$NAME.yaml" ] || { echo "no ~/maps/$NAME.yaml"; tools/stop_all.sh; exit 1; }
  start nav 35 ros2 launch framework_bringup navigation.launch.py map:="$HOME/maps/$NAME.yaml" initial_pose:="$POSE"
else
  start slam 35 ros2 launch framework_bringup navigation.launch.py slam:=slam_toolbox navigation:=false
fi
if ! timeout 10 ros2 topic echo --once /scan --field header.frame_id > /dev/null 2>&1; then
  echo "NO /scan: the lidar is not publishing (see the [sensors] lines above). Mapping needs it; stopping."
  tools/stop_all.sh; exit 1
fi
echo "/scan OK"
if [ -n "$NAV" ]; then
  echo "navigating on $NAME, initial pose $POSE. Send goals from the dashboard/BT, Ctrl-C to stop everything."
  trap 'tools/stop_all.sh; exit 0' INT TERM
  while :; do sleep 1; done
fi
echo "teleop: w/a/s/d, slow, return to the start spot, then Ctrl-C to save"
ros2 run framework_bringup teleop
echo "saving ~/maps/$NAME ..."
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/$NAME" || echo "SAVE FAILED"
tools/stop_all.sh
