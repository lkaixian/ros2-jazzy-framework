#!/usr/bin/env bash
# One command instead of many terminals. It never turns on real motion by itself.
#   tools/start.sh base                    e-stop reader + Kobuki driver
#   tools/start.sh arm [--oak]             RealSense + object detector + arm extension (+ the OAK eye with --oak)
#                                          the arm DRIVER is not in here: run tools/arm/arm_up.sh in its own terminal first (Ctrl+C there parks the arm)
#   tools/start.sh task NAME [--move]      base + one task launch; NAME = gpsr egpsr inspection receptionist restaurant help_me_carry edu_luggage edu_mates edu_reception
#                                          storing_groceries clean_the_table give_me_a_hand robocup
#   tools/start.sh stop                    stop everything this project started (tools/stop_all.sh)
# --list on any of them prints the commands and starts nothing.
# --move runs the task with dry_run:=false (the base and the extensions WILL command hardware) and asks you to type go. The arm's own switch is
# separate: `live` in src/framework_bringup/param/extensions.yaml (false = it only plans and prints DRY:).
# Output goes to /tmp/robot_start/<name>.log and to this terminal with a prefix; Ctrl+C stops everything this script started.
set -u
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/.." && pwd)
mode=${1:-}
[ $# -gt 0 ] && shift
task=""
if [ "$mode" = task ]; then task=${1:-}; [ $# -gt 0 ] && shift; fi
LIST="" MOVE="" OAK=""
for a in "$@"; do
  case $a in --list) LIST=1 ;; --move) MOVE=1 ;; --oak) OAK=1 ;; *) echo "unknown option: $a" >&2; exit 2 ;; esac
done
TASKS="gpsr egpsr inspection receptionist restaurant help_me_carry storing_groceries clean_the_table give_me_a_hand robocup edu_luggage edu_mates edu_reception"
ARM_TASKS="egpsr storing_groceries clean_the_table give_me_a_hand robocup"

if [ "$mode" = stop ]; then exec "$here/stop_all.sh"; fi

estop_port() {
  if [ -n "${ESTOP_PORT:-}" ]; then echo "$ESTOP_PORT"
  elif [ -e /dev/estop ]; then echo /dev/estop
  else echo /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0; fi
}

NAMES=() CMDS=() NOTES=()
add() { NAMES+=("$1"); CMDS+=("$2"); }
base() {
  add estop "python3 tools/estop/estop_node.py --port $(estop_port)"
  # the stock launch file opens /dev/ttyUSB0, which is the RPLIDAR on this robot (udev names: /dev/kobuki, /dev/rplidar): name the port
  add kobuki "ros2 run kobuki_node kobuki_ros_node --ros-args --params-file $HOME/ros2_ws/install/kobuki_node/share/kobuki_node/config/kobuki_node_params.yaml -p device_port:=${KOBUKI_PORT:-/dev/kobuki}"
}
case $mode in
  base) base ;;
  arm)
    NOTES+=("the arm driver must already be running: tools/arm/arm_up.sh in its own terminal (Ctrl+C there parks the arm)")
    add realsense "ros2 launch framework_bringup sensors.launch.py enable_lidar:=false enable_camera:=true"
    add detector "ros2 launch framework_bringup perception.launch.py enable_person:=false enable_face:=false"
    add arm_ext "ros2 launch framework_bringup extensions.launch.py enable_odom:=false enable_pid:=false enable_pose:=false enable_social:=false enable_nav:=false enable_arm:=true"
    [ -n "$OAK" ] && add oak "tools/arm/arm_eye.sh" ;;
  task)
    case " $TASKS " in *" $task "*) ;; *) echo "task NAME must be one of: $TASKS" >&2; exit 2 ;; esac
    case " $ARM_TASKS " in *" $task "*) NOTES+=("this task uses the arm: start tools/arm/arm_up.sh (and tools/arm/arm_eye.sh) first; the arm still only moves when live: true") ;; esac
    base
    add "$task" "ros2 launch framework_bringup $task.launch.py dry_run:=$([ -n "$MOVE" ] && echo false || echo true)" ;;
  *) sed -n 2,12p "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

for n in "${NOTES[@]:-}"; do [ -n "$n" ] && echo "NOTE: $n"; done
for i in "${!NAMES[@]}"; do printf '%-9s %s\n' "${NAMES[$i]}" "${CMDS[$i]}"; done
[ -n "$LIST" ] && exit 0
if [ -n "$MOVE" ]; then
  read -r -p "--move: the robot base and extensions WILL command hardware. Floor clear, hand on the e-stop. Type go: " ok
  [ "$ok" = go ] || { echo "not started"; exit 1; }
fi

logs=/tmp/robot_start
mkdir -p "$logs"
pids=() tails=()
for i in "${!NAMES[@]}"; do
  n=${NAMES[$i]}
  : > "$logs/$n.log"
  # own session: Ctrl+C in this terminal must reach only this script, which then stops each group in order
  setsid bash -c "cd '$repo'; source env.sh >/dev/null 2>&1; [ -f \"\$HOME/ros2_ws/install/setup.bash\" ] && source \"\$HOME/ros2_ws/install/setup.bash\"; exec ${CMDS[$i]}" \
    > "$logs/$n.log" 2>&1 &
  pids+=($!)
  tail -n0 -F "$logs/$n.log" 2>/dev/null | sed -u "s/^/[$n] /" &
  tails+=($!)
  echo "started $n (pid $!), log $logs/$n.log"
  sleep "${DELAY:-3}"
done

alive() { local p; for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && return 0; done; return 1; }
cleanup() {
  trap '' INT TERM
  trap - EXIT
  echo; echo "stopping: SIGINT to each program, then SIGTERM, then SIGKILL"
  local p
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  for _ in $(seq 1 16); do alive || break; sleep 0.5; done
  for p in "${pids[@]}"; do kill -TERM -- "-$p" 2>/dev/null; done
  for _ in $(seq 1 8); do alive || break; sleep 0.5; done
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
  kill "${tails[@]}" 2>/dev/null
  echo stopped
  exit 0
}
trap cleanup INT TERM
echo "running. Ctrl+C stops all of the above. Logs: $logs/"
while alive; do sleep 1; done
cleanup
