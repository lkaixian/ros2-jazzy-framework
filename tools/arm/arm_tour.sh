#!/usr/bin/env bash
# Range-of-motion tour of the OpenMANIPULATOR-X, to run in your OWN terminal with your hand ready:
#   rest -> home -> long (39 cm forward) -> high (straight up) -> turn 180 deg (+-90 about the front) -> home -> rest
#
#   tools/arm/arm_tour.sh --dry                 print the plan, start nothing
#   tools/arm/arm_tour.sh                       real run, first pass at half the park speed (~11 min)
#   tools/arm/arm_tour.sh --first right         turn right first;  --rate / --turn-rate rad/s to go faster
#
# Ctrl+C = FREEZE: the arm is held where it is with torque ON (it does not drop). Then a prompt asks:
#   p = park back to rest slowly (Ctrl+C during the park freezes again),  q = stop the driver (torque off: a raised arm FALLS).
# Ctrl+C is deliberately ignored at that prompt so a second press cannot drop the arm. Not an e-stop: keep a hand on the 12 V supply.
# Closing the terminal also freezes the arm; the driver is left running (holding) in its own session.
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
# shellcheck disable=SC1091
[ -f "$repo/env.sh" ] && source "$repo/env.sh" >/dev/null 2>&1   # ROS setup scripts are not safe under `set -u`
set -u

if [[ " $* " == *" --dry "* ]]; then exec python3 "$here/arm_tool.py" tour "$@"; fi

port=$(ls /dev/serial/by-id/*OpenCR* 2>/dev/null | head -1)
[ -n "$port" ] || { echo "no OpenCR under /dev/serial/by-id (cable? 12 V power?)"; exit 1; }
pgrep -f "[l]ib/controller_manager/ros2_control_node" >/dev/null && { echo "a driver is already running; stop it first"; exit 1; }
if [[ ${CYCLONEDDS_URI:-} == file://* && ! -f ${CYCLONEDDS_URI#file://} ]]; then
  echo "warn: ignoring CYCLONEDDS_URI, file missing: ${CYCLONEDDS_URI}"; unset CYCLONEDDS_URI
fi

echo "starting the driver (torque turns on at the rest pose, nothing moves)..."
setsid ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  "port_name:=$port" init_position:=false >/tmp/arm_tour_driver.log 2>&1 &
lp=$!
stop_driver() { kill -TERM -- "-$lp" 2>/dev/null; wait "$lp" 2>/dev/null; }

for _ in $(seq 1 20); do
  timeout 4 ros2 topic echo --once /joint_states >/dev/null 2>&1 && break
  sleep 1
done
timeout 4 ros2 topic echo --once /joint_states >/dev/null 2>&1 || {
  echo "driver did not come up (log: /tmp/arm_tour_driver.log)"; stop_driver; exit 1; }

read -r -p "Area clear, hand on the 12 V supply, OAK cable has slack? Type go to start: " ans
if [[ $ans != go ]]; then echo "not started"; stop_driver; exit 0; fi

rc=0
python3 "$here/arm_tool.py" tour "$@" || rc=$?
while (( rc != 0 )); do
  trap 'echo; echo "(Ctrl+C is ignored here: it must not drop the arm. Type p or q.)"' INT
  read -r -p "ARM IS HELD. [p] park back to rest slowly   [q] stop driver, torque off (a raised arm FALLS): " ans
  trap - INT
  case $ans in
    p) python3 "$here/arm_tool.py" park && rc=0 || rc=1 ;;
    q) rc=0 ;;
  esac
done
stop_driver
echo "driver stopped"
