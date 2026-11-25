#!/usr/bin/env bash
# Start the OpenMANIPULATOR-X stack. On Ctrl+C / SIGTERM it parks the arm slowly BEFORE stopping
# the driver (stopping the driver turns torque off, and a raised arm would drop).
#   tools/arm/arm_up.sh            hold where the arm is (no startup move)
#   tools/arm/arm_up.sh --home     also do the slow (45 s) move out to the stock `home` pose
# If the park fails the driver is left running (holding the arm): Ctrl+C again to stop anyway.
# Not an e-stop. Keep a hand on the 12 V supply.
set -u
here=$(cd "$(dirname "$0")" && pwd)

port=$(ls /dev/serial/by-id/*OpenCR* 2>/dev/null | head -1)
[ -n "$port" ] || { echo "no OpenCR under /dev/serial/by-id (cable? power?)"; exit 1; }

# a CYCLONEDDS_URI that points at a missing file makes every ROS node crash at start
if [[ ${CYCLONEDDS_URI:-} == file://* && ! -f ${CYCLONEDDS_URI#file://} ]]; then
  echo "warn: ignoring CYCLONEDDS_URI, file missing: ${CYCLONEDDS_URI}"; unset CYCLONEDDS_URI
fi

args=("port_name:=$port" "init_position:=false")
if [[ ${1:-} == --home ]]; then
  tmp=$(mktemp --suffix=.yaml)
  sed 's/duration: 3.0/duration: 45.0/' \
    "$(ros2 pkg prefix open_manipulator_bringup)/share/open_manipulator_bringup/config/open_manipulator_x/initial_positions.yaml" > "$tmp"
  args=("port_name:=$port" "init_position:=true" "init_position_file:=$tmp")
fi

# own session: a terminal Ctrl+C must reach only this script, not the driver, or torque goes off
# while the park is still moving. SIGTERM to the whole group: INT is ignored by background jobs and a lone TERM orphans the driver.
setsid ros2 launch open_manipulator_bringup open_manipulator_x.launch.py "${args[@]}" &
lp=$!

stop() {
  trap 'kill -TERM -- "-$lp" 2>/dev/null' INT TERM     # a second Ctrl+C stops the driver anyway
  if python3 "$here/arm_tool.py" park; then
    kill -TERM -- "-$lp" 2>/dev/null
  else
    echo "PARK FAILED: driver left running so the arm stays held. Support the arm, then Ctrl+C again."
  fi
}
trap stop INT TERM
while kill -0 "$lp" 2>/dev/null; do wait "$lp"; done
