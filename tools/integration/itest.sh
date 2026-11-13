#!/bin/bash
# itest.sh NAME dry_run(true|false) SPEC_JSON halt_after_sec|none : task -> fake llama -> SLM -> validator -> bt_executor -> odom_extension -> mux -> simulated robot.
# Portable: finds its own folder, logs to $OUT (default /tmp/robocup_it), uses an isolated DDS
# domain (default 45, localhost only) and stops what it started on exit.
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TOOLS/../.." && pwd)"
OUT="${OUT:-/tmp/robocup_it}"; mkdir -p "$OUT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
trap 'kill -INT $(jobs -p) 2>/dev/null; sleep 3; kill $(jobs -p) 2>/dev/null' EXIT
# usage: itest.sh <name> <dry_run true|false> <spec-json> <halt_after_sec|none>
export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash"
N=$1; echo "$3" > $OUT/spec_$N.json
SPEC=$OUT/spec_$N.json python3 $TOOLS/fake_llama2.py > /dev/null 2>&1 &
python3 $TOOLS/sim_robot.py $OUT/pose_$N.txt > $OUT/sim_$N.log 2>&1 &
ros2 launch framework_bringup extensions.launch.py enable_arm:=false enable_pose:=false > $OUT/ext_$N.log 2>&1 &
ros2 launch framework_bringup pipeline.launch.py dry_run:=$2 auto_run:=true > $OUT/pipe_$N.log 2>&1 &
sleep 11
(timeout 30 ros2 topic echo /bt/status --field data > $OUT/status_$N.txt 2>&1 &)
sleep 1
ros2 topic pub --once /bt/task std_msgs/msg/String "{data: 'do it'}" > /dev/null 2>&1
if [ "$4" != "none" ]; then sleep $4; ros2 service call /bt_executor/halt std_srvs/srv/Trigger > $OUT/halt_$N.txt 2>&1; fi
sleep 9
echo "== $N (dry_run=$2)"; echo "status:"; grep -o '"state":"[a-z_]*","message":"[^"]*"' $OUT/status_$N.txt | tail -4
echo "pose:   $(cat $OUT/pose_$N.txt)"
