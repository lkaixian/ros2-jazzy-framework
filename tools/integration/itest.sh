#!/bin/bash
# itest.sh NAME dry_run(true|false) SPEC_JSON halt_after_sec|none : task -> fake llama -> SLM -> validator -> bt_executor -> odom_extension -> mux -> simulated robot.
# Portable: finds its own folder, logs to $OUT (default /tmp/robocup_it), uses an isolated DDS
# domain (default 45, localhost only) and stops what it started on exit.
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TOOLS/../.." && pwd)"
OUT="${OUT:-/tmp/robocup_it}"; mkdir -p "$OUT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# set -m: every background job is its own process group, so the trap reaches ros2 launch's
# children too (they used to survive and pile up, and a stale live executor broke the next run)
set -m
trap 'for p in $(jobs -p); do kill -INT -- -$p 2>/dev/null; done; sleep 3; for p in $(jobs -p); do kill -KILL -- -$p 2>/dev/null; done' EXIT
# usage: itest.sh <name> <dry_run true|false> <spec-json> <halt_after_sec|none>
export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash"
N=$1; echo "$3" > $OUT/spec_$N.json
SPEC=$OUT/spec_$N.json python3 $TOOLS/fake_llama2.py > /dev/null 2>&1 &
python3 $TOOLS/sim_robot.py $OUT/pose_$N.txt > $OUT/sim_$N.log 2>&1 &
# this fake robot has no lidar: the odom guard must not refuse to drive for want of a scan (extensions.yaml: scan_required true on the real robot)
ros2 launch framework_bringup extensions.launch.py enable_arm:=false scan_required:=false scan_yaw_offset:=0.0 > $OUT/ext_$N.log 2>&1 &
ros2 launch framework_bringup pipeline.launch.py start_llama_server:=false dry_run:=$2 auto_run:=true $ITEST_PIPELINE_ARGS > $OUT/pipe_$N.log 2>&1 &
# one rclpy helper waits for the pipeline, sends the task, records bt/status and (optionally)
# halts after $4 s; the CLI versions raced DDS discovery and lost the task on slow machines
python3 $TOOLS/itest_task.py $OUT/status_$N.txt "$4"
sleep 1
echo "== $N (dry_run=$2)"; echo "status:"; grep -o '"state":"[a-z_]*","message":"[^"]*"' $OUT/status_$N.txt | tail -4
echo "pose:   $(cat $OUT/pose_$N.txt)"
