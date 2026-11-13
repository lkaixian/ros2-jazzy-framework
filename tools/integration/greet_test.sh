#!/bin/bash
# greet_test.sh TREE.xml IDENTITY NAME : run a social BT (greet/enroll) against fake perception + mock TTS.
# Portable: finds its own folder, logs to $OUT (default /tmp/robocup_it), uses an isolated DDS
# domain (default 45, localhost only) and stops what it started on exit.
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$TOOLS/../.." && pwd)"
OUT="${OUT:-/tmp/robocup_it}"; mkdir -p "$OUT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
trap 'kill -INT $(jobs -p) 2>/dev/null; sleep 3; kill $(jobs -p) 2>/dev/null' EXIT
export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash"
TREE=$1; IDENT=$2; N=$3
python3 $TOOLS/fake_perception.py $IDENT > $OUT/fp_$N.log 2>&1 &
ros2 launch framework_bringup pipeline.launch.py dry_run:=false > $OUT/gp_$N.log 2>&1 &
ros2 launch framework_bringup extensions.launch.py enable_arm:=false enable_pose:=false enable_odom:=false enable_pid:=false > $OUT/ge_$N.log 2>&1 &
ros2 launch framework_bringup audio.launch.py enable_stt:=false announce_plans:=false tts_engine:=mock > $OUT/ga_$N.log 2>&1 &
sleep 12
(timeout 25 ros2 topic echo /tts/status --field text > $OUT/tts_$N.txt 2>&1 &)
(timeout 25 ros2 topic echo /bt/status --field data > $OUT/bts_$N.txt 2>&1 &)
sleep 2
ros2 run bt_orchestrator submit_bt $ROOT/install/framework_bringup/share/framework_bringup/behavior_trees/$TREE --run 2>&1 | tail -4
sleep 6
echo "--- spoken (tts/status text):"; grep -v "^---" $OUT/tts_$N.txt | grep -v '^$' | sort | uniq -c
echo "--- bt/status:"; grep -o '"state":"[a-z_]*","message":"[^"]*"' $OUT/bts_$N.txt | tail -3
echo "--- enrolled:"; grep ENROLLED $OUT/fp_$N.log | cut -c1-120
