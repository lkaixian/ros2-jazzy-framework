#!/bin/bash
# rehearsal.sh: scripted task rehearsals (P5). Each case feeds a task spec (what the LLM would answer) through the real
# orchestrator -> macro expansion -> validator -> bt_executor in DRY-RUN and checks the exact node sequence that would run.
# Cases mirror the score sheets in notes/TASKS.md. Uses itest.sh (isolated DDS domain, fake LLM, cleans up after itself).
#   tools/integration/rehearsal.sh            # exit 0 only if every case produced the expected sequence
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OUT="${OUT:-/tmp/robocup_rehearsal}"; mkdir -p "$OUT"
# every extension up + one manifest republish before a task is sent: otherwise the validator may not know all nodes yet and the
# retry path silently drops the unknown steps (seen once: scout_room ran only its CountPeople steps)
export ITEST_ACTION_SERVERS="odom_extension/execute,social_extension/execute,nav_extension/execute,pose_extension/execute"
export ITEST_SETTLE=5
# a rehearsal checks the node sequence from the executor's [dry-run] log lines: nothing may run for real (pipeline.yaml exempts social_extension)
export ITEST_PIPELINE_ARGS="dry_run_exempt:=none"
fails=0
# case NAME SPEC_JSON "expected node ids, space separated, in execution order"
case_() {
  local name=$1 spec=$2 want=$3
  bash "$TOOLS/itest.sh" "$name" true "$spec" none > "$OUT/run_$name.txt" 2>&1
  local got
  got=$(grep -a "\[dry-run\]" "$OUT/pipe_$name.log" | sed -E 's/.*\[dry-run\] [a-z_]+\/([A-Za-z]+)\(.*/\1/' | tr '\n' ' ' | sed 's/ $//')
  if [[ "$got" == "$want" ]] && grep -aq '"state":"finished","message":"SUCCESS"' "$OUT/run_$name.txt"; then
    echo "PASS $name: $got"
  else
    echo "FAIL $name"; echo "  want: $want"; echo "  got : $got"; fails=$((fails + 1))
  fi
}
# GPSR: "go to the door and say hello"
case_ gpsr_go_and_say '{"goal":"gpsr","steps":[{"macro":"go_and_say","args":{"place":"door","text":"hello"}}]}' \
  "NavigateToPlace Speak"
# GPSR: patrol, repeated (loop support)
case_ gpsr_patrol_laps '{"goal":"laps","steps":[{"macro":"patrol_laps","args":{"first":"left_room","second":"right_room","laps":"2"}}]}' \
  "NavigateToPlace NavigateToPlace NavigateToPlace NavigateToPlace"
# HRIC: ask a guest name + drink, remember, enrol the face
case_ hric_meet_guest '{"goal":"hric","steps":[{"macro":"meet_guest","args":{"slot":"guest1"}}]}' \
  "Speak WaitForTranscript ExtractAnswer Speak Speak WaitForTranscript ExtractAnswer EnrollFace"
# GPSR: a task the robot cannot do must end in an apology, not in motion
case_ gpsr_refuse_bring '{"goal":"bring","steps":[{"macro":"say","args":{"text":"Sorry, I have no arm and cannot bring objects"}}]}' \
  "Speak"
# HRIC: one guest at the door (doorbell -> door -> name + drink); find_person is left out: DetectPerson cannot succeed in dry-run
case_ hric_receive_guest '{"goal":"rg","steps":[{"macro":"receive_guest","args":{"slot":"guest1"}}]}' \
  "DetectSound Speak NavigateToPlace Speak WaitForTranscript ExtractAnswer Speak Speak WaitForTranscript ExtractAnswer EnrollFace"
# walking around: legs of DriveDistance + RotateBy (the lidar guard turns away at run time; dry-run drives happily)
case_ walk_wander '{"goal":"w","steps":[{"macro":"wander","args":{"legs":"2"}}]}' \
  "DriveDistance RotateBy DriveDistance RotateBy"
# a loop written by the LLM: foreach unrolled at plan time, macros expanded inside
case_ foreach_tour '{"goal":"tour","steps":[{"foreach":["left_room","door"],"as":"p","steps":[{"macro":"go_and_say","args":{"place":"${p}","text":"hi"}}]}]}' \
  "NavigateToPlace Speak NavigateToPlace Speak"
# driving and speaking at the same time (Parallel)
case_ announce_and_go '{"goal":"a","steps":[{"macro":"announce_and_go","args":{"place":"door","text":"On my way"}}]}' \
  "NavigateToPlace Speak"
# look-around report: 4 stops, each counts and speaks
case_ scout_room '{"goal":"s","steps":[{"macro":"scout_room"}]}' \
  "RotateBy CountPeople Speak RotateBy CountPeople Speak RotateBy CountPeople Speak RotateBy CountPeople Speak"
echo "$fails case(s) failed"
exit $((fails > 0))
