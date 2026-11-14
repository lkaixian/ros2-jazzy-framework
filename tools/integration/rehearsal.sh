#!/bin/bash
# rehearsal.sh: scripted task rehearsals (P5). Each case feeds a task spec (what the LLM would answer) through the real
# orchestrator -> macro expansion -> validator -> bt_executor in DRY-RUN and checks the exact node sequence that would run.
# Cases mirror the score sheets in notes/TASKS.md. Uses itest.sh (isolated DDS domain, fake LLM, cleans up after itself).
#   tools/integration/rehearsal.sh            # exit 0 only if every case produced the expected sequence
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OUT="${OUT:-/tmp/robocup_rehearsal}"; mkdir -p "$OUT"
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
echo "$fails case(s) failed"
exit $((fails > 0))
