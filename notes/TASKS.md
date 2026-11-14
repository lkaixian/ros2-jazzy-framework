# Target competition tasks (RoboCup@Home 2027 draft score sheets, rev 2026-10-14)

Source: https://robocupathome.github.io/RuleBook/scoresheets/master.pdf. Each test: 7:00 min limit, up to 3 tries,
"Not attending" -500, "Using alternative start signal" -100 (GPSR/PnPC/Laundry). Scores are max per try
(the sheet's "Score per try", without the outstanding-performance bonus). These are the acceptance targets.

## GPSR (max 1490)
3 spoken commands. Per command: understand 80, show the plan 100, solve 250 (partial solve = percentage penalty).
Interleaved-task bonus 200. Penalties: custom operator -20, rephrasing request -30, bypassing ASR -50 each.
Fit: STT -> LLM -> spec -> validator -> executor is exactly this path. Gaps: command grammar coverage (needs the
real LLM, B-21), "demonstrate a plan" = `bt/plan` spoken/shown by `plan_announcer`, no interleaved-command test.

## HRIC (max 1450): receptionist
Doorbell sound 2x30, open the door 2x200, look at the speaker 2x50, offer a free seat 2x100, look toward the navigation
goal 2x15, tell guest 1's visual attribute to guest 2 (+-20 x4), no needless confirmation questions 4x15,
introduce name + drink 4x30, look at the right guest while introducing 2x50, take the bag by handover 50,
follow the host to the bag area 200 (penalties for losing/asking/touch), drop the bag in the right area 50.
Penalties: wrong info memorised -40 x4, alternative HRI -20 x6, not acknowledging people -200 x2.
Fit: face enrol + Speak + pose exist. Gaps: doorbell sound detection, door opening, free-seat detection, gaze/head
control, visual attribute (clothes/hair) extraction, handover, person following with rediscovery.

## PnPC (max 3515): breakfast / tidy up
12 objects: recognise 10, pick 50, place 40 (bonuses for floor, cutlery, plate, dishwasher tab; shelf placement 2x30).
Extras: dishwasher rack 2x100, door 2x200, open milk 400, pour cereal/milk 2x200. Penalties: drops -40, human handover -100.
Fit: object detection exists. Gaps: the whole arm side (B-25: only goal construction tested), grasp planning,
placing, door/rack, pouring. Biggest points are manipulation, so this is arm-bound.

## Doing Laundry (max 4415)
Navigate 15, pick a garment from the basket 100, fold it 800 (-800 quality, assistance penalties), extras: open the
washing machine 300, take clothes out 300, carry with the basket 300, fold 5 more 5x400, stack folded 6x100.
Fit: navigation only. Gaps: cloth manipulation (folding is 800 + 5x400 of the 4415), basket carrying, machine door.

## Final demonstration (max 8450, 10:00)
Find/state a problem 3x150, solve a problem 10x650, move the laundry basket 600, open the apartment door 600, close the
dishwasher 300. Reuses all of the above.

## What this means for the backlog
1. GPSR is reachable first (software only): B-21 real LLM + B-01 node outputs + B-03 grammar, then rehearse
   with scripted commands as acceptance tests.
2. HRIC next: doorbell detector, seat finder, person follow, handover (arm needed for the last).
3. PnPC and Laundry are dominated by manipulation; they wait on B-25 (arm), so decide the arm early.
