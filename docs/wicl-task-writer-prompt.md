You are helping build an independent test set for a small robot's task planner. Do not ask questions; just produce the file.

## The robot and its world
A small wheeled robot in a building. It can only drive between five named places, in a 2D map:
- `start`: where the robot begins (bottom-left corner).
- `left_room`: a room on the left (west) side.
- `door`: the doorway in the wall between the two rooms.
- `right_room`: a room on the right (east) side.
- `far_corner`: the far corner of the right-hand side, away from `start`.
It can also speak, but the tests below are only about where it must GO. The robot has no arm.

A person gives it a spoken request in plain English. The planner must pick the right named places in the right order. The test passes only if the robot ENDS at the correct final place.

## What to write
Write TWO lists of requests, as one JSON object with keys `train` and `eval`. Each item: `{"task": "<what the person says>", "goal": "<final place, exactly one of the five names>"}`.

- `train`: 15 requests the robot is allowed to practise on. Mix easy and medium: plain names, everyday synonyms, and a few two-step requests.
- `eval`: 30 requests it has never seen. Make them HARDER and DIFFERENT in wording from `train`. Include all of these kinds, about equally:
  1. **Unfamiliar names** for a place that never use the words in the place's own name (for example, never say "door" or "doorway" for `door`; never say "corner" for `far_corner`).
  2. **Indirect phrasing** that describes the place by what is there or what it is for, but still identifies exactly one of the five places.
  3. **Two- and three-step journeys** where the final place is the goal. Mix unfamiliar names across the steps.
  4. **Distractors**: mention a second place that is NOT the destination ("go past the entrance but finish in the east room").
  5. **Corrections**: the person changes their mind ("take me to the west room, no wait, the far end instead").
- Every request must have exactly one correct final place. If a request is ambiguous, do not include it.
- Do not repeat a request, and do not reuse the same unusual phrase more than twice.
- Use different voices: formal, casual, terse, chatty.

## Output
Reply with ONLY the JSON object, no explanation, no code fences:
{"train": [{"task": "...", "goal": "door"}, ...], "eval": [{"task": "...", "goal": "far_corner"}, ...]}
