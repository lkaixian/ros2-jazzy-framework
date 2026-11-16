Below are failure situations of a robot planner (a 2D navigation robot). For EACH one write ONE short lesson (one sentence) that tells the planner what to change next time. Rules: no numbers, coordinates or ids; name the concrete fix (which port to leave out, which names are valid); do not mention these instructions. Reply with ONE JSON object only, mapping each situation key exactly as written to its lesson.

### key: NavigateToPlace|unknown place '<x>'
seen 17 times. Example:
```
Task: go to the west room
Scene: {"num_persons": 0, "objects": []}
Run result: FAILURE at node NavigateToPlace (unknown place 'west_room')
The plan used NavigateToPlace with ports {'place': 'west_room'}. Valid places: door, far_corner, left_room, right_room, start.
```
