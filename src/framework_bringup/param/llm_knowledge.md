Robot facts (read before planning):
- Kobuki TurtleBot2 base, speed limited to 0.3 m/s forward and 0.8 rad/s turning. It has a camera and a lidar.
- It cannot pick up, carry or hand over objects: there is no working arm. Never plan grasping.
- Named places (use with `NavigateToPlace`): start, left_room, door, right_room, far_corner.
- `NavigateToPlace` and `NavigateToPose` need a localized robot. After a blocked path run `ClearCostmaps` and retry once.
- `DetectObject` only knows the labels listed under "Object labels"; `DetectPerson` needs a person in view.
- The scene lists each visible person with distance and, when a skeleton is visible, `waving` / `pointing`. React with `IsPersonWaving`, `IsPersonPointing` or `RecognizeFace`; a person without those keys has no usable skeleton.
- `WaitForTranscript` listens for one spoken answer (put it right after a `Speak` that asks a question); `out_text="{x}"` hands the words to later steps.
- Numeric ports (`min_conf`, `timeout_ms`, `distance`, ...) take a plain number such as "0.5", never a comparison like ">0.5"; leave optional ports out when unsure.
- A question about whether someone or something is in a place ("is there anyone in the left room", "is there a cup at the door") is answered by going there and looking (`go_find_person`, `go_find_object`), never from memory.
- `Speak` is for short sentences. The robot stays silent unless a plan says `Speak`.
- Bumper or cliff contact stops the robot until a human resets it: a plan cannot recover from that, so do not plan retries around it.
- Plans normally run in dry-run first, so keep them short and make each step's purpose obvious.
Examples (request -> steps; copy the style, pick the macro that matches the words):
Example: go to the door and say hello -> [{"macro":"go_and_say","args":{"place":"door","text":"hello"}}]
Example: find a person in the left room and tell them hello -> [{"macro":"go_find_and_say","args":{"place":"left_room","text":"Hello"}}]
Example: follow me -> [{"macro":"follow_person"}]
Example: go home -> [{"macro":"return_home"}]
Example: bring me a cup -> [{"macro":"say","args":{"text":"Sorry, I have no arm and cannot bring objects"}}]
Example: guide the guest to the right room -> [{"macro":"escort_person_to","args":{"place":"right_room"}}]
Example: welcome the guest and ask their name and drink -> [{"macro":"meet_guest","args":{"slot":"guest1"}}]
Example: is there anyone in the right room -> [{"macro":"go_find_person","args":{"place":"right_room"}}]
Example: do you see a laptop -> [{"macro":"report_object","args":{"label":"laptop"}}]
Example: what is in front of you -> [{"macro":"scout_room"}]
Example: describe the guest -> [{"macro":"describe_person"}]
Example: look at me -> [{"macro":"look_at_person"}]
Example: come with me -> [{"macro":"follow_person"}]
Example: tell me when someone rings the bell -> [{"macro":"wait_for_doorbell"}]
Example: patrol between the door and the start 4 times -> [{"macro":"patrol_laps","args":{"first":"door","second":"start","laps":"4"}}]
Example: say hello at the start -> [{"macro":"go_and_say","args":{"place":"start","text":"hello"}}]
Example: go to the far corner and look for a backpack -> [{"macro":"go_find_object","args":{"place":"far_corner","label":"backpack"}}]
Example: take the orange to the door -> [{"macro":"say","args":{"text":"Sorry, I have no arm and cannot bring objects"}}]
Example: open the door -> [{"macro":"say","args":{"text":"Sorry, I have no arm and cannot open doors"}}]
Example: tell me how many people you see -> [{"node":"CountPeople","ports":{"out_count":"{n}"}},{"node":"Speak","ports":{"text":"I see {person} people","person":"{n}"}}]
Example: visit the left room, then the right room, then the door, saying hello in each -> [{"foreach":["left_room","right_room","door"],"as":"p","steps":[{"macro":"go_and_say","args":{"place":"${p}","text":"hello"}}]}]
Example: walk around for a while -> [{"macro":"wander","args":{"legs":"6"}}]
Example: look around the room and tell me how many people are there -> [{"macro":"scout_room"}]
