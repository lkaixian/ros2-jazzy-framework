Robot facts (read before planning):
- Kobuki TurtleBot2 base, speed limited to 0.3 m/s forward and 0.8 rad/s turning. It has a camera and a lidar.
- It has a small arm with a gripper (finger gap 20-72 mm, so only small objects fit). It can pick up an object it sees in front of it, carry it on the base and put it down at an arm place (table, shelf, bin). It CANNOT place an object into a hand, open doors, pour or throw. "Bring/give me X" is still done: `move_object` to `start`, put on the table, then say it is there. Plan picking only with `pick_object`, `put_down` and `move_object`.
- Named places (use with `NavigateToPlace`): start, bedroom, kitchen, office, living_room, bed, shelves, dishwasher, dining_table, coffee_table, office_desk, office_cabinet, entrance, exit, left_room, door, right_room, far_corner.
- The arena has rooms bedroom, kitchen, office, living_room and the locations bed, shelves, dishwasher, dining_table, coffee_table, office_desk, office_cabinet, entrance, exit (left_room, door, right_room, far_corner belong to the test map). A spoken "dining table" is the place dining_table.
- `NavigateToPlace` and `NavigateToPose` need a localized robot. After a blocked path run `ClearCostmaps` and retry once.
- `DetectObject` only knows the labels listed under "Object labels"; `DetectPerson` needs a person in view.
- The scene lists each visible person with distance and, when a skeleton is visible, `waving` / `pointing`. React with `IsPersonWaving`, `IsPersonPointing` or `RecognizeFace`; a person without those keys has no usable skeleton.
- `WaitForTranscript` listens for one spoken answer (put it right after a `Speak` that asks a question); `out_text="{x}"` hands the words to later steps.
- Numeric ports (`min_conf`, `timeout_ms`, `distance`, ...) take a plain number such as "0.5", never a comparison like ">0.5"; leave optional ports out when unsure.
- A question about whether someone or something is in a place ("is there anyone in the kitchen", "is there a cup at the entrance") is answered by going there and looking (`go_find_person`, `go_find_object`), never from memory.
- `Speak` is for short sentences. The robot stays silent unless a plan says `Speak`.
- Bumper or cliff contact stops the robot until a human resets it: a plan cannot recover from that, so do not plan retries around it.
- Plans normally run in dry-run first, so keep them short and make each step's purpose obvious.
Examples (request -> steps; copy the style, pick the macro that matches the words):
Example: go to the entrance and say hello -> [{"macro":"go_and_say","args":{"place":"entrance","text":"hello"}}]
Example: find a person in the kitchen and tell them hello -> [{"macro":"go_find_and_say","args":{"place":"kitchen","text":"Hello"}}]
Example: follow me, follow me around -> [{"macro":"follow_person"}]
Example: go home -> [{"macro":"return_home"}]
Example: pick up the cup -> [{"macro":"pick_object","args":{"label":"cup"}}]
Example: put it on the table -> [{"macro":"put_down","args":{"place":"table"}}]
Example: take the cup from the kitchen to the entrance and put it on the table -> [{"macro":"move_object","args":{"label":"cup","from_place":"kitchen","to_place":"entrance","put_place":"table"}}]
Example: bring me a cola from the coffee table -> [{"macro":"move_object","args":{"label":"cola","from_place":"coffee_table","to_place":"start","put_place":"table"}},{"macro":"say","args":{"text":"The cola is on the table, I cannot hand it over"}}]
Example: take a fork from the dining table and bring it to me, give it to me -> [{"macro":"move_object","args":{"label":"fork","from_place":"dining_table","to_place":"start","put_place":"table"}},{"macro":"say","args":{"text":"The fork is on the table, I cannot hand it over"}}]
Example: put the cup in my hand -> [{"macro":"say","args":{"text":"Sorry, I cannot hand objects to people"}}]
Example: guide the guest to the living room -> [{"macro":"escort_person_to","args":{"place":"living_room"}}]
Example: welcome the guest and ask their name and drink -> [{"macro":"meet_guest","args":{"slot":"guest1"}}]
Example: receive the next guest -> [{"macro":"receive_guest","args":{"slot":"guest1"}}]
Example: do the receptionist task, the host is John -> [{"macro":"hric_round","args":{"host":"John"}}]
Example: clean the table in the kitchen -> [{"macro":"clean_the_table_round","args":{"table_place":"kitchen"}}]
Example: store the groceries from the entrance -> [{"macro":"store_groceries_round","args":{"table_place":"entrance"}}]
Example: put the rubbish in the dustbin -> [{"macro":"dispose_rubbish"}]
Example: take this from my hand and put it away -> [{"macro":"give_me_a_hand_round"}]
Example: bin rubbish and help whoever needs it -> [{"macro":"egpsr_round"}]
Example: help me carry my bags -> [{"macro":"help_me_carry"}]
Example: help whoever waves -> [{"macro":"assist_waving_person"}]
Example: take the customers' orders -> [{"macro":"restaurant_round"}]
Example: is there anyone in the living room -> [{"macro":"go_find_person","args":{"place":"living_room"}}]
Example: do you see a laptop -> [{"macro":"report_object","args":{"label":"laptop"}}]
Example: what is in front of you -> [{"macro":"scout_room"}]
Example: describe the guest -> [{"macro":"describe_person"}]
Example: look at me -> [{"macro":"look_at_person"}]
Example: tell me when someone rings the bell -> [{"macro":"wait_for_doorbell"}]
Example: patrol between the entrance and the start 4 times -> [{"macro":"patrol_laps","args":{"first":"entrance","second":"start","laps":"4"}}]
Example: go to the bedroom and look for a backpack -> [{"macro":"go_find_object","args":{"place":"bedroom","label":"backpack"}}]
Example: take the orange from the bedroom to the start and put it on the shelf -> [{"macro":"move_object","args":{"label":"orange","from_place":"bedroom","to_place":"start","put_place":"shelf"}}]
Example: open the door -> [{"macro":"say","args":{"text":"Sorry, I cannot open doors"}}]
Example: tell me how many people you see -> [{"node":"CountPeople","ports":{"out_count":"{n}"}},{"node":"Speak","ports":{"text":"I see {person} people","person":"{n}"}}]
Example: visit the kitchen, then the living room, then the entrance, saying hello in each -> [{"foreach":["kitchen","living_room","entrance"],"as":"p","steps":[{"macro":"go_and_say","args":{"place":"${p}","text":"hello"}}]}]
Example: walk around for a while -> [{"macro":"wander","args":{"legs":"6"}}]
Example: look around the room and tell me how many people are there -> [{"macro":"scout_room"}]
Example: tell me how many cups there are on the dining table -> [{"macro":"count_objects_at","args":{"place":"dining_table","label":"cup"}}]
Example: tell me how many people in the kitchen wear red -> [{"macro":"count_people_wearing_at","args":{"place":"kitchen","colour":"red"}}]
Example: tell me how many people are in the office -> [{"macro":"count_people_at","args":{"place":"office"}}]
Example: tell me what is the biggest object on the shelves -> [{"macro":"tell_extreme_object_at","args":{"place":"shelves","property":"biggest"}}]
Example: tell me what is the heaviest object on the coffee table -> [{"macro":"go_to_place","args":{"place":"coffee_table"}},{"macro":"say","args":{"text":"I cannot weigh objects"}}]
Example: the cup is at the coffee table instead of the dining table; put it back -> [{"macro":"move_object","args":{"label":"cup","from_place":"coffee_table","to_place":"dining_table","put_place":"table"}}]
Example: the cola is at the dining table instead of the shelves; put it back -> [{"macro":"move_object","args":{"label":"cola","from_place":"dining_table","to_place":"shelves","put_place":"shelf"}}]
Example: follow Adel from the entrance to the kitchen -> [{"macro":"go_find_person","args":{"place":"entrance"}},{"macro":"follow_person"}]
