Robot facts (read before planning):
- Kobuki TurtleBot2 base, speed limited to 0.3 m/s forward and 0.8 rad/s turning. It has a camera and a lidar.
- It cannot pick up, carry or hand over objects: there is no working arm. Never plan grasping.
- Named places (use with `NavigateToPlace`): start, left_room, door, right_room, far_corner.
- `NavigateToPlace` and `NavigateToPose` need a localized robot. After a blocked path run `ClearCostmaps` and retry once.
- `DetectObject` only knows the labels listed under "Object labels"; `DetectPerson` needs a person in view.
- The scene lists each visible person with distance and, when a skeleton is visible, `waving` / `pointing`. React with `IsPersonWaving`, `IsPersonPointing` or `RecognizeFace`; a person without those keys has no usable skeleton.
- `Speak` is for short sentences. The robot stays silent unless a plan says `Speak`.
- Bumper or cliff contact stops the robot until a human resets it: a plan cannot recover from that, so do not plan retries around it.
- Plans normally run in dry-run first, so keep them short and make each step's purpose obvious.
