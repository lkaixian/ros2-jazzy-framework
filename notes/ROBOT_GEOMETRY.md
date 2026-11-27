# Robot geometry (hand-measured; none of it ruler-verified except where noted)

Frame: base_link = the Kobuki centre, x forward (bumper side), y left, z up from the Kobuki base. Arm frame = the arm's joint-1 axis on the plate it bolts to, same axes.

| What | Value | Source / confidence |
|---|---|---|
| Arm joint-1 axis, in base_link | x ~ +0.16 m, y ~ 0 | user, "best effort" 2026-11-25; "pretty centred" left-right |
| Arm base plate height | ~0.78-0.80 m above the Kobuki base | FK from the rest pose (+-3 cm); user 2026-11-26: lens 3 cm and plate 5 cm above the same surface, so the plate is ~2 cm above the RealSense lens |
| Robot height, arm folded (rest pose) | top of the OAK ~0.90 m; gripper ~0.875 m | user-measured 2026-11-25 |
| **RealSense lens centre height** | **0.755 m above the Kobuki base** | user-measured 2026-11-26 |
| RealSense, relative to the arm axis | dx ~ 0 (same line across the robot), 0.08 m to the RIGHT, ~0.02 m below the plate | user 2026-11-26. Cross-check: the two cameras agree on x within 0.7 cm +- 2 cm |
| RealSense, in base_link | x ~ +0.16, y ~ -0.08, z ~ 0.755 | derived from the two rows above. Consistent with the user's "~19 cm from the centre, at the Kobuki's radius": sqrt(0.16^2 + 0.08^2) = 0.18 m |
| RealSense direction | forward, parallel to the arm's rest direction | user 2026-11-26. **Tilt not measured: assume level until someone checks with a level/phone** |
| Mounted fixed to the base? | yes, bolted next to the arm base | user. The arm does not carry it |
| Side of the arm | right-hand side looking forward | user 2026-11-26. An older note (ARM_BRINGUP / teach survey) says "~90 deg left of the arm axis": one of them is wrong |

Body: the RealSense blocks arm joint 1 between +1.28 and +1.87 rad; the folded gripper overhangs ~11 cm ahead of the base. Depth: the arm camera (OAK) cannot measure closer than ~0.36 m; the RealSense is the locator for anything nearer.
