# Workspaces and security (SROS2)

## Workspaces and security (SROS2)
A workspace = an SROS2 keystore (authentication + encryption, `Enforce`) + a DDS domain + an LLM API key + a join token.
```
ros2 run framework_bringup ws create lab             # first machine: prints the invite
source ~/.robocup/workspaces/lab/env.sh              # every terminal that should be inside it
ros2 run framework_bringup ws serve lab              # admits machines, announces itself on the LAN
ros2 run framework_bringup ws discover               # other machine: what is announced here?
ros2 run framework_bringup ws join lab --token ...   # then: source $(ros2 run framework_bringup ws env lab)
```
Join runs over TLS; both sides prove they know the token with an HMAC bound to the certificate the client saw, so a wrong token, an impostor server and a relaying man-in-the-middle all get nothing (tested). Outsiders and nodes holding another workspace's keys cannot see or talk to workspace nodes (tested). **Roles:** `ws invite lab --role observer|operator` prints an invite with its own token; the member's SROS2 enclave then only allows that role's topics (`observer` reads everything and publishes only ROS housekeeping, `operator` also publishes `bt/task` and `teleop/cmd_vel`; DDS Security enforces it, tested: an observer reads, its writer is refused). **Revocation:** `ws revoke lab HOST` removes the member, rotates all tokens and the LLM key and re-keys the workspace (new CA), so the old keys stop working; the other members re-run `ws join`. **Limits:** tested on one machine only; roles are fixed in `ws.py` (`ROLES`), not configurable per topic; after `revoke` restart llama-server with the new key; the owner cannot be revoked. `tools/net/cyclonedds_lan.xml` is a multicast-off peer-list discovery config. Details: `notes/RUNBOOK.md`, `notes/technical_debt.md` T-03.
