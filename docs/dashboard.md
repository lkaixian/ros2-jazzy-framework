# Dashboard and machine health

## Dashboard
```
ros2 run framework_bringup dash            # 1 Hz: domain + SROS2 state, LLM endpoints with link type and TCP ms, Kobuki battery, task state, the plan tree with live node status, cluster jobs, every node ONLINE/BUSY/ERROR/OFFLINE
ros2 run framework_bringup dash --logs --level warn --node llm_bridge     # verbose /rosout stream
```
The **plan tree** comes from `bt_executor` (`bt/tree` + `bt/node_status`): `>` running, `+` done, `x` failed, `-` halted, `.` not reached; `t` hides it. Keys: `j/k` select a node, `r/R` restart it (graceful/force; the launch files respawn it), `s/S` stop the system, `w/W` restart it, `h` halt the task; `s S w W` ask y/n. The link type describes this machine's side of the route to each endpoint. Everything an operator types is in `docs/operator-guide.md`; `ros2 run framework_bringup teleop` takes over with the keyboard (highest `cmd_vel_mux` priority, halts the plan on the first key).

## Machine health
`ros2 run cluster_core host_info [host_id]` prints CPU, RAM, GPU utilisation and VRAM, temperature and battery of every machine that runs a `cluster_agent` (service `host_info/<host_id>`, type `cluster_msgs/srv/HostInfo`; the same numbers are in the ~2 Hz `cluster/node_info` heartbeat). A desktop PC reports battery `-`; no NVIDIA driver reports GPU `-`.
