# Dashboard, Groot2 and machine health

Three ways to see what the robot is doing, from a terminal glance to a graphical tree.

## Terminal dashboard (`dash`)

```bash
ros2 run framework_bringup dash                 # 1 Hz full-screen view
ros2 run framework_bringup dash --once          # print one frame and exit (good for logs and CI)
ros2 run framework_bringup dash --logs --level warn --node llm_bridge   # filtered /rosout stream
```

What one frame shows:

| Section | Content |
|---|---|
| Header | `ROS_DOMAIN_ID`, SROS2 state |
| LLM | each endpoint with link type (this machine's side of the route) and TCP time in ms; `llm/status` |
| Robot | Kobuki battery (volts, level, charging), robot mode |
| Task | `task/state`: state, task text, repairs used |
| **Plan tree** | the running tree with live node status |
| Cluster | jobs and where they run |
| Nodes | every node ONLINE / BUSY / ERROR / OFFLINE |

The plan tree comes from `bt_executor` (`bt/tree` is latched JSON with uid, depth, parent, type, name, ports; `bt/node_status` is `{"s": {uid: R|S|F|K|I}}` published on change). Symbols: `>` running, `+` done, `x` failed, `-` halted, `.` not reached. Statuses are **sticky** on purpose (done and failed stay after a Sequence resets its children); a `dash` started after a run still shows the last tree.

| Key | Action |
|---|---|
| `j` / `k`, arrows | select a node |
| `r` / `R` | restart the selected node (graceful / force; launch files respawn perception and extension nodes) |
| `s` / `S` | stop the whole system (asks y/n) |
| `w` / `W` | restart the whole system (asks y/n) |
| `h` | halt the task (`bt_executor/halt`) |
| `t` | hide / show the tree |

## Groot2 (the BehaviorTree.CPP GUI)

```bash
ros2 launch framework_bringup pipeline.launch.py groot2:=true      # or groot2: true in param/pipeline.yaml
~/Groot2/groot2.sh                                                 # Monitor tab, connect to 127.0.0.1 port 1667
```

The executor serves each running tree through `BT::Groot2Publisher` on `groot2_port` (1667); node colours follow the run. The server lives for one plan, so reconnect for the next one. To edit trees with the extension nodes in the palette, set `nodes_model_path: /tmp/bt_nodes_model.xml` (written at each run) and import it in Groot2. Tested: the port opens and the palette is written (`tools/integration/groot2_test.py`); not tried with the Groot2 GUI itself.

## Machine health

```bash
ros2 run cluster_core host_info [host_id]
```

Prints CPU, RAM, GPU utilisation and VRAM, temperature and battery of every machine that runs a `cluster_agent` (service `host_info/<host_id>`, type `cluster_msgs/srv/HostInfo`; the same numbers ride the roughly 2 Hz `cluster/node_info` heartbeat). A desktop PC reports battery `-`; no NVIDIA driver reports GPU `-`.

## Preflight and recording

```bash
ros2 run framework_bringup preflight        # PASS/WARN/FAIL: key, SROS2, mux, executor dry-run, LLM, /scan, TF, clock, disk
tools/record_run.sh                          # rosbag2 of the run topics
```
