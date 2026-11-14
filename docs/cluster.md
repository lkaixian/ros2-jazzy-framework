# Cluster platform: job placement, failover and load balancing over DDS

Run the framework on several machines (a laptop, a Raspberry Pi / Jetson that owns the robot core,
more laptops joining later) and let a scheduler decide **which machine runs which job**.

This is a **control plane only**. Topics, services and actions already flow directly between
machines over DDS, so the camera image, detections, `bt/plan`, `cmd_vel` ... never pass through the
scheduler. The platform decides placement, notices dead machines, and moves jobs; nothing else.

```
                      cluster/assignments  (desired state, transient_local, 1 Hz re-publish)
   +-----------+ ----------------------------------------------------+
   | scheduler |                                                     v
   | (exactly  | <-- cluster/node_info (heartbeats, 2 Hz) --  +---------------+   +---------------+
   |  one)     |                                              | agent: pi     |   | agent: laptop |
   +-----------+ --> cluster/status (for cluster_status)      | roles=core    |   | roles=compute |
                                                              | tags=camera.. |   | tags=gpu      |
                                                              |  supervisor:  |   |  supervisor:  |
                                                              |  odom, mux,.. |   |  llm, slm,..  |
                                                              +---------------+   +---------------+
```

* **agent** (one per machine, `cluster_agent_node`): sends heartbeats (cpu, memory, GPU memory,
  temperature, running jobs) and **reconciles**: starts the jobs assigned to it, stops the ones that
  no longer are. It only ever runs jobs from its **local catalog**.
* **scheduler** (one in the whole cluster, `cluster_scheduler_node`): reads heartbeats, computes the
  placement, publishes the **desired state** (a complete assignment set). Stateless between runs:
  after a restart it *adopts* what the heartbeats say is running.
* **job**: one or several processes that live and die together (e.g. `llama-server` + `llm_bridge`
  so the bridge's `localhost` URL stays valid). Defined in `param/cluster_jobs.yaml`.

## Quick start

```bash
# every machine: same workspace build, same param/ folder (the catalog hash is compared)
# robot controller (Pi / Jetson), owns the core jobs and the camera
ros2 launch framework_bringup cluster.launch.py role:=agent host_id:=pi roles:=core tags:=camera,audio

# a laptop with a GPU: agent + the one scheduler
ros2 launch framework_bringup cluster.launch.py role:=both host_id:=laptop1 roles:=compute tags:=gpu

# another laptop joins later: that is all it takes
ros2 launch framework_bringup cluster.launch.py role:=agent host_id:=laptop2 roles:=compute

ros2 run cluster_core cluster_status          # hosts, placements, recent decisions
```

Design a placement before you own the machines:

```bash
ros2 run cluster_core plan_preview --hosts hosts.yaml --jobs src/framework_bringup/param/cluster_jobs.yaml
ros2 run cluster_core plan_preview --hosts hosts.yaml --jobs ... --dead pi     # what happens if pi dies
```
```yaml
# hosts.yaml
hosts:
  pi:      {roles: core,    tags: "camera,audio", cores: 4,  cpu_load: 0.1, mem_total_mb: 4096,  mem_free_mb: 3000}
  laptop1: {roles: compute, tags: gpu,            cores: 12, cpu_load: 0.2, mem_total_mb: 16000, mem_free_mb: 9000, gpu_mem_free_mb: 3500}
```
The preview also prints a second pass: it must be identical (a placement must be stable).
Typical lesson: a 4-core board cannot hold every job the shipped catalog pins to camera/audio hosts
(`stt`, `tts`, three detectors); the planner leaves the overflow `pending` instead of overcommitting.

## Job catalog (`param/cluster_jobs.yaml`)

| field | meaning |
|---|---|
| `cmd` / `cmds` | argv list(s). **No shell**; a string is rejected. `cmds` = several processes of one job |
| `role` / `host` | hard pin: host must have the role / be that host |
| `tags` / `prefer_tags` | host must have all `tags`; `prefer_tags` is a soft bonus (e.g. `gpu`) |
| `cpu`, `mem_mb`, `gpu_mem_mb` | declared cost, used for fit and spreading |
| `movable: false` | **hardware-bound**: placed once, never migrated, never failed over to another machine, keeps running if the scheduler is away |
| `priority` | higher is placed first when capacity is short |
| `restart` | `max_restarts`, `window_sec`, `backoff_base`, `backoff_max` |
| `grace_sec` | SIGTERM -> SIGKILL delay |

`${PARAMS_DIR}` expands to the agent's param folder (any other `${VAR}` is an error: no environment
leakage). The catalog **must be identical on every machine**; its hash is compared.

## Placement rules

Hard: pinned host/role, required tags, resource fit (observed free CPU/memory/GPU memory minus the
cost of jobs newly placed in the same plan, plus the reserved cost of jobs assigned but not yet
running), host alive, same catalog, host not quarantined for that job. Soft: `prefer_tags`, spread by
free capacity. Then:

* **Stickiness**: a healthy job never moves because a better host appeared.
* **Failover**: only movable jobs, only after a host was silent for `failover_after_sec` (5 s).
* **Immovable jobs on a dead host** stay *assigned* to it and are reported `unplaceable`; they resume
  by themselves when the host returns. They are never started on another machine.
* **Quarantine**: a host that reports a job `FAILED` is avoided for that job for `quarantine_sec`.
* **Rebalancing**: a host above `overload` (0.85) for `hysteresis_sec` (10 s) sheds its biggest
  movable job to a host below `underload` (0.6); one migration per cycle, `cooldown_sec` (60 s) per job.
* Deterministic and idempotent (property-tested): same input, same output; re-planning a result
  changes nothing.

## Failure semantics (read this before trusting it)

| event | what happens |
|---|---|
| agent process dies (even SIGKILL) | its jobs die within ~0.1 s (`cluster_core.guard` notices the parent change and kills the job's process group). After `failover_after_sec` the scheduler restarts its **movable** jobs elsewhere. No duplicate instance exists at any time. |
| a machine is silent but alive (network partition) | the agent stops its movable jobs after `orphan_timeout_sec` (4 s) without an accepted assignment set. `failover_after_sec` (5 s) must exceed it by >= 0.5 s (the scheduler refuses to start otherwise): **fencing by timeout**, assuming clocks run at similar *rates* (not synchronised). |
| scheduler dies | agents keep **immovable** jobs running (fail-static); movable jobs stop after 4 s. A new scheduler adopts what is running. Its blackout (start + `adopt_wait_sec`) must stay well below 4 s: measured 1.6 s with the default 1 s wait. (It was 2 s once and measured 2.65 s: that passed on an idle machine but failed under load, because the old scheduler's last publish is also up to 1 s old. The node now warns if adopt_wait_sec > orphan timeout - 2.5 s.) If the scheduler is down longer, movable jobs stop and are started again afterwards. |
| scheduler frozen (VM pause, SIGSTOP) | on waking it notices its own stall and extends host liveness timers, so it does **not** declare the whole cluster dead. Measured: no failover, every job back on the same host. |
| a half partition (the agent hears the scheduler, the scheduler no longer hears the agent) | by design the agent follows the newest set, so overlap is bounded by DDS delivery rather than by a timeout. **Not tested.** |
| host with a different catalog | excluded by the scheduler; its agent refuses assignments and (after the orphan timeout) stops movable jobs. |

The agent runs its jobs through `cluster_core.guard`, so **killing the agent also kills the core
jobs on that machine** (they could otherwise outlive a dead agent and be duplicated). Run the agent
under a supervisor (systemd `Restart=always`) on the robot controller. The `cmd_vel_mux` watchdog
(0.5 s) stops motion when the odometry extension is gone; this platform is **not an e-stop** and gives
no hard real-time guarantee. Keep a hardware stop within reach.

The scheduler is a **single point of failure for changes** (not for running jobs). There is no
leader election: run exactly one; a second one is detected and logged but not prevented.

## Security

* Agents run **only commands from their own catalog**. An assignment is a list of job ids; unknown
  ids are ignored and logged, a set with a different catalog hash is refused. Remote code execution
  through the cluster topics is therefore not possible.
* DDS itself is **not authenticated** by default: anyone on the network and domain can publish an
  assignment set and start/stop *catalog* jobs (the integration test does exactly that with a rogue
  publisher). For anything beyond a lab, use SROS2 (signed/encrypted DDS), a dedicated
  `ROS_DOMAIN_ID`, and a network you trust.

## Network setup (2-3 machines)

1. Same ROS 2 Jazzy, same workspace build (`./setup.sh`), same `param/` folder on every machine
   (`diff` the catalog; the scheduler lists mismatching hosts as `EXCLUDED(catalog)`).
2. Same `ROS_DOMAIN_ID` everywhere; unique `host_id` per machine.
3. Discovery:
   * **Ethernet, same subnet**: multicast normally just works. `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.
   * **WiFi**: multicast is often dropped by access points. Use unicast peers:
     `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="192.168.1.10;192.168.1.11"`
     (list the OTHER machines), or a **Fast DDS discovery server**: on one machine
     `fastdds discovery --server-id 0`, on all `export ROS_DISCOVERY_SERVER=<that-ip>:11811`
     (restart the ROS daemon: `ros2 daemon stop`).
   * Check from each machine: `ros2 node list`, then `ros2 run cluster_core cluster_status`.
4. Do not send raw camera images over WiFi. The shipped catalog keeps the detectors on the host that
   has the camera (`tags: [camera]`); only detections cross the network. If a node must consume images
   remotely, use compressed transport and `BEST_EFFORT` QoS.
5. Time: the cluster itself uses only local receive times and needs no clock sync, but stamped
   sensor/TF data across machines does: run `chrony` on all of them.
6. RMW: tested on the Jazzy default. Large messages and many participants behave differently per RMW
   (Cyclone vs Fast DDS); if discovery is slow with many nodes, a discovery server helps.
7. Run the agent as a service on every machine, e.g. systemd `Restart=always` with
   `ExecStart=/bin/bash -lc 'source ~/ros2-jazzy-robocup-fw/env.sh && ros2 launch framework_bringup cluster.launch.py role:=agent host_id:=pi roles:=core tags:=camera,audio'`.

Changing the catalog: edit it on **every** machine, then restart agents (they read it at start) and
the scheduler. Machines with an old catalog are excluded until they are updated. Restarting an agent
stops its jobs.

## Parameters (`param/cluster.yaml`)

| key | default | meaning |
|---|---|---|
| agent `heartbeat_hz` | 2.0 | heartbeat rate |
| agent `orphan_timeout_sec` | 4.0 | no accepted set for this long: stop movable jobs |
| agent `min_rx_before_act` | 2 | act only after 2 receptions (ignores a stale latched set replayed to a fresh agent); 1 = obey immediately |
| agent `use_guard` | true | run jobs through the guard (die with the agent). `false` is for tests only |
| scheduler `failover_after_sec` | 5.0 | silent host = dead. Must be >= `agent_orphan_timeout_sec` + 0.5 |
| scheduler `adopt_wait_sec` | 1.0 | restart: listen, then adopt. Keep restart blackout < agent orphan timeout |
| scheduler `overload` / `underload` | 0.85 / 0.6 | rebalancing thresholds |
| scheduler `hysteresis_sec` / `cooldown_sec` | 10 / 60 | overload must persist / per-job minimum between migrations |
| scheduler `quarantine_sec` | 60 | avoid a host for a job that FAILED there |
| scheduler `settle_sec` | 10 | reserve the cost of freshly assigned jobs until they run |

`roles` and `tags` are **comma-separated strings** (empty ROS arrays are unreliable).
Job logs: `~/.ros/cluster/<host_id>/<job>.<n>.log`.

## Measured on ONE machine (simulated hosts, `it_cluster.py`)

Run: `export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; python3 src/cluster_core/test/integration/it_cluster.py`
(54 checks). Scheduler, three to four agents and the jobs are separate processes; the network is
loopback, so these are **control-loop** numbers, not WiFi numbers.

| measurement | result |
|---|---|
| heartbeat -> assignment published (6 joins, random phase of the 2 Hz tick) | min 0.09 / mean 0.28 / max 0.50 s |
| late agent joins -> its job RUNNING | 0.5 s |
| SIGKILL of an agent -> its job processes dead | 0.04-0.10 s |
| SIGKILL -> new assignment published (failover) | 5.1-5.3 s (= `failover_after_sec` + one plan tick) |
| SIGKILL -> all its jobs RUNNING elsewhere | 5.2-5.4 s |
| core host SIGKILL -> immovable job reported `unplaceable` | 4.9-5.4 s, never started elsewhere |
| core host back -> core jobs RUNNING | 1.4 s |
| scheduler restart: old stopped -> new publishes | 2.65 s; every job kept its pid |
| scheduler frozen 7 s -> all jobs back on the same hosts | no failover, core job pid unchanged |

Positive control: `--control-no-guard` (agents without the process guard) makes the duplicate-instance
and leftover-process checks fail as they must (a SIGKILLed agent left 2 live copies of a job).

**Not verified**: a second physical machine, a real network (WiFi/Ethernet latency, loss, multicast
behaviour), real GPU/Jetson metrics, load-based rebalancing on live hosts (unit-tested only, the
simulated hosts share one CPU), SROS2, and any real robot hardware. Failover time on a real network
is the same 5 s plus detection noise; discovery time of a joining machine is network dependent.

## Machine health service
Every `cluster_agent` serves `host_info/<host_id>` (`cluster_msgs/srv/HostInfo`, the host id is sanitised into a valid ROS name: other characters
become `_`, a leading digit gets an `h` prefix) and `cluster/node_info` carries the same numbers (CPU load, RAM, GPU util/VRAM, temperature, laptop
battery) at about 2 Hz. `ros2 run cluster_core host_info [host_id]` prints one line per machine. Time sync between hosts: `tools/net/timesync.sh`.


## Overview
`cluster_msgs` + `cluster_core`: one agent per machine, one scheduler, job catalog in `param/cluster_jobs.yaml`; places jobs by role/tags/capacity, fails movable jobs over, keeps hardware-bound jobs (`odom`, `pid`, `cmd_vel_mux`) on the `core` host, adopts state after a scheduler restart. Control plane only (data still flows directly over DDS). `ros2 launch framework_bringup cluster.launch.py role:=agent host_id:=pi roles:=core tags:=camera`, `ros2 run cluster_core cluster_status`, offline `plan_preview`. See `docs/cluster.md` (setup, failure semantics, security, measured numbers). Verified on one machine with simulated hosts only; not on a second machine or WiFi.

