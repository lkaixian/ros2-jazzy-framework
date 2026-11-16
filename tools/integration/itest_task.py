#!/usr/bin/env python3
"""Send one task to a running pipeline and record bt/status (used by itest.sh).

usage: itest_task.py STATUS_FILE [HALT_AFTER_SEC|none]

Replaces `ros2 topic pub --once` + `ros2 topic echo`, which raced DDS discovery on slow machines:
a volatile message published before the subscriber matched is simply lost. This waits for both
bt/task subscribers (bridge, orchestrator), publishes, and republishes (max 3x) until the plan is
received; stops publishing at once after that so a slow reply can never start a second run.
"""
import json
import os
import sys
import time

import rclpy
from bt_pipeline_msgs.action import ExecuteNode
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger

out_path = sys.argv[1]
halt_after = None if len(sys.argv) < 3 or sys.argv[2] == 'none' else float(sys.argv[2])
rclpy.init()
n = rclpy.create_node('itest_task')
log = []
n.create_subscription(String, 'bt/status', lambda m: log.append(m.data), 10)
pub = n.create_publisher(String, 'bt/task', 10)
halt = n.create_client(Trigger, 'bt_executor/halt')
# extensions start in parallel with the pipeline: a plan sent before their action server exists
# would sit in the executor until its goal timeout
servers = [ActionClient(n, ExecuteNode, a) for a in
           os.environ.get('ITEST_ACTION_SERVERS', 'odom_extension/execute').split(',') if a]


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def saw(state):
    return any(json.loads(s).get('state') == state for s in log)


end = time.monotonic() + 120
while time.monotonic() < end and not (pub.get_subscription_count() >= 2 and halt.service_is_ready()
                                      and all(c.server_is_ready() for c in servers)):
    spin(0.2)
spin(float(os.environ.get('ITEST_SETTLE', '4.0')))     # manifests are republished every 3 s: wait longer than that so the executor and validator know every node
for _ in range(3):
    pub.publish(String(data='do it'))
    t = time.monotonic() + 15
    while time.monotonic() < t and not saw('plan_received'):
        spin(0.1)
    if saw('plan_received'):
        break
if halt_after is not None:
    spin(halt_after)
    halt.call_async(Trigger.Request())
t = time.monotonic() + 40
while time.monotonic() < t and not saw('finished'):
    spin(0.1)
spin(1.0)
with open(out_path, 'w') as f:
    f.write('\n---\n'.join(log) + '\n')
rclpy.try_shutdown()
