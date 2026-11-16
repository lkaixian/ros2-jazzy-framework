#!/usr/bin/env python3
"""bt_executor's dashboard feed: bt/tree (structure, latched) and bt/node_status ({"s": {uid: R|S|F|K|I}}).

A fake extension and the live executor run a 2-step plan (as in outputs_test.py); the test checks the tree has the right
shape and that the statuses end up SUCCESS for every node. Isolated DDS domain (TREE_TEST_DOMAIN, default 104).
Exit 0 only if every check passed."""
import json
import os
import subprocess
import sys
import tempfile
import threading
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('TREE_TEST_DOMAIN', '104')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_common.executor_node import ExtensionExecutor  # noqa: E402
from bt_pipeline_msgs.msg import BtPlan  # noqa: E402
from bt_slm.fallback import build_xml  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

MANIFEST = """extension: odom_extension
nodes:
  Step: {category: Action, required: [], optional: [tag], description: "test"}
"""


class Fake(ExtensionExecutor):
    def __init__(self):
        super().__init__('odom_extension')
        self.handlers = {'Step': lambda p, c: (c.sleep(0.3), 'ok')[1] and (True, 'ok')}


def main():
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(MANIFEST)
    exe = subprocess.Popen(['ros2', 'run', 'bt_executor', 'bt_executor_node', '--ros-args', '-p', 'dry_run:=false'],
                           start_new_session=True)
    res = {}
    ex = spin_thread = n = fake = None
    try:
        rclpy.init(args=['--ros-args', '-p', f'manifest_file:={f.name}'])
        fake = Fake()
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(fake)
        spin_thread = threading.Thread(target=ex.spin, daemon=True)
        spin_thread.start()
        n = rclpy.create_node('tree_probe')
        status, tree, nodes = [], [], []
        n.create_subscription(String, 'bt/status', lambda m: status.append(json.loads(m.data)), 10)
        n.create_subscription(String, 'bt/tree', lambda m: tree.append(json.loads(m.data)), QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(String, 'bt/node_status', lambda m: nodes.append(json.loads(m.data)['s']), 10)
        pub = n.create_publisher(BtPlan, 'bt/plan', 10)
        cli = n.create_client(Trigger, 'bt_executor/run')
        spec = {'goal': 'feed', 'steps': [{'node': 'Step', 'ports': {'tag': 'a'}}, {'node': 'Step', 'ports': {'tag': 'b'}}]}
        plan = BtPlan(task='t', valid=True, xml=build_xml(json.dumps(spec)))
        end, sent = time.monotonic() + 60, False
        while time.monotonic() < end and not any(s['state'] == 'finished' for s in status):
            rclpy.spin_once(n, timeout_sec=0.2)
            fin = [s for s in status if s['state'] == 'finished']
            if fin and fin[-1]['message'].startswith('ERROR') and 'not recognized' in fin[-1]['message']:
                # slow machine: the executor had not received the extension's manifest yet (republished every 3 s): try again
                status.clear()
                sent = False
                time.sleep(3.5)
            if not any(s['state'] == 'plan_received' for s in status):
                pub.publish(plan)
            elif not sent and cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
                sent = fut.done() and fut.result().success
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:     # topics are independent: the last node_status may arrive after bt/status 'finished'
            rclpy.spin_once(n, timeout_sec=0.1)
        t = tree[-1] if tree else []
        res['tree has root Sequence + 2 Step children with their ports'] = (
            len(t) == 3 and t[0]['name'] == 'Sequence' and t[0]['parent'] == -1
            and [x['name'] for x in t[1:]] == ['Step', 'Step'] and [x['ports'].get('tag') for x in t[1:]] == ['a', 'b']
            and all(x['depth'] == 1 and x['parent'] == t[0]['uid'] for x in t[1:]))
        seen = {}
        for s in nodes:
            seen.update(s)
        res['statuses were published and every node ended SUCCESS'] = bool(t) and all(seen.get(str(x['uid'])) == 'S' for x in t)
        res['a node was RUNNING at some point (live feed, not only the end)'] = any('R' in s.values() for s in nodes)
        print('tree:', json.dumps(t)[:300], '\nupdates:', len(nodes))
    finally:
        os.killpg(os.getpgid(exe.pid), 15)
        os.unlink(f.name)
        # stop the executor and wait for its thread BEFORE shutting rclpy down: shutting down under a spinning
        # executor aborted the process at exit on slow CI machines ("terminate called without an active exception")
        if ex is not None:
            ex.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=5)
        for node in (n, fake):
            if node is not None:
                node.destroy_node()
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
