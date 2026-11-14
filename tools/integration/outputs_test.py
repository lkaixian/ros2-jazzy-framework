#!/usr/bin/env python3
"""Round trip of node outputs (B-01): a fake extension returns {"out_text": ...} from node Produce,
the live bt_executor writes it to the blackboard, and node Consume receives it as its `text` port.

Needs the workspace sourced. Isolated DDS domain (OUTPUTS_TEST_DOMAIN, default 78, localhost only).
Exit 0 only if Consume saw the value Produce returned.
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('OUTPUTS_TEST_DOMAIN', '78')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from bt_common.executor_node import ExtensionExecutor  # noqa: E402
from bt_pipeline_msgs.msg import BtPlan  # noqa: E402
from bt_slm.fallback import build_xml  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
import json  # noqa: E402

MANIFEST = """extension: odom_extension
nodes:
  Produce: {category: Action, required: [], optional: [out_text], description: "test"}
  Consume: {category: Action, required: [text], optional: [], description: "test"}
"""
seen = []


class Fake(ExtensionExecutor):
    def __init__(self):
        super().__init__('odom_extension')    # reuse a real package name for the default path only
        self.handlers = {'Produce': lambda p, c: (True, 'made', {'out_text': 'hello 42'}),
                         'Consume': lambda p, c: (seen.append(p.get('text')) or True, 'ok')}


def main():
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(MANIFEST)
    exe = subprocess.Popen(['ros2', 'run', 'bt_executor', 'bt_executor_node', '--ros-args',
                            '-p', 'dry_run:=false'], start_new_session=True)
    rc = 1
    try:
        rclpy.init(args=['--ros-args', '-p', f'manifest_file:={f.name}'])
        fake = Fake()
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(fake)
        threading.Thread(target=ex.spin, daemon=True).start()
        n = rclpy.create_node('outputs_probe')
        status = []
        n.create_subscription(String, 'bt/status', lambda m: status.append(json.loads(m.data)), 10)
        pub = n.create_publisher(BtPlan, 'bt/plan', 10)
        cli = n.create_client(Trigger, 'bt_executor/run')
        spec = {'goal': 'outputs', 'steps': [{'node': 'Produce', 'ports': {'out_text': '{v}'}},
                                              {'node': 'Consume', 'ports': {'text': '{v}'}}]}
        plan = BtPlan(task='t', valid=True, xml=build_xml(json.dumps(spec)))
        end = time.monotonic() + 60
        sent = False
        while time.monotonic() < end and not any(s['state'] == 'finished' for s in status):
            rclpy.spin_once(n, timeout_sec=0.2)
            fin = [s for s in status if s['state'] == 'finished']
            if fin and fin[-1]['message'].startswith('ERROR') and 'not recognized' in fin[-1]['message']:
                # slow machine: the executor had not received the extension's manifest yet (republished every 3 s): try again
                status.clear()
                sent = False
                time.sleep(3.5)
            if not any(s['state'] == 'plan_received' for s in status):
                pub.publish(plan)          # republish until the executor has the manifest + plan
            elif not sent and cli.wait_for_service(timeout_sec=1.0):
                f2 = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(n, f2, timeout_sec=5)
                sent = f2.done() and f2.result().success
        fin = [s for s in status if s['state'] == 'finished']
        ok = bool(fin) and seen == ['hello 42']
        print('finished:', fin[-1] if fin else None, '| Consume saw:', seen)
        print('PASS' if ok else 'FAIL')
        rc = 0 if ok else 1
    finally:
        os.killpg(os.getpgid(exe.pid), 2)
        os.unlink(f.name)
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
