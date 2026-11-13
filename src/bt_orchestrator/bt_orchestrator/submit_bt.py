"""submit_bt FILE.xml [--run]: validate a hand-written BT and hand it to bt_executor.

The XML goes through the same bt_validator service as generated plans; an invalid tree is
printed and never published. --run then calls bt_executor/run (still dry-run unless the
executor was launched with dry_run:=false).
"""
import argparse
import os
import sys
import time

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from bt_pipeline_msgs.srv import ValidateTree
from std_srvs.srv import Trigger


def _call(node, client, req, timeout):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('file')
    ap.add_argument('--run', action='store_true', help='call bt_executor/run afterwards')
    ap.add_argument('--timeout', type=float, default=5.0)
    a = ap.parse_args(argv)
    try:
        with open(a.file) as f:
            xml = f.read()
    except OSError as e:
        print(f'cannot read {a.file}: {e}')
        return 2
    rclpy.init()
    node = rclpy.create_node('submit_bt')
    try:
        val = node.create_client(ValidateTree, 'bt_validator/validate')
        if not val.wait_for_service(timeout_sec=a.timeout):
            print('bt_validator is not running (ros2 launch bt_validator bt_validator.launch.py)')
            return 2
        res = _call(node, val, ValidateTree.Request(xml=xml), a.timeout)
        if res is None:
            print('validator did not answer')
            return 2
        if not res.valid:
            print(f'{a.file}: INVALID, not submitted')
            for e in res.errors:
                print(f'  - {e}')
            return 1
        pub = node.create_publisher(BtPlan, 'bt/plan', 10)
        end = time.monotonic() + a.timeout
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print('nobody subscribes to bt/plan (is bt_executor running?)')
            return 2
        plan = BtPlan(task='file:' + os.path.basename(a.file), xml=xml, valid=True)
        plan.header.stamp = node.get_clock().now().to_msg()
        pub.publish(plan)
        print(f'{a.file}: OK, submitted to bt/plan')
        if a.run:
            run = node.create_client(Trigger, 'bt_executor/run')
            if not run.wait_for_service(timeout_sec=a.timeout):
                print('bt_executor/run unavailable')
                return 2
            time.sleep(0.3)      # let the executor store the plan first
            r = _call(node, run, Trigger.Request(), a.timeout)
            print(f'run: {r.message if r else "no answer"}')
            return 0 if r and r.success else 1
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
