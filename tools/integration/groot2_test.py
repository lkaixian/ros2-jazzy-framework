#!/usr/bin/env python3
"""bt_executor with groot2:=true: while a tree runs, port 1667 is open (Groot2 Monitor) and nodes_model_path holds the node palette.
Own DDS domain, no other node needed. Run with the workspace sourced: python3 tools/integration/groot2_test.py"""
import os
import signal
import socket
import subprocess
import sys
import time

import rclpy
from bt_pipeline_msgs.msg import BtPlan

XML = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"><Sequence>'
       '<Sleep msec="3000"/></Sequence></BehaviorTree></root>')
PORT, MODEL = 18667, '/tmp/groot2_test_model.xml'


def open_port(port):
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(('127.0.0.1', port)) == 0


def main():
    os.environ['ROS_DOMAIN_ID'] = '87'
    ex = subprocess.Popen(['ros2', 'run', 'bt_executor', 'bt_executor_node', '--ros-args', '-p', 'groot2:=true', '-p', f'groot2_port:={PORT}',
                           '-p', f'nodes_model_path:={MODEL}', '-p', 'auto_run:=true'], start_new_session=True)
    ok = False
    try:
        rclpy.init()
        n = rclpy.create_node('groot2_test')
        pub = n.create_publisher(BtPlan, 'bt/plan', 10)
        end = time.monotonic() + 30
        while time.monotonic() < end and pub.get_subscription_count() < 1:
            rclpy.spin_once(n, timeout_sec=0.2)
        pub.publish(BtPlan(task='groot2 test', xml=XML, valid=True))
        seen = False
        end = time.monotonic() + 8
        while time.monotonic() < end and not seen:
            rclpy.spin_once(n, timeout_sec=0.1)
            seen = open_port(PORT)
        ok = seen and os.path.exists(MODEL) and '<TreeNodesModel' in open(MODEL).read()
        print(f"groot2 port open while running: {seen}; palette file: {os.path.exists(MODEL)}")
    finally:
        os.killpg(ex.pid, signal.SIGTERM)
        rclpy.try_shutdown()
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
