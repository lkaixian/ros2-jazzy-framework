#!/usr/bin/env python3
"""WaitForTranscript + the stt/capture flag: while a node listens for an answer, transcript_to_task must NOT turn
the answer into a bt/task; afterwards it must again. Needs the workspace sourced. Isolated DDS domain
(LISTEN_TEST_DOMAIN, default 79, localhost only). Exit 0 only if every check passed."""
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('LISTEN_TEST_DOMAIN', '79')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from audio_msgr.msg import Transcript  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def say(pub, text):
    pub.publish(Transcript(text=text, is_final=True, confidence=1.0))


def main():
    procs = [subprocess.Popen(['ros2', 'run', pkg, exe], start_new_session=True)
             for pkg, exe in (('stt_pipeline', 'transcript_to_task_node'),
                              ('social_extension', 'social_extension_node'))]
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('listen_probe')
        tasks = []
        n.create_subscription(String, 'bt/task', lambda m: tasks.append(m.data), 10)
        pub = n.create_publisher(Transcript, 'stt/transcript', 10)
        ac = ActionClient(n, ExecuteNode, 'social_extension/execute')
        assert ac.wait_for_server(timeout_sec=15), 'social_extension/execute not up'
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)      # let discovery settle
        fut = ac.send_goal_async(ExecuteNode.Goal(node_id='WaitForTranscript',
                                                  port_names=['timeout_ms'], port_values=['8000']))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        gh = fut.result()
        rfut = gh.get_result_async()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:       # listener is up and the flag has propagated
            rclpy.spin_once(n, timeout_sec=0.1)
        say(pub, 'my name is alice')
        rclpy.spin_until_future_complete(n, rfut, timeout_sec=8)
        r = rfut.result().result
        res['answer returned as out_text'] = (r.success, list(r.output_names), list(r.output_values)) == (
            True, ['out_text'], ['my name is alice'])
        end = time.monotonic() + 1
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        res['answer did not become a task'] = tasks == []
        say(pub, 'bring me a cup')
        end = time.monotonic() + 3
        while time.monotonic() < end and not tasks:
            rclpy.spin_once(n, timeout_sec=0.1)
        res['later speech is a task again'] = tasks == ['bring me a cup']
        fut = ac.send_goal_async(ExecuteNode.Goal(node_id='WaitForTranscript',
                                                  port_names=['timeout_ms'], port_values=['500']))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        rfut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(n, rfut, timeout_sec=5)
        res['silence times out'] = rfut.result().result.success is False
    finally:
        for p in procs:
            os.killpg(os.getpgid(p.pid), 2)
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
