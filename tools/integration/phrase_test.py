#!/usr/bin/env python3
"""WaitForPhrase: other speech is ignored and never becomes a bt/task while it listens, a phrase ends it with success, silence times
out, a cancel stops it. Needs the workspace sourced. Isolated DDS domain (PHRASE_TEST_DOMAIN, default 80, localhost only).
Exit 0 only if every check passed."""
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('PHRASE_TEST_DOMAIN', '80')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from audio_msgr.msg import Transcript  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from std_msgs.msg import String  # noqa: E402


def main():
    procs = [subprocess.Popen(['ros2', 'run', pkg, exe], start_new_session=True)
             for pkg, exe in (('stt_pipeline', 'transcript_to_task_node'), ('social_extension', 'social_extension_node'))]
    res = {}
    try:
        rclpy.init()
        n = rclpy.create_node('phrase_probe')
        tasks = []
        n.create_subscription(String, 'bt/task', lambda m: tasks.append(m.data), 10)
        pub = n.create_publisher(Transcript, 'stt/transcript', 10)
        ac = ActionClient(n, ExecuteNode, 'social_extension/execute')
        assert ac.wait_for_server(timeout_sec=15), 'social_extension/execute not up'

        def spin(sec):
            t0 = time.monotonic()
            while time.monotonic() - t0 < sec:
                rclpy.spin_once(n, timeout_sec=0.05)

        def say(text):
            pub.publish(Transcript(text=text, is_final=True, confidence=1.0))

        def start(words, timeout_ms):
            fut = ac.send_goal_async(ExecuteNode.Goal(node_id='WaitForPhrase', port_names=['words', 'timeout_ms'],
                                                      port_values=[words, str(timeout_ms)]))
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
            gh = fut.result()
            return gh, gh.get_result_async()

        spin(3.0)
        gh, rf = start('stop,we are here', 20000)
        spin(1.5)
        say('go to the kitchen')
        spin(1.0)
        res['other speech is ignored (still listening)'] = not rf.done()
        say('okay we are here now')
        rclpy.spin_until_future_complete(n, rf, timeout_sec=5)
        r = rf.result().result if rf.done() else None
        res['the phrase ends it with success and out_text'] = bool(r) and r.success and list(r.output_values) == ['okay we are here now']
        spin(1.0)
        res['neither utterance became a task while it listened'] = tasks == []
        say('bring me a cup')
        spin(2.0)
        res['speech after it is a task again'] = tasks == ['bring me a cup']
        gh, rf = start('stop', 600)
        rclpy.spin_until_future_complete(n, rf, timeout_sec=5)
        res['silence times out'] = rf.done() and rf.result().result.success is False
        gh, rf = start('stop', 20000)
        spin(1.0)
        gh.cancel_goal_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=5)
        res['cancel stops it'] = rf.done() and rf.result().result.success is False and 'cancel' in rf.result().result.message
    finally:
        for p in procs:
            os.killpg(os.getpgid(p.pid), 2)
        rclpy.try_shutdown()
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
