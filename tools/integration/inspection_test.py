#!/usr/bin/env python3
"""Robot inspection (3 Dec): navigate to a predefined place in the arena within 3 minutes, in the simulator (nav_sim + real Nav2/AMCL + the
safety mux, no robot). The mux gets safety_events_enabled=false (a copy of the param folder) because the simulator has no Kobuki bumper messages;
that is the only change. usage: inspection_test.py [goal_x goal_y] [limit_sec]   (default: right_room 9.5 5.5 through the door, 180 s).
Domain INSPECT_DOMAIN (default 85). Exit 0 only if the goal was reached within 0.35 m inside the limit. NOT a test of obstacle avoidance
(nothing is placed in the way) and not of the real robot, its lidar or the arena."""
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('INSPECT_DOMAIN', '85')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402

gx, gy = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (9.5, 5.5)
limit = float(sys.argv[3]) if len(sys.argv) > 3 else 180.0


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    tmp = tempfile.mkdtemp(prefix='inspection_params_')
    params = os.path.join(tmp, 'param')
    shutil.copytree(os.path.join(get_package_share_directory('framework_bringup'), 'param'), params)
    f = os.path.join(params, 'extensions.yaml')
    txt = open(f).read()
    assert 'safety_events_enabled: true' in txt
    open(f, 'w').write(re.sub(r'safety_events_enabled: true', 'safety_events_enabled: false', txt, count=1))
    quiet = dict(start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs = [subprocess.Popen(c, **quiet) for c in (
        ['ros2', 'launch', 'framework_bringup', 'nav_sim.launch.py', 'initial_pose:=0.5,0.5,0.0', f'params_dir:={params}'],
        ['ros2', 'launch', 'framework_bringup', 'extensions.launch.py', 'enable_arm:=false', 'enable_pose:=false', 'enable_odom:=false',
         'enable_pid:=false', 'enable_social:=false', 'enable_nav:=false', 'cmd_vel_topic:=cmd_vel', f'params_dir:={params}'])]
    rclpy.init()
    n = rclpy.create_node('inspection_probe')
    ok = False
    try:
        amcl = []
        n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: amcl.append(m), QoSProfile(depth=1))
        cli = ActionClient(n, NavigateToPose, 'navigate_to_pose')
        t_end = time.monotonic() + 90
        while time.monotonic() < t_end and not (cli.server_is_ready() and amcl):
            rclpy.spin_once(n, timeout_sec=0.2)
        time.sleep(3)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.w = 1.0
        t0 = time.monotonic()
        fut = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('FAIL goal rejected')
            return 1
        rf = gh.get_result_async()
        while time.monotonic() - t0 < limit and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.2)
        wall = time.monotonic() - t0
        status = rf.result().status if rf.done() else None
        err = math.dist((amcl[-1].pose.pose.position.x, amcl[-1].pose.pose.position.y), (gx, gy)) if amcl else float('nan')
        ok = status == GoalStatus.STATUS_SUCCEEDED and err < 0.35 and wall <= limit
        print(f'inspection run: result={"SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else status} time={wall:.1f}s of {limit:.0f}s final_error={err:.2f}m')
        print(('PASS ' if ok else 'FAIL ') + f'reached ({gx}, {gy}) within {limit:.0f} s')
    finally:
        for p in procs:
            kill(p, signal.SIGINT)
        time.sleep(4)
        for p in procs:
            kill(p)
        shutil.rmtree(tmp, ignore_errors=True)
        n.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
