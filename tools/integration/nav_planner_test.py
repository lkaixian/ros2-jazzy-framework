#!/usr/bin/env python3
"""Drive the sample map through the door with a chosen Nav2 global planner (nav_sim, no robot).

usage: nav_planner_test.py navfn|dstar_lite [goal_x goal_y]     (default goal: right_room 9.5 5.5)
Starts nav_sim (+ real Nav2/AMCL) and the cmd_vel mux, sends one NavigateToPose goal and prints
planner, result, wall time, planned path length and AMCL's final error. Domain NAVTEST_DOMAIN
(default 84). Exit code 0 only if the goal was reached within 0.35 m.
"""
import math
import os
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('NAVTEST_DOMAIN', '84')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402

planner = sys.argv[1] if len(sys.argv) > 1 else 'navfn'
gx, gy = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (9.5, 5.5)


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    procs = [subprocess.Popen(c, start_new_session=True, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL) for c in (
        ['ros2', 'launch', 'framework_bringup', 'nav_sim.launch.py', f'planner:={planner}',
         'initial_pose:=0.5,0.5,0.0'],
        ['ros2', 'launch', 'framework_bringup', 'extensions.launch.py', 'enable_arm:=false',
         'enable_pose:=false', 'enable_odom:=false', 'enable_pid:=false', 'enable_social:=false',
         'enable_nav:=false', 'cmd_vel_topic:=cmd_vel'])]
    rclpy.init()
    n = rclpy.create_node('nav_planner_test')
    ok = False
    try:
        plans, amcl = [], []
        n.create_subscription(Path, 'plan', lambda m: plans.append(m), 10)
        n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: amcl.append(m),
                              QoSProfile(depth=1))
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
        t_end = time.monotonic() + 180
        while time.monotonic() < t_end and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.2)
        wall = time.monotonic() - t0
        status = rf.result().status if rf.done() else None
        length = 0.0
        if plans:
            pts = [(p.pose.position.x, p.pose.position.y) for p in plans[0].poses]
            length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        err = float('nan')
        if amcl:
            p = amcl[-1].pose.pose.position
            err = math.dist((p.x, p.y), (gx, gy))
        ok = status == GoalStatus.STATUS_SUCCEEDED and err < 0.35
        print(f'planner={planner} result={"SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else status} '
              f'time={wall:.1f}s first_plan_length={length:.2f}m plans_received={len(plans)} '
              f'final_error={err:.2f}m')
        print(('PASS ' if ok else 'FAIL ') + f'reached ({gx}, {gy})')
    finally:
        for p in procs:
            kill(p, signal.SIGINT)
        time.sleep(4)
        for p in procs:
            kill(p)
        n.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
