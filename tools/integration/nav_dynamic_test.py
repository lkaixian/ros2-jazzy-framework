#!/usr/bin/env python3
"""Nav2 with people and overhead obstacles in the simulator (nav_sim scenario + real Nav2/AMCL + the safety mux).
usage: [NAVDYN_LOG=file] nav_dynamic_test.py CASE      CASE = crossing | door_crowd | overhead | overhead_off
  crossing      a person walks across the straight path while the robot drives it: arrive, never touch anyone.
  door_crowd    three people stand in the doorway for 25 s: wait / retry / replan, never push through, then arrive.
  overhead      a table top the lidar cannot see lies on the straight path, depth_obstacles:=true: arrive without touching it.
  overhead_off  the same without depth_obstacles: must TOUCH the table (the control that proves the scenario and the feature matter).
Domain NAVDYN_DOMAIN (default 86). Exit 0 only if the case's expectations hold. Simulation only: nothing here says how the real robot,
its lidar noise, its camera tilt or real people behave."""
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('NAVDYN_DOMAIN', '86')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from std_msgs.msg import Empty, Int32  # noqa: E402

# case: (scenario, start x, y, goal x, y, depth_obstacles, limit s, must arrive, must touch)
CASES = {
    'crossing': ('crossing', 0.5, 0.5, 4.0, 0.5, False, 90.0, True, False),
    'door_crowd': ('door_crowd', 3.0, 2.9, 7.0, 2.9, False, 150.0, True, False),
    'overhead': ('overhead', 0.5, 0.5, 4.0, 0.5, True, 90.0, True, False),
    'overhead_off': ('overhead', 0.5, 0.5, 4.0, 0.5, False, 60.0, False, True),
}


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    case = sys.argv[1] if len(sys.argv) > 1 else ''
    if case not in CASES:
        print(__doc__)
        return 2
    scen, sx, sy, gx, gy, depth, limit, must_arrive, must_touch = CASES[case]
    share = get_package_share_directory('framework_bringup')
    tmp = tempfile.mkdtemp(prefix='navdyn_params_')
    params = os.path.join(tmp, 'param')
    shutil.copytree(os.path.join(share, 'param'), params)
    f = os.path.join(params, 'extensions.yaml')
    txt = open(f).read()
    txt = re.sub(r'safety_events_enabled: true', 'safety_events_enabled: false', txt, count=1)          # no Kobuki bumper in the sim
    open(f, 'w').write(re.sub(r'estop_topic: estop', "estop_topic: ''", txt, count=1))                  # and no ESP32 e-stop publishing
    logf = open(os.environ['NAVDYN_LOG'], 'w') if os.environ.get('NAVDYN_LOG') else subprocess.DEVNULL      # NAVDYN_LOG=file keeps the stack's output
    quiet = dict(start_new_session=True, stdout=logf, stderr=subprocess.STDOUT)
    procs = [subprocess.Popen(c, **quiet) for c in (
        ['ros2', 'launch', 'framework_bringup', 'nav_sim.launch.py', f'initial_pose:={sx},{sy},0.0', f'start_x:={sx}', f'start_y:={sy}',
         f'scenario:={os.path.join(share, "scenarios", scen + ".yaml")}', 'manual_start:=true', f'depth_obstacles:={str(depth).lower()}', f'params_dir:={params}'],
        ['ros2', 'launch', 'framework_bringup', 'extensions.launch.py', 'enable_arm:=false', 'enable_pose:=false', 'enable_odom:=false',
         'enable_pid:=false', 'enable_social:=false', 'enable_nav:=false', 'cmd_vel_topic:=cmd_vel', f'params_dir:={params}'])]
    rclpy.init()
    n = rclpy.create_node('navdyn_probe')
    ok = False
    try:
        amcl, contacts, st = [], [0], dict(robot=None, gap=9.9)

        def on_actors(m):
            if st['robot']:
                for q in m.poses:
                    st['gap'] = min(st['gap'], math.dist(st['robot'], (q.position.x, q.position.y)))
        n.create_subscription(PoseStamped, 'nav_sim/ground_truth', lambda m: st.update(robot=(m.pose.position.x, m.pose.position.y)), 10)
        n.create_subscription(PoseArray, 'nav_sim/actors', on_actors, 10)
        n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: amcl.append(m), QoSProfile(depth=1))
        n.create_subscription(Int32, 'nav_sim/contacts', lambda m: contacts.__setitem__(0, m.data), 10)
        cli = ActionClient(n, NavigateToPose, 'navigate_to_pose')
        t_end = time.monotonic() + 90
        while time.monotonic() < t_end and not (cli.server_is_ready() and amcl):
            rclpy.spin_once(n, timeout_sec=0.2)
        time.sleep(3)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.w = 1.0
        go = n.create_publisher(Empty, 'nav_sim/scenario_start', 10)
        time.sleep(1)
        go.publish(Empty())                                         # the people start now, not whenever Nav2 happened to be up
        t0 = time.monotonic()
        fb = dict(rec=0, left=None)
        fut = cli.send_goal_async(goal, feedback_callback=lambda m: fb.update(rec=m.feedback.number_of_recoveries, left=m.feedback.distance_remaining))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=20)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('FAIL goal rejected')
            return 1
        rf = gh.get_result_async()
        while time.monotonic() - t0 < limit and not rf.done() and not (must_touch and contacts[0] > 0):
            rclpy.spin_once(n, timeout_sec=0.1)
        wall = time.monotonic() - t0
        status = rf.result().status if rf.done() else None
        err = math.dist((amcl[-1].pose.pose.position.x, amcl[-1].pose.pose.position.y), (gx, gy)) if amcl else float('nan')
        arrived = status == GoalStatus.STATUS_SUCCEEDED and err < 0.35
        if not rf.done():
            gh.cancel_goal_async()
        touched = contacts[0]
        print(f'{case}: status={status} arrived={arrived} time={wall:.1f}s final_error={err:.2f}m contacts={touched} recoveries={fb['rec']} closest_person={st["gap"]:.2f}m')
        met = case not in ('crossing',) or st['gap'] < 1.2         # the walker really did cross the robot's way, or the test proved nothing
        if not met:
            print('the person never came within 1.2 m of the robot: the scenario did not test avoidance')
        ok = met and (arrived if must_arrive else True) and ((touched > 0) if must_touch else touched == 0)
        print(('PASS ' if ok else 'FAIL ') + case + (' (arrived, no contact)' if must_arrive and not must_touch else ' (touched the table: the control)' if must_touch else ''))
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
