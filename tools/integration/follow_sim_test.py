#!/usr/bin/env python3
"""FollowTarget end to end in the simulator: nav_sim (map test_room, lidar, RealSense-model persons) + pid_extension, no Nav2, no mux.
scenarios/follow.yaml: the operator (red) walks east through the door and turns north at 0.2 m/s; a second person (blue) cuts between
the operator and the robot and hides them for a moment. FollowTarget runs lock + until_cancelled at 1.0 m. PASS when, for the whole walk,
the robot never touches anyone, is never farther than 3 m from the operator once it has caught up, goes through the door, and the
action is still running (no 'target lost') when the operator stops. A second run with the search turned off (search:=false) is the
control: it must lose the operator at the occlusion, so a pass above is the search doing its job. Domain FOLLOW_SIM_DOMAIN (default 96)."""
import math
import os
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('FOLLOW_SIM_DOMAIN', '96')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bt_pipeline_msgs.action import ExecuteNode  # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from std_msgs.msg import Empty, Int32  # noqa: E402

SCENARIO = os.path.join(get_package_share_directory('framework_bringup'), 'scenarios', 'follow.yaml')
WALK_SEC = 48.0                     # the operator needs ~40 s for the whole path


def kill(p):
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def run(search):
    quiet = dict(start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs = [
        subprocess.Popen(['ros2', 'run', 'nav_sim', 'nav_sim_node', '--ros-args', '-p', f'scenario:={SCENARIO}', '-p', 'start_x:=1.0',
                          '-p', 'start_y:=2.9', '-p', 'manual_start:=true', '-p', 'cmd_vel_topic:=pid_extension/cmd_vel', '-p', 'lidar_x:=0.15', '-p', 'lidar_yaw:=3.1416'], **quiet),
        subprocess.Popen(['ros2', 'run', 'pid_extension', 'pid_extension_node', '--ros-args', '-p', 'scan_yaw_offset:=3.1416',
                          '-p', 'max_linear:=0.25'], **quiet)]
    rclpy.init()
    n = rclpy.create_node('follow_sim_probe')
    st = dict(robot=None, actors=None, contacts=0)
    n.create_subscription(PoseStamped, 'nav_sim/ground_truth', lambda m: st.update(robot=(m.pose.position.x, m.pose.position.y)), 10)
    n.create_subscription(PoseArray, 'nav_sim/actors', lambda m: st.update(actors=[(p.position.x, p.position.y) for p in m.poses]), 10)
    n.create_subscription(Int32, 'nav_sim/contacts', lambda m: st.update(contacts=m.data), 10)
    out = dict(max_gap=0.0, final_gap=None, door=False, contacts=0, msg='running')
    try:
        ac = ActionClient(n, ExecuteNode, 'pid_extension/execute')
        t_end = time.monotonic() + 30
        while time.monotonic() < t_end and not (ac.server_is_ready() and st['robot'] and st['actors']):
            rclpy.spin_once(n, timeout_sec=0.1)
        ports = {'approach': 'true', 'distance': '1.0', 'until_cancelled': 'true', 'lock': 'true', 'timeout_ms': '120000',
                 'search': 'true' if search else 'false', 'trail': os.environ.get('FOLLOW_TRAIL', 'false')}   # FOLLOW_TRAIL=true: along the walked path
        fut = ac.send_goal_async(ExecuteNode.Goal(node_id='FollowTarget', port_names=list(ports), port_values=list(ports.values())))
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        gh = fut.result()
        rf = gh.get_result_async()
        go = n.create_publisher(Empty, 'nav_sim/scenario_start', 10)
        time.sleep(1)
        go.publish(Empty())                                     # the operator starts walking now
        t0 = time.monotonic()
        while time.monotonic() - t0 < WALK_SEC and not rf.done():
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.monotonic() - t0 > 12 and st['robot'] and st['actors']:                     # after the initial catch-up
                out['max_gap'] = max(out['max_gap'], math.dist(st['robot'], st['actors'][0]))
            out['door'] = out['door'] or (st['robot'] or (0, 0))[0] > 5.5
        out['contacts'] = st['contacts']
        out['final_gap'] = math.dist(st['robot'], st['actors'][0]) if st['robot'] and st['actors'] else None
        out['msg'] = rf.result().result.message if rf.done() else 'still following'
        if not rf.done():
            gh.cancel_goal_async()
            t1 = time.monotonic()
            while not rf.done() and time.monotonic() - t1 < 3:
                rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
        for p in procs:
            kill(p)
    return out


def main():
    res = {}
    with_search = run(True)
    print('search on :', with_search)
    res['search: still following at the end (no target lost)'] = with_search['msg'] == 'still following'
    res['search: never touched anyone'] = with_search['contacts'] == 0
    res['search: never more than 3 m from the operator'] = with_search['max_gap'] < 3.0
    res['search: went through the door'] = with_search['door']
    res['search: ends close behind the operator'] = with_search['final_gap'] is not None and 0.5 <= with_search['final_gap'] <= 2.5
    time.sleep(2)
    control = run(False)
    print('search off:', control)
    res['control (search off) loses the operator at the occlusion'] = 'lost' in control['msg']
    for k, v in res.items():
        print(('PASS ' if v else 'FAIL ') + k)
    return 0 if all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
