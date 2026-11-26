#!/usr/bin/env python3
"""Help Me Carry in the simulator, through the whole BT path: task spec -> orchestrator -> bt_validator -> bt_executor (dry_run:=false) ->
pid_extension FollowTarget (+ nav_extension / Nav2 for the way back) -> cmd_vel_mux -> nav_sim with scenarios/follow.yaml (an operator
walking east through the door and turning north, a second person cutting in once).
The spec is what the LLM would answer for "follow me, then go back to the start": follow (lock, until cancelled) for 45 s, in place of
WaitForPhrase because the simulator has no voice, then `return_home`. PASS: task done; nobody touched; the robot was never more than 3 m
behind the operator while following; it ends within 0.5 m of the `start` place. Simulation only. Domain HMC_DOMAIN (default 87)."""
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('HMC_DOMAIN', '87')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Empty, Int32, String  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FOLLOW_SEC = 45
SPEC = {'goal': 'follow me and then go back to the start', 'steps': [
    {'group': 'Parallel', 'success_count': 1, 'steps': [
        {'node': 'FollowTarget', 'ports': {'approach': 'true', 'distance': '1.0', 'until_cancelled': 'true', 'lock': 'true', 'timeout_ms': '900000'}},
        {'node': 'Sleep', 'ports': {'msec': str(FOLLOW_SEC * 1000)}}]},
    {'macro': 'return_home'}]}
START = (0.5, 0.5)                    # places.yaml `start`


def kill(p, sig=signal.SIGKILL):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def main():
    share = get_package_share_directory('framework_bringup')
    tmp = tempfile.mkdtemp(prefix='hmc_params_')
    params = os.path.join(tmp, 'param')
    shutil.copytree(os.path.join(share, 'param'), params)
    f = os.path.join(params, 'extensions.yaml')
    txt = re.sub(r'safety_events_enabled: true', 'safety_events_enabled: false', open(f).read(), count=1)    # no Kobuki bumper in the sim
    open(f, 'w').write(re.sub(r'estop_topic: estop', "estop_topic: ''", txt, count=1))                      # no ESP32 e-stop
    ros = ['--ros-args', '-p']
    logf = open(os.environ['HMC_LOG'], 'w') if os.environ.get('HMC_LOG') else subprocess.DEVNULL      # HMC_LOG=file keeps the stack's output
    quiet = dict(start_new_session=True, stdout=logf, stderr=subprocess.STDOUT)
    cmds = [
        ['ros2', 'launch', 'framework_bringup', 'nav_sim.launch.py', 'start_x:=1.0', 'start_y:=2.9', 'initial_pose:=1.0,2.9,0.0',
         f'scenario:={os.path.join(share, "scenarios", "follow.yaml")}', 'manual_start:=true', f'params_dir:={params}'],
        ['ros2', 'launch', 'framework_bringup', 'extensions.launch.py', 'enable_arm:=false', 'enable_pose:=false', 'enable_odom:=false',
         'enable_social:=false', 'enable_pid:=true', 'enable_nav:=true', 'cmd_vel_topic:=cmd_vel', f'params_dir:={params}'],
        ['ros2', 'run', 'bt_validator', 'bt_validator_node'],
        ['ros2', 'run', 'bt_slm', 'bt_slm_node'] + ros + ['model_path:=none'],
        ['ros2', 'run', 'bt_orchestrator', 'orchestrator_node'],
        ['ros2', 'run', 'bt_executor', 'bt_executor_node'] + ros + ['dry_run:=false', '-p', 'auto_run:=true', '-p', 'max_runtime_sec:=240.0']]
    procs = [subprocess.Popen(c, **quiet) for c in cmds]
    rclpy.init()
    n = rclpy.create_node('hmc_probe')
    st = dict(robot=None, actors=None, contacts=0, state={}, max_gap=0.0, following=False)
    ok = False
    t_start = [None]
    try:
        pub = {t: n.create_publisher(String, t, 10) for t in ('bt/task', 'bt/task_spec')}
        go = n.create_publisher(Empty, 'nav_sim/scenario_start', 10)

        def on_robot(m):
            st['robot'] = (m.pose.position.x, m.pose.position.y)
            if st['following'] and st['actors']:
                st['max_gap'] = max(st['max_gap'], math.dist(st['robot'], st['actors'][0]))
        n.create_subscription(PoseStamped, 'nav_sim/ground_truth', on_robot, 10)
        n.create_subscription(PoseArray, 'nav_sim/actors', lambda m: st.update(actors=[(p.position.x, p.position.y) for p in m.poses]), 10)
        n.create_subscription(Int32, 'nav_sim/contacts', lambda m: st.update(contacts=m.data), 10)
        n.create_subscription(String, 'task/state', lambda m: st.update(state=json.loads(m.data)), LATCHED)

        trace, last = [], [0.0]

        def spin(sec, pred=lambda: False):
            end = time.monotonic() + sec
            while time.monotonic() < end and not pred():
                rclpy.spin_once(n, timeout_sec=0.05)
                if t_start[0] is not None and time.monotonic() - last[0] > 10 and st['robot'] and st['state'].get('state') != 'done':
                    last[0] = time.monotonic()
                    trace.append(f'{time.monotonic() - t_start[0]:.0f}s robot=({st["robot"][0]:.1f},{st["robot"][1]:.1f})' +
                                 (f' operator=({st["actors"][0][0]:.1f},{st["actors"][0][1]:.1f})' if st['actors'] else ''))
            return pred()

        spin(90, lambda: pub['bt/task_spec'].get_subscription_count() and st['robot'] and st['actors'])
        spin(25)                                                     # Nav2 up, extension manifests registered (3 s republish)
        t0 = t_start[0] = time.monotonic()
        pub['bt/task'].publish(String(data=SPEC['goal']))
        spin(0.3)
        pub['bt/task_spec'].publish(String(data=json.dumps(SPEC)))
        spin(4)                                                      # the executor starts FollowTarget; the operator starts walking now
        go.publish(Empty())
        st['following'] = True
        spin(FOLLOW_SEC - 7)                                         # stop measuring the gap when the follow step ends
        st['following'] = False
        done = spin(200, lambda: st['state'].get('state') in ('done', 'failed') and st['state'].get('task') == SPEC['goal'])
        wall = time.monotonic() - t0
        home = math.dist(st['robot'], START) if st['robot'] else float('nan')
        res = {'task done': done and st['state'].get('state') == 'done', 'nobody touched': st['contacts'] == 0,
               'stayed within 3 m of the operator': st['max_gap'] < 3.0, 'ended within 0.5 m of start': home < 0.5}
        print(f'state={st["state"].get("state")} message={st["state"].get("message", "")!r} time={wall:.0f}s contacts={st["contacts"]} '
              f'max_gap={st["max_gap"]:.2f}m home_error={home:.2f}m')
        print('trace: ' + ' | '.join(trace))
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        ok = all(res.values())
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
