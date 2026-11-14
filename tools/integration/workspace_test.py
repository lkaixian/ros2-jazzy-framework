#!/usr/bin/env python3
"""Workspaces end to end through the real `ws` CLI + SROS2 enforcement, on one machine.

create -> serve -> discover -> join (wrong token refused, right token admitted) -> two members talk
over secured DDS; an unsecured outsider and an outsider holding another workspace's keys see nothing.
Roles: an `observer` invite joins read-only (it receives, its publications are dropped by DDS Security).
Revoke: `ws revoke` removes a member, rotates tokens and re-keys; the revoked member's old keys no longer
reach the workspace, the old token is refused, and a remaining member can re-join and talk again.
Uses a temp ROBOCUP_WS_HOME and port 47650. Exit code 0 only if every check passed.
"""
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import time

import yaml

HOME = tempfile.mkdtemp(prefix='wstest_')
HOME2 = tempfile.mkdtemp(prefix='wstest2_')       # a second machine's ~/.robocup (joined/<ws> is per workspace name)
BASE = dict(os.environ, ROBOCUP_WS_HOME=HOME)
BASE.pop('ROS_DOMAIN_ID', None)
PEER = textwrap.dedent('''
    import sys, time, rclpy
    from std_msgs.msg import String
    role = sys.argv[1]
    rclpy.init(); n = rclpy.create_node('peer_' + role); got = []
    if role == 'pub':
        try:
            p = n.create_publisher(String, 'chat', 10)
        except Exception as e:           # DDS Security can refuse the writer right here
            print('DENIED', flush=True); sys.exit(0)
        end = time.monotonic() + 14
        print('STARTED', flush=True)
        while time.monotonic() < end:
            p.publish(String(data='hi')); rclpy.spin_once(n, timeout_sec=0.1)
    else:
        n.create_subscription(String, 'chat', lambda m: got.append(1), 10); end = time.monotonic() + 10
        while time.monotonic() < end and not got:
            rclpy.spin_once(n, timeout_sec=0.1)
        print('RECEIVED' if got else 'NOTHING')
''')


def ws(*args, env=BASE, check=True):
    r = subprocess.run(['ros2', 'run', 'framework_bringup', 'ws', *args], env=env,
                       capture_output=True, text=True, timeout=90)
    if check and r.returncode:
        raise RuntimeError(f'ws {args}: {r.stderr}')
    return r


def env_of(path):
    out = subprocess.run(['bash', '-c', f'source {path} && env'], env=BASE, capture_output=True,
                         text=True, check=True).stdout
    e = dict(l.split('=', 1) for l in out.splitlines() if '=' in l)
    e['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    e['PYTHONUNBUFFERED'] = '1'
    return e


def peer(role, env):
    return subprocess.Popen([sys.executable, '-c', PEER, role], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)


def result(name, p):
    out, err = p.communicate(timeout=40)
    out = out.strip()
    if out not in ('RECEIVED', 'NOTHING'):      # the probe itself broke: say why, do not guess
        print(f'--- probe {name} printed {out!r}; stderr tail:\n{err[-600:]}', file=sys.stderr)
    return out


def main():
    res, serve = {}, None
    try:
        ws('create', 'itest', '--host-name', 'hosta', '--address', '127.0.0.1', '--port', '47650')
        serve = subprocess.Popen(
            ['ros2', 'run', 'framework_bringup', 'ws', 'serve', 'itest', '--bind', '127.0.0.1',
             '--announce-to', '127.0.0.1', '--address', '127.0.0.1'], env=BASE,
            start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4)
        d = ws('discover', '--timeout', '4').stdout
        res['discover: the workspace is announced'] = 'itest' in d and '47650' in d
        token = yaml.safe_load(open(f'{HOME}/workspaces/itest/secret.yaml'))['token']
        bad = ws('join', 'itest', '--token', 'wrong-token', '--host-name', 'hostb', check=False)
        res['join with a wrong token is refused'] = bad.returncode != 0 and not os.path.isdir(
            f'{HOME}/joined/itest')
        for _ in range(3):                  # join finds the workspace by its UDP announcement: a loaded runner can miss one
            ok = ws('join', 'itest', '--token', token, '--host-name', 'hostb', '--timeout', '8', check=False)
            if ok.returncode == 0:
                break
        if ok.returncode:
            print(f'--- join failed; stderr:\n{ok.stderr[-600:]}', file=sys.stderr)
        res['join with the token succeeds'] = ok.returncode == 0 and os.path.isdir(
            f'{HOME}/joined/itest/keystore/enclaves/itest/hostb')
        members = ws('members', 'itest').stdout
        res['members lists both machines'] = 'hosta' in members and 'hostb' in members
        ea, eb = env_of(f'{HOME}/workspaces/itest/env.sh'), env_of(f'{HOME}/joined/itest/env.sh')
        res['both share domain and LLM key'] = (ea['ROS_DOMAIN_ID'] == eb['ROS_DOMAIN_ID']
                                                and ea['LLM_API_KEY'] == eb['LLM_API_KEY']
                                                and len(ea['LLM_API_KEY']) > 20)
        dom = ea['ROS_DOMAIN_ID']
        # an outsider workspace with its own keystore in the SAME domain
        ws('create', 'evil', '--domain', dom, '--host-name', 'mallory', '--address', '127.0.0.1',
           '--port', '47651')
        ee = env_of(f'{HOME}/workspaces/evil/env.sh')
        out = dict(os.environ, ROS_DOMAIN_ID=dom, RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                   CYCLONEDDS_URI=ea['CYCLONEDDS_URI'], ROBOCUP_PEER='localhost')
        out.pop('ROS_SECURITY_ENABLE', None)
        pub = peer('pub', ea)
        time.sleep(1)
        member = peer('sub', eb)
        outsider = peer('sub', out)
        evil = peer('sub', {**ee, 'ROBOCUP_PEER': 'localhost'})
        got = {k: result(k, p) for k, p in
               (('member', member), ('unsecured', outsider), ('evil', evil))}
        pub_err = pub.communicate(timeout=20)[1] if pub.poll() is not None else ''
        if pub_err:
            print(f'--- publisher stderr tail:\n{pub_err[-600:]}', file=sys.stderr)
        pub.kill()
        res['a joined member receives over secured DDS'] = got['member'] == 'RECEIVED'
        res['an unsecured outsider receives nothing'] = got['unsecured'] == 'NOTHING'
        res['an outsider with another workspace\'s keys receives nothing'] = got['evil'] == 'NOTHING'

        # ---- roles: an observer reads but cannot publish
        base2 = dict(BASE, ROBOCUP_WS_HOME=HOME2)
        inv = ws('invite', 'itest', '--role', 'observer', '--address', '127.0.0.1').stdout.strip().split('\n')[-1]
        res['an observer invite carries the role'] = inv.endswith('~observer')
        j2 = ws('join', '--invite', inv, '--host-name', 'hostc', env=base2, check=False)
        res['an observer joins with its own token'] = j2.returncode == 0 and 'role observer' in j2.stdout
        res['members lists the observer role'] = any(
            'hostc' in l and 'observer' in l for l in ws('members', 'itest').stdout.splitlines())
        ec = env_of(f'{HOME2}/joined/itest/env.sh')
        pub = peer('pub', ea)
        time.sleep(1)
        obs_sub = peer('sub', ec)
        res['an observer receives what a full member publishes'] = result('observer sub', obs_sub) == 'RECEIVED'
        pub.kill()
        pub = peer('pub', ec)                       # the observer tries to publish
        state = pub.stdout.readline().strip()        # DENIED: the writer was refused locally; STARTED: it must not arrive
        full_sub = peer('sub', ea)
        seen = result('full sub', full_sub)
        print(f'--- observer publish probe: publisher {state!r}, full member saw {seen!r}', file=sys.stderr)
        res['an observer cannot publish (dropped by DDS Security)'] = state in ('DENIED', 'STARTED') and seen == 'NOTHING'
        pub.kill()

        # ---- revoke: hostb out, tokens rotated, workspace re-keyed
        old_full = yaml.safe_load(open(f'{HOME}/workspaces/itest/secret.yaml'))['token']
        rv = ws('revoke', 'itest', 'hostb', check=False)
        res['revoke succeeds and says what to do'] = rv.returncode == 0 and 'must run `ws join` again' in rv.stdout
        members = ws('members', 'itest').stdout
        res['the revoked member is gone, the other must re-join'] = 'hostb' not in members and 'must re-join' in members
        stale = ws('join', 'itest', '--token', old_full, '--host-name', 'hostd', check=False)
        res['the old token is refused after revoke'] = stale.returncode != 0
        pub = peer('pub', ea)                       # owner, new keystore
        time.sleep(1)
        old_b = peer('sub', eb)                     # hostb still holds the OLD keys
        res['the revoked member\'s old keys reach nothing'] = result('revoked sub', old_b) == 'NOTHING'
        pub.kill()
        new_inv = ws('invite', 'itest', '--role', 'observer', '--address', '127.0.0.1').stdout.strip().split('\n')[-1]
        j3 = ws('join', '--invite', new_inv, '--host-name', 'hostc', env=base2, check=False)
        pub = peer('pub', ea)
        time.sleep(1)
        again = peer('sub', env_of(f'{HOME2}/joined/itest/env.sh'))
        res['a remaining member re-joins and talks again'] = j3.returncode == 0 and result('rejoined sub', again) == 'RECEIVED'
        pub.kill()
    finally:
        for k, v in res.items():
            print(('PASS ' if v else 'FAIL ') + k)
        if serve:
            for sig in (signal.SIGINT, signal.SIGKILL):
                try:
                    os.killpg(serve.pid, sig)
                except ProcessLookupError:
                    break
                time.sleep(1)
        shutil.rmtree(HOME, ignore_errors=True)
        shutil.rmtree(HOME2, ignore_errors=True)
    return 0 if res and all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
