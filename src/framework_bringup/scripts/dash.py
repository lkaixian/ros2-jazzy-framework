#!/usr/bin/python3
"""dash: terminal view of what the framework is doing.

  ros2 run framework_bringup dash            # live status table (1 Hz): nodes, LLM endpoints, cluster
  ros2 run framework_bringup dash --logs     # verbose: every /rosout line, coloured by level
  ros2 run framework_bringup dash --logs --level warn --node llm_bridge_node
  ros2 run framework_bringup dash --once     # one frame, no clearing (pipes, CI)

Node states: ONLINE (in the ROS graph), BUSY (bt_executor running a tree), ERROR (logged an
error in the last 30 s, or llm_bridge reports `down`), OFFLINE (expected but not in the graph).
The devices section lists the hardware in param/devices.yaml: device file present, driver topic rate.
Expected nodes = the table in check_params. Each LLM endpoint line also shows the link to it: local / wired / wireless (the interface this
machine routes through) and the TCP connect time in ms, measured every 3 s in the background.

Keys (status view, on a terminal): j/k or arrows select a node; r restart it (SIGINT), R force (SIGKILL);
the launch file respawns it. s stop the whole system (SIGINT to `ros2 launch`), S force; w restart the whole
system (stop, then relaunch the same command line, output in /tmp/robocup_launch.log), W force; h halt the
running task (bt_executor/halt); t shows/hides the plan tree (live node status from bt_executor).
s/S/w/W ask y/n first. Without keys it only reads.
"""
import argparse
import collections
import glob
import importlib.machinery
import json
import ipaddress
import os
import re
import select
import signal
import socket
import subprocess
import sys
import threading
import termios
import time
import tty
from urllib.parse import urlparse

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from bt_common.battery_guard import battery_level
from cluster_msgs.msg import ClusterStatus
from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

try:
    from kobuki_ros_interfaces.msg import SensorState
except ImportError:     # no Kobuki driver messages installed: no battery line
    SensorState = None

ERR_WINDOW = 30.0
COL = {'ONLINE': '32', 'BUSY': '36', 'ERROR': '31', 'OFFLINE': '90', 'WARN': '33'}
LEVEL = {10: ('DEBUG', '90'), 20: ('INFO', '0'), 30: ('WARN', '33'), 40: ('ERROR', '31'),
         50: ('FATAL', '1;31')}
LEVEL_NUM = {'debug': 10, 'info': 20, 'warn': 30, 'error': 40, 'fatal': 50}
TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)


def classify(online, err_age, busy=False, down=False):
    """err_age: seconds since the node's last ERROR/FATAL log, or None."""
    if not online:
        return 'OFFLINE'
    if down or (err_age is not None and err_age < ERR_WINDOW):
        return 'ERROR'
    return 'BUSY' if busy else 'ONLINE'


def parse_route_dev(text):
    """`ip route get <addr>` output -> interface name ('lo' for this machine), or ''."""
    m = re.search(r'\bdev (\S+)', text or '')
    return m.group(1) if m else ''


def link_type(dev, sysroot='/sys/class/net'):
    """local | wireless | wired | virtual | ? for a Linux interface. Wireless interfaces have a
    `wireless` or `phy80211` entry in sysfs; a physical NIC has a `device` entry, a bridge/veth/tun
    (docker0, VPN) does not. Only describes THIS machine's side of the link."""
    if not dev:
        return '?'
    if dev == 'lo':
        return 'local'
    base = os.path.join(sysroot, dev)
    if not os.path.isdir(base):
        return '?'
    if any(os.path.exists(os.path.join(base, n)) for n in ('wireless', 'phy80211')):
        return 'wireless'
    return 'wired' if os.path.exists(os.path.join(base, 'device')) else 'virtual'


def tcp_ms(host, port, timeout=1.0):
    """TCP connect time in ms (one SYN/SYN-ACK round trip), or None if unreachable."""
    t0 = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return (time.monotonic() - t0) * 1000.0
    except OSError:
        return None


def check_link(url):
    """{'kind', 'dev', 'ms'} for an http(s) endpoint url."""
    u = urlparse(url)
    host, port = u.hostname or '', u.port or (443 if u.scheme == 'https' else 80)
    try:
        addr = socket.gethostbyname(host)
        if ipaddress.ip_address(addr).is_loopback:
            dev = 'lo'          # no need for `ip`, which minimal containers do not have
        else:                   # needs iproute2; without it the link type shows '?'
            dev = parse_route_dev(subprocess.run(
                ['ip', '-o', 'route', 'get', addr], capture_output=True, text=True,
                timeout=2).stdout)
    except (OSError, subprocess.SubprocessError, socket.gaierror, ValueError):
        dev = ''
    first = tcp_ms(host, port) if host else None
    ms = first
    if first is not None:      # median of 3: one sample over WiFi is often an outlier
        more = [m for m in (tcp_ms(host, port), tcp_ms(host, port)) if m is not None]
        ms = sorted([first] + more)[len(more) // 2] if more else first
    return {'kind': link_type(dev), 'dev': dev, 'ms': ms}


def battery_state(raw, charger=0):
    """Kobuki SensorState.battery (0.1 V units) -> (dash colour key, text); levels come from bt_common.battery_guard."""
    level = battery_level(raw, charger)
    v = raw / 10.0
    note = ' (charging)' if charger else ''
    return {'unknown': ('OFFLINE', 'base battery unknown'),
            'critical': ('ERROR', f'base battery {v:.1f} V CRITICAL{note}'),
            'low': ('WARN', f'base battery {v:.1f} V low{note}'),
            'ok': ('ONLINE', f'base battery {v:.1f} V{note}')}[level]


TREE_GLYPH = {'R': ('>', '36'), 'S': ('+', '32'), 'F': ('x', '31'), 'K': ('-', '90'), None: ('.', '90')}


def render_tree(tree, state, color=True, max_lines=22, width=100):
    """bt/tree (list of {uid, depth, name, ports}) + bt/node_status ({uid: R|S|F|K}) -> text lines.
    R running (cyan >), S done (green +), F failed (red x), . not reached yet. Long trees show a window around the
    running node."""
    lines = []
    for n in tree:
        g, code = TREE_GLYPH.get(state.get(str(n['uid'])), TREE_GLYPH[None])
        ports = ','.join(f'{k}={v}' for k, v in (n.get('ports') or {}).items())
        label = n['name'] + (f'({ports})' if ports else '')
        lines.append('  ' * n['depth'] + paint(g, code, color) + ' ' + label[:max(10, width - 2 * n['depth'])])
    if len(lines) <= max_lines:
        return lines
    run = next((i for i, n in enumerate(tree) if state.get(str(n['uid'])) == 'R'), None)
    if run is None:
        run = max((i for i, n in enumerate(tree) if state.get(str(n['uid']))), default=0)
    start = min(max(0, run - max_lines // 2), len(lines) - max_lines)
    out = lines[start:start + max_lines]
    if start:
        out[0] = f'  ... {start} more above'
    if start + max_lines < len(lines):
        out[-1] = f'  ... {len(lines) - start - max_lines + 1} more below'
    return out


def paint(text, code, color=True):
    return f'\033[{code}m{text}\033[0m' if color else text


def cmdlines():
    for d in os.listdir('/proc'):
        if d.isdigit() and int(d) != os.getpid():
            try:
                with open(f'/proc/{d}/cmdline', 'rb') as f:
                    yield int(d), f.read().decode(errors='replace').split('\0')[:-1]
            except OSError:
                pass


def find_pids(name):
    # shortcut: matches the executable name, so a node renamed at launch is not found
    return [p for p, a in cmdlines() if any(os.path.basename(x) == name for x in a[:3])]


def launch_pids():
    """`ros2 launch` processes that s/w target: framework.launch.py when it runs (the Kobuki driver, started
    separately, must not count), else every `ros2 launch`."""
    out = [(p, a) for p, a in cmdlines()
           if len(a) > 2 and os.path.basename(a[1]) == 'ros2' and a[2] == 'launch']
    return [p for p, a in out if 'framework.launch.py' in a] or [p for p, _ in out]


def kill(pid, force):
    if force:   # whole group so a force stop leaves no children, unless it is dash's own group
        pg = os.getpgid(pid)
        return os.killpg(pg, signal.SIGKILL) if pg != os.getpgid(0) else os.kill(pid, signal.SIGKILL)
    os.kill(pid, signal.SIGINT)


RATE_WINDOW = 5.0


def load_devices():
    """param/devices.yaml from the install, else from the source tree next to this script."""
    here = os.path.dirname(os.path.realpath(__file__))
    for d in (os.path.join(get_package_share_directory('framework_bringup'), 'param'),
              os.path.join(here, '..', 'param')):
        try:
            with open(os.path.join(d, 'devices.yaml')) as f:
                return (yaml.safe_load(f) or {}).get('devices', [])
        except (OSError, PackageNotFoundError):
            continue
    return []


def device_state(present, rate, topic, optional=False):
    """present: device file exists; rate: Hz of its topic (None = not subscribed yet)."""
    if not present:
        return 'OFFLINE' if optional else 'ERROR'
    if not topic or rate:
        return 'ONLINE'
    return 'WARN'      # plugged in, but the driver is not publishing


def usb_name(path):
    """Short hardware name from udev (cached by the caller); '' when udev knows nothing."""
    try:
        out = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', path], timeout=2,
                             capture_output=True, text=True).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    p = dict(ln.split('=', 1) for ln in out.splitlines() if '=' in ln)
    return p.get('ID_MODEL_FROM_DATABASE') or p.get('ID_MODEL') or p.get('ID_VENDOR') or ''


def expected_nodes():
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        mod = importlib.machinery.SourceFileLoader(
            'check_params', os.path.join(here, 'check_params')
            if os.path.exists(os.path.join(here, 'check_params')) else
            os.path.join(here, 'check_params.py')).load_module()
        return sorted(mod.NODES) + sorted(mod.PERCEPTION)
    except Exception:  # noqa: BLE001  shortcut: no expected list -> only show nodes that exist
        return []


class Dash:
    def __init__(self, node, color, keys=False):
        self.n, self.color, self.keys = node, color, keys
        self.err = {}          # node name -> (monotonic time, message)
        self.llm = {}
        self.bt = {}
        self.task = {}
        self.links = {}      # endpoint url -> check_link() result, refreshed by a thread
        threading.Thread(target=self._probe_links, daemon=True).start()
        self.cluster = None
        self.battery = None
        if SensorState is not None:
            node.create_subscription(SensorState, 'sensors/core', lambda m: setattr(
                self, 'battery', (m.battery, m.charger, time.monotonic())), 5)
        self.names, self.sel, self.msg, self.pending = [], 0, '', None
        self.devices, self.hw_name = load_devices(), {}
        self.since = {}
        self.stamps = {}     # topic -> deque of arrival times (raw subscriptions: no deserialising)
        self.tree, self.nstate, self.show_tree = [], {}, True
        node.create_subscription(String, 'bt/tree', self._on_tree, TL)
        node.create_subscription(String, 'bt/node_status', self._on_node_status, 10)
        self.halt = node.create_client(Trigger, 'bt_executor/halt')
        node.create_subscription(Log, '/rosout', self._log, 100)
        node.create_subscription(String, 'llm/status', lambda m: self._js('llm', m), TL)
        node.create_subscription(String, 'bt/status', lambda m: self._js('bt', m), 10)
        node.create_subscription(String, 'task/state', lambda m: self._js('task', m), TL)
        node.create_subscription(ClusterStatus, 'cluster/status',
                                 lambda m: setattr(self, 'cluster', m), TL)

    def _probe_links(self):
        while True:     # blocking probes live here so a dead host never stalls the 1 Hz frame
            for e in list(self.llm.get('endpoints', [])):
                self.links[e['url']] = check_link(e['url'])
            time.sleep(3.0)

    def _js(self, attr, m):
        try:
            setattr(self, attr, json.loads(m.data))
        except ValueError:
            pass

    def _log(self, m):
        if m.level >= 40:
            self.err[m.name] = (time.monotonic(), m.msg)

    def _on_tree(self, m):
        try:
            self.tree, self.nstate = json.loads(m.data), {}
        except ValueError:
            pass

    def _on_node_status(self, m):
        try:
            self.nstate.update(json.loads(m.data).get('s', {}))
        except ValueError:
            pass

    def say(self, text):
        self.msg = text

    def key(self, k):
        """Returns True when the frame should be redrawn now."""
        if self.pending:
            act, force = self.pending
            self.pending = None
            if k == 'y':
                threading.Thread(target=self._system, args=(act, force), daemon=True).start()
            else:
                self.say('cancelled')
            return True
        if k in ('j', '\x1b[B', 'k', '\x1b[A') and self.names:
            self.sel = (self.sel + (1 if k in ('j', '\x1b[B') else -1)) % len(self.names)
        elif k in 'rR':
            self._node(self.names[self.sel] if self.names else '', k == 'R')
        elif k in ('s', 'S', 'w', 'W'):
            self.pending = ('stop' if k in 'sS' else 'restart', k.isupper())
            self.say(f"{'FORCE ' if k.isupper() else ''}{self.pending[0]} the whole system? y/n")
        elif k == 't':
            self.show_tree = not self.show_tree
        elif k == 'h':
            ok = self.halt.service_is_ready()
            self.say('halt requested' if ok else 'bt_executor/halt not available')
            if ok:
                self.halt.call_async(Trigger.Request())
        else:
            return False
        return True

    def _node(self, name, force):
        pids = find_pids(name)
        if not pids:
            return self.say(f'{name}: no running process found')
        for p in pids:
            kill(p, force)    # the launch file's respawn starts it again
        self.say(f"{name}: {'killed' if force else 'asked to stop'}, respawns in ~3 s")

    def _system(self, act, force):
        lp = launch_pids()
        if len(lp) != 1:
            return self.say(f'{len(lp)} `ros2 launch` processes found, need exactly 1')
        pid = lp[0]
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(x.split('=', 1) for x in f.read().decode().split('\0') if '=' in x)
            argv, cwd = dict(cmdlines())[pid], os.readlink(f'/proc/{pid}/cwd')
            kill(pid, force)
        except (OSError, KeyError) as e:
            return self.say(f'cannot stop launch: {e}')
        self.say('stopping...')
        for _ in range(150):    # graceful stop of the whole tree can take a while
            if not os.path.exists(f'/proc/{pid}'):
                break
            time.sleep(0.1)
        else:
            return self.say(f'launch {pid} still running; press S to force')
        if act == 'stop':
            return self.say('system stopped')
        with open('/tmp/robocup_launch.log', 'ab') as log:
            subprocess.Popen(argv, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True)
        self.say('system relaunched, output in /tmp/robocup_launch.log')

    def rate(self, topic):
        """Hz over the last RATE_WINDOW s; subscribes the first time the topic's type is known."""
        if topic not in self.stamps:
            types = dict(self.n.get_topic_names_and_types()).get(topic)
            if not types:
                return None
            dq = self.stamps[topic] = collections.deque(maxlen=1000)
            self.since[topic] = time.monotonic()
            self.n.create_subscription(get_message(types[0]), topic,
                                       lambda _m, dq=dq: dq.append(time.monotonic()),
                                       qos_profile_sensor_data, raw=True)
        now = time.monotonic()
        span = max(1.0, min(RATE_WINDOW, now - self.since[topic]))
        return sum(1 for t in self.stamps[topic] if now - t < span) / span

    def device_rows(self):
        out = []
        for d in self.devices:
            hits = glob.glob(d['path'])
            path = hits[0] if hits else None
            topic = d.get('topic', '')
            rate = self.rate(topic) if path and topic else None
            s = device_state(bool(path), rate, topic, d.get('optional', False))
            if path and path not in self.hw_name:
                self.hw_name[path] = usb_name(path)
            what = {'ONLINE': ('present' if not topic else f'{rate or 0:.0f} Hz {topic}'),
                    'WARN': f'no data on {topic}' if rate is not None else f'{topic} not advertised',
                    'ERROR': 'missing', 'OFFLINE': 'absent'}[s]
            out.append({'name': d['name'], 'path': d['path'], 'hw': self.hw_name.get(path, ''),
                        'state': s, 'what': what})
        return out

    def device_lines(self, c):
        return [f"   {r['name']:<8}{r['path'][:26]:<27}{r['hw'][:21]:<22}" +
                paint(f"{r['state']:<8}", COL[r['state']], c) + r['what'] for r in self.device_rows()]

    def node_rows(self, names, online, now):
        busy_node = 'bt_executor_node' if self.bt.get('state') == 'running' else None
        out = []
        for name in names:
            err = self.err.get(name)
            s = classify(name in online, now - err[0] if err else None,
                         busy=name == busy_node,
                         down=name == 'llm_bridge_node' and self.llm.get('state') == 'down')
            note = (err[1][:70] if err and s == 'ERROR' else
                    self.bt.get('message', '') if s == 'BUSY' else '')
            out.append((name, s, note))
        return out

    def frame(self, expected):
        c, now = self.color, time.monotonic()
        online = {n for n, _ in self.n.get_node_names_and_namespaces()}
        online = {n for n in online if not n.startswith('_ros2cli')}   # the CLI's own daemon
        names = sorted(set(expected) | online - {self.n.get_name()})
        sec = os.environ.get('ROS_SECURITY_ENABLE', '').lower() == 'true'
        where = (f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}  " +
                 (paint(f"SROS2 {os.environ.get('ROS_SECURITY_STRATEGY', '?')} "
                        f"enclave {os.environ.get('ROS_SECURITY_ENCLAVE_OVERRIDE', '/')}",
                        COL['ONLINE'], c) if sec else
                  paint('DDS UNSECURED (no workspace: source `ws env <name>`)', COL['WARN'], c)))
        out = [paint(time.strftime('framework  %H:%M:%S'), '1', c) + '   ' + where, '']
        # LLM
        st = self.llm.get('state', '?')
        out.append(paint(f'LLM  {st}', COL['ERROR'] if st == 'down' else
                         COL['WARN'] if st == 'degraded' else COL['ONLINE'], c) +
                   f"   {self.llm.get('reason', '')}")
        for e in self.llm.get('endpoints', []):
            lk = self.links.get(e['url'])
            if lk is None:
                link = 'checking...'
            elif lk['ms'] is None:
                link = paint(f"{lk['kind']:<8} unreachable", COL['ERROR'], c)
            else:
                link = f"{lk['kind']:<8} {lk['ms']:6.1f} ms"
            out.append('   ' + paint('up  ' if e['available'] else 'DOWN',
                                     COL['ONLINE'] if e['available'] else COL['ERROR'], c) +
                       f"  {e['url']:<34} {link}")
        if self.battery and now - self.battery[2] < 10.0:
            lvl, txt = battery_state(self.battery[0], self.battery[1])
            out.append(paint(txt, COL[lvl] if lvl in COL else COL['OFFLINE'], c))
        # task lifecycle (orchestrator)
        t = self.task
        if t:
            ts = t.get('state', '?')
            out.append(paint(f'task {ts}', COL['ERROR'] if ts == 'failed' else
                             COL['BUSY'] if ts in ('running', 'repairing') else COL['ONLINE'], c) +
                       f"   repairs {t.get('repairs', 0)}/{t.get('max_repairs', 0)}  "
                       f"{t.get('detail', '') or t.get('task', '')[:50]}")
        if self.tree and self.show_tree:
            done = sum(1 for n in self.tree if self.nstate.get(str(n['uid'])) in ('S', 'F'))
            out += ['', f'plan tree  ({done}/{len(self.tree)} nodes finished; t hides)']
            out += ['  ' + ln for ln in render_tree(self.tree, self.nstate, c)]
        # cluster
        if self.cluster:
            cs = self.cluster
            out += ['', f"cluster  scheduler {cs.scheduler_id}  alive: {', '.join(cs.hosts_alive) or '-'}"
                        f"  dead: {', '.join(cs.hosts_dead) or '-'}"]
            for j in cs.jobs:
                col = {'running': 'ONLINE', 'backoff': 'WARN', 'failed': 'ERROR'}.get(
                    j.job_state, 'OFFLINE' if j.state != 'placed' else 'ONLINE')
                out.append(f'   {j.job_id:<18} {j.host_id or "-":<12} ' +
                           paint(j.job_state if j.state == "placed" else j.state, COL[col], c))
        if self.devices:
            out += ['', 'devices'] + self.device_lines(c)
        # nodes
        out += ['', 'nodes']
        self.names = names
        self.sel = min(self.sel, max(len(names) - 1, 0))
        for i, (name, s, note) in enumerate(self.node_rows(names, online, now)):
            out.append(('>> ' if i == self.sel and self.keys else '   ') + f'{name:<30}' + paint(f'{s:<8}', COL[s], c) + note)
        if self.keys:
            out += ['', paint(self.msg, COL['WARN'], c) if self.msg else '',
                    'j/k select  r/R restart node (graceful/force)  s/S stop system  '
                    'w/W restart system  h halt task  t plan tree']
        return '\n'.join(out)


def logs(node, a, color):
    floor = LEVEL_NUM[a.level]

    def show(m):
        if m.level < floor or (a.node and a.node not in m.name):
            return
        lvl, code = LEVEL.get(m.level, (str(m.level), '0'))
        t = time.strftime('%H:%M:%S', time.localtime(m.stamp.sec))
        print(f'{t} ' + paint(f'{lvl:<5}', code, color) + f' {m.name}: {m.msg}', flush=True)
    node.create_subscription(Log, '/rosout', show, 1000)
    rclpy.spin(node)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--logs', action='store_true', help='verbose: stream /rosout')
    ap.add_argument('--level', default='info', choices=sorted(LEVEL_NUM, key=LEVEL_NUM.get))
    ap.add_argument('--node', default='', help='only logs from nodes whose name contains this')
    ap.add_argument('--once', action='store_true', help='print one frame and exit')
    a = ap.parse_args(argv)
    color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
    rclpy.init()
    node = rclpy.create_node('framework_dash')
    keys = False
    try:
        if a.logs:
            logs(node, a, color)
            return 0
        keys = sys.stdin.isatty() and sys.stdout.isatty() and not a.once
        d, exp = Dash(node, color, keys), expected_nodes()
        if keys:
            old_tty = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
        end = time.monotonic() + 2.5      # let latched topics and the graph arrive
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        d.device_lines(False)             # subscribe to the device topics, then measure ~2 s
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if a.once:     # give the background link check one round before the single frame
            end = time.monotonic() + 6.0
            while time.monotonic() < end and not all(
                    e['url'] in d.links for e in d.llm.get('endpoints', [])):
                rclpy.spin_once(node, timeout_sec=0.1)
        while True:
            frame = d.frame(exp)
            print(frame if a.once else '\033[H\033[J' + frame, flush=True)
            if a.once:
                return 0
            t = time.monotonic() + 1.0
            while time.monotonic() < t:
                rclpy.spin_once(node, timeout_sec=0.05)
                if keys and select.select([sys.stdin], [], [], 0)[0]:
                    k = os.read(sys.stdin.fileno(), 8).decode(errors='replace')
                    if d.key(k):
                        break
    except KeyboardInterrupt:
        return 0
    finally:
        if keys:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
