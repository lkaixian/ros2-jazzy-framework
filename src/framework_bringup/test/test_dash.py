import importlib.util
import os

spec = importlib.util.spec_from_file_location(
    'dash', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'dash.py'))
dash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dash)


def test_classify_priority():
    c = dash.classify
    assert c(False, None) == 'OFFLINE'
    assert c(False, 1.0, busy=True) == 'OFFLINE'          # gone beats everything
    assert c(True, 5.0, busy=True) == 'ERROR'             # recent error beats busy
    assert c(True, 31.0, busy=True) == 'BUSY'             # old errors expire
    assert c(True, None, down=True) == 'ERROR'
    assert c(True, None) == 'ONLINE'


def test_paint_and_plain():
    assert dash.paint('x', '31', False) == 'x'
    assert dash.paint('x', '31', True).startswith('\033[31m')


def test_expected_nodes_come_from_check_params():
    names = dash.expected_nodes()
    assert 'llm_bridge_node' in names and 'cmd_vel_mux_node' in names


def test_parse_route_dev():
    assert dash.parse_route_dev('192.168.1.50 via 192.168.1.1 dev wlp3s0 src 192.168.1.9 uid 1000') == 'wlp3s0'
    assert dash.parse_route_dev('local 127.0.0.1 dev lo src 127.0.0.1 uid 1000') == 'lo'
    assert dash.parse_route_dev('') == '' and dash.parse_route_dev(None) == ''


def test_link_type_from_sysfs(tmp_path):
    (tmp_path / 'wlan0' / 'wireless').mkdir(parents=True)
    (tmp_path / 'wlan1' / 'phy80211').mkdir(parents=True)
    (tmp_path / 'enp2s0' / 'device').mkdir(parents=True)
    (tmp_path / 'docker0').mkdir()
    lt = lambda d: dash.link_type(d, str(tmp_path))      # noqa: E731
    assert lt('wlan0') == 'wireless' and lt('wlan1') == 'wireless'
    assert lt('enp2s0') == 'wired' and lt('docker0') == 'virtual'
    assert lt('lo') == 'local' and lt('') == '?' and lt('gone0') == '?'


def test_tcp_ms_measures_open_port_and_rejects_closed_one():
    import socket
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    ms = dash.tcp_ms('127.0.0.1', port)
    srv.close()
    assert ms is not None and 0.0 <= ms < 500.0
    assert dash.tcp_ms('127.0.0.1', port, timeout=0.5) is None       # nothing listens any more


def test_check_link_on_this_machine():
    import socket
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)
    r = dash.check_link(f'http://127.0.0.1:{srv.getsockname()[1]}')
    srv.close()
    assert r['kind'] == 'local' and r['dev'] == 'lo' and r['ms'] is not None


def test_check_link_unreachable_has_no_ms():
    import socket
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    port = srv.getsockname()[1]
    srv.close()                                      # nothing listens: connection refused
    r = dash.check_link(f'http://127.0.0.1:{port}')
    assert r['ms'] is None and r['kind'] == 'local'


def test_find_pids_and_kill():
    import subprocess
    p = subprocess.Popen(['sleep', '60'])
    try:
        assert p.pid in dash.find_pids('sleep')
        dash.kill(p.pid, False)
        assert p.wait(5) != 0
        assert p.pid not in dash.find_pids('sleep')
    finally:
        p.kill()


def test_preflight_llm_verdict():
    spec = importlib.util.spec_from_file_location(
        'preflight', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'preflight.py'))
    pf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pf)
    assert pf.llm_verdict(None)[0] == 'FAIL'
    assert pf.llm_verdict({'state': 'down', 'endpoints': []})[0] == 'FAIL'
    up = {'state': 'ok', 'endpoints': [{'available': True}]}
    assert pf.llm_verdict(up)[0] == 'PASS'
    assert pf.llm_verdict({**up, 'state': 'degraded'})[0] == 'WARN'


def test_teleop_keys_and_priority():
    spec = importlib.util.spec_from_file_location(
        'teleop', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'teleop.py'))
    tp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tp)
    assert tp.twist_for('w') == (0.2, 0.0) and tp.twist_for('S') == (-0.2, 0.0)
    assert tp.twist_for('a')[1] > 0 > tp.twist_for('d')[1] and tp.twist_for(' ') == (0.0, 0.0)
    assert tp.twist_for('x') is None
    assert 'w/s' in tp.HELP and 'quit' in tp.HELP and 'r reset' in tp.HELP
    assert tp.twist_for('r') is None       # r resets the stop latch, it never drives
    # (the start-up help line above: its old lookup raised IndexError)
    import yaml
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'param', 'extensions.yaml')))
    assert cfg['cmd_vel_mux_node']['ros__parameters']['inputs'][0] == 'teleop/cmd_vel'   # manual override wins


def test_preflight_local_checks():
    from types import SimpleNamespace as N
    spec = importlib.util.spec_from_file_location(
        'preflight2', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'preflight.py'))
    pf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pf)
    ok = pf.local_checks(lambda *a, **k: N(stdout='yes\n'), lambda p: N(free=50e9))
    assert [l for l, _ in ok] == ['PASS', 'PASS']
    bad = pf.local_checks(lambda *a, **k: N(stdout='no\n'), lambda p: N(free=0.5e9))
    assert [l for l, _ in bad] == ['WARN', 'FAIL']
    missing = pf.local_checks(lambda *a, **k: (_ for _ in ()).throw(FileNotFoundError()), lambda p: N(free=3e9))
    assert [l for l, _ in missing] == ['WARN', 'WARN'] and 'unavailable' in missing[0][1]


def test_battery_state():
    assert dash.battery_state(158)[0] == 'ONLINE' and '15.8 V' in dash.battery_state(158)[1]
    assert dash.battery_state(135)[0] == 'WARN'
    assert dash.battery_state(130)[0] == 'ERROR' and 'CRITICAL' in dash.battery_state(130)[1]
    assert dash.battery_state(150, charger=2)[1].endswith('(charging)')
    assert dash.battery_state(0)[0] == 'OFFLINE'


def test_render_tree_status_and_window():
    tree = [{'uid': 1, 'depth': 0, 'name': 'Sequence', 'parent': -1},
            {'uid': 2, 'depth': 1, 'name': 'NavigateToPlace', 'parent': 1, 'ports': {'place': 'door'}},
            {'uid': 3, 'depth': 1, 'name': 'Speak', 'parent': 1, 'ports': {'text': 'hi'}}]
    lines = dash.render_tree(tree, {'1': 'R', '2': 'S', '3': 'F'}, color=False)
    assert lines == ['> Sequence', '  + NavigateToPlace(place=door)', '  x Speak(text=hi)']
    assert dash.render_tree(tree, {}, color=False)[1].startswith('  . ')            # not reached yet
    big = [{'uid': i, 'depth': 0, 'name': f'N{i}', 'parent': -1} for i in range(60)]
    win = dash.render_tree(big, {'30': 'R'}, color=False, max_lines=10)
    assert len(win) == 10 and any('N30' in l for l in win) and 'more above' in win[0] and 'more below' in win[-1]


def test_preflight_chrony_offset():
    import pytest
    from types import SimpleNamespace as N
    spec = importlib.util.spec_from_file_location(
        'preflight3', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'preflight.py'))
    pf = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pf)
    fast = 'Reference ID : C0A8010A\nSystem time     : 0.000123456 seconds fast of NTP time\n'
    assert pf.parse_chrony_offset(fast) == pytest.approx(0.000123456)
    assert pf.parse_chrony_offset(fast.replace('fast', 'slow')) == pytest.approx(-0.000123456)
    assert pf.parse_chrony_offset('garbage') is None

    def chrony(sec, direction='fast'):
        return lambda cmd, **k: N(stdout=f'System time     : {sec} seconds {direction} of NTP time\n' if cmd[0] == 'chronyc' else 'yes\n')
    ok = pf.local_checks(chrony(0.002), lambda p: N(free=50e9))
    assert ok[0][0] == 'PASS' and 'ms (chrony)' in ok[0][1] and len(ok) == 2        # chrony replaces the NTP-flag line
    assert pf.local_checks(chrony(0.2, 'slow'), lambda p: N(free=50e9))[0][0] == 'WARN'
    assert pf.local_checks(chrony(2.0), lambda p: N(free=50e9))[0][0] == 'FAIL'


def test_device_state():
    s = dash.device_state
    assert s(False, None, '/scan') == 'ERROR'
    assert s(False, None, '', optional=True) == 'OFFLINE'
    assert s(True, None, '/scan') == 'WARN'         # plugged in, topic not advertised
    assert s(True, 0.0, '/scan') == 'WARN'          # advertised, nothing arriving
    assert s(True, 10.0, '/scan') == 'ONLINE'
    assert s(True, None, '') == 'ONLINE'            # presence-only device


def test_launch_pids_ignores_the_kobuki_driver(monkeypatch):
    ros = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch']
    monkeypatch.setattr(dash, 'cmdlines', lambda: [
        (1, ros + ['kobuki_node', 'kobuki_node-launch.py']),
        (2, ros + ['framework_bringup', 'framework.launch.py', 'enable_audio:=true'])])
    assert dash.launch_pids() == [2]
    monkeypatch.setattr(dash, 'cmdlines', lambda: [(1, ros + ['kobuki_node', 'kobuki_node-launch.py'])])
    assert dash.launch_pids() == [1]       # no framework launch: old behaviour
