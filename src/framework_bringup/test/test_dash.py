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
    import time
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
