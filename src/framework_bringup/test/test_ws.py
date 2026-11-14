import importlib.util
import json
import os
import socket
import ssl
import subprocess
import threading
import time

import pytest
import yaml

spec = importlib.util.spec_from_file_location(
    'ws', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'ws.py'))
ws = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ws)


def test_names_domain_host_and_invite():
    assert ws.check_name('lab-1') == 'lab-1'
    for bad in ('', 'Lab', '-x', 'a b', 'x' * 33, '../x'):
        with pytest.raises(ws.WsError):
            ws.check_name(bad)
    assert 40 <= ws.domain_for('lab') <= 89 and ws.domain_for('lab') == ws.domain_for('lab')
    assert ws.host_label('My_PC.local') == 'my-pc-local'
    inv = ws.make_invite('lab', '10.0.0.2', 47610, 'ab' * 32, 'tok_en-1')
    p = ws.parse_invite(inv)
    assert p['name'] == 'lab' and p['host'] == '10.0.0.2' and p['port'] == 47610
    assert p['fp'] == 'ab' * 8 and p['token'] == 'tok_en-1'
    with pytest.raises(ws.WsError):
        ws.parse_invite('hello')


def test_proof_binds_every_field():
    base = ws.proof('t', 'S', 'nc', 'ns', 'fp', 'host')
    assert base == ws.proof('t', 'S', 'nc', 'ns', 'fp', 'host')
    for args in (('x', 'S', 'nc', 'ns', 'fp', 'host'), ('t', 'C', 'nc', 'ns', 'fp', 'host'),
                 ('t', 'S', 'n2', 'ns', 'fp', 'host'), ('t', 'S', 'nc', 'n2', 'fp', 'host'),
                 ('t', 'S', 'nc', 'ns', 'f2', 'host'), ('t', 'S', 'nc', 'ns', 'fp', 'h2')):
        assert ws.proof(*args) != base


@pytest.fixture()
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOCUP_WS_HOME', str(tmp_path))
    return tmp_path


class Args:
    def __init__(self, **k):
        self.__dict__.update(dict(domain=None, port=0, host_name='owner', address='127.0.0.1'), **k)


@pytest.fixture()
def served(home):
    ws.cmd_create(Args(name='lab'))
    adm = ws.Admitter('lab')
    stop, got = threading.Event(), {}
    t = threading.Thread(target=ws.serve_forever, args=(adm, '127.0.0.1', 0, '127.0.0.1'),
                         kwargs=dict(stop=stop, ready=lambda p: got.update(port=p),
                                     announce_port=47999), daemon=True)
    t.start()
    for _ in range(100):
        if 'port' in got:
            break
        time.sleep(0.05)
    yield adm, got['port']
    stop.set()
    t.join(3)


def test_create_binds_permissions_to_the_workspace_domain_not_the_shell_one(home, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')                      # the trap seen in practice
    ws.cmd_create(Args(name='dom', domain=57))
    perms = (home / 'workspaces' / 'dom' / 'keystore' / 'enclaves' / 'dom' / 'owner' /
             'permissions.xml').read_text()
    assert '<id>57</id>' in perms and '<id>30</id>' not in perms
    assert oct((home / 'workspaces' / 'dom' / 'secret.yaml').stat().st_mode & 0o777) == '0o600'
    assert oct((home / 'workspaces' / 'dom' / 'server.key').stat().st_mode & 0o777) == '0o600'
    with pytest.raises(ws.WsError):
        ws.cmd_create(Args(name='dom'))                            # no overwriting


def test_join_with_the_token_installs_a_working_enclave(served, home):
    adm, port = served
    b = ws.join_remote('127.0.0.1', port, adm.secret['token'], 'laptop2', adm.fp[:16])
    d = ws.install_joined('lab', 'laptop2', b)
    enc = os.path.join(d, 'keystore', 'enclaves', 'lab', 'laptop2')
    assert set(ws.ENCLAVE_FILES) <= set(os.listdir(enc))
    assert oct(os.stat(os.path.join(enc, 'key.pem')).st_mode & 0o777) == '0o600'
    assert open(os.path.join(d, 'llm.key')).read() == adm.secret['llm_key']
    env = open(os.path.join(d, 'env.sh')).read()
    assert 'ROS_SECURITY_ENABLE=true' in env and 'Enforce' in env and '/lab/laptop2' in env
    assert adm.secret['llm_key'] not in env                       # the key is read from its file
    members = yaml.safe_load(open(home / 'workspaces' / 'lab' / 'members.yaml'))
    assert [m['host'] for m in members] == ['owner', 'laptop2']
    ws.join_remote('127.0.0.1', port, adm.secret['token'], 'laptop2', None)   # re-join is fine
    assert len(yaml.safe_load(open(home / 'workspaces' / 'lab' / 'members.yaml'))) == 2


def test_wrong_token_is_refused_by_the_client_before_anything_is_sent(served):
    adm, port = served
    with pytest.raises(ws.WsError, match='does not know the token'):
        ws.join_remote('127.0.0.1', port, 'not-the-token', 'evil')
    time.sleep(0.3)
    assert ('admitted', 'evil') not in adm.log


def test_a_client_that_cannot_prove_the_token_gets_no_enclave(served):
    """Bypass the client-side check: a hand-written client with a bad proof_c must be refused."""
    adm, port = served
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
    with socket.create_connection(('127.0.0.1', port), timeout=5) as raw, ctx.wrap_socket(raw) as tls:
        f = tls.makefile('rwb')
        ws.send_json(f, {'op': 'join', 'host': 'evil', 'nc': 'aa' * 8})
        ws.recv_json(f, 4096)
        ws.send_json(f, {'proof_c': '0' * 64})
        r = ws.recv_json(f, 1 << 20)
    assert r['ok'] is False and 'files' not in r
    assert not os.path.isdir(os.path.join(adm.wdir, 'keystore', 'enclaves', 'lab', 'evil'))


def test_impostor_server_that_does_not_know_the_token_gets_nothing(home):
    got = []
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    d = str(home)
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'ec', '-pkeyopt', 'ec_paramgen_curve:prime256v1',
                    '-nodes', '-keyout', 'k.pem', '-out', 'c.pem', '-days', '2', '-subj', '/CN=x'],
                   cwd=d, check=True, capture_output=True)
    ctx.load_cert_chain(os.path.join(d, 'c.pem'), os.path.join(d, 'k.pem'))
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)

    def fake():
        c, _ = srv.accept()
        tls = ctx.wrap_socket(c, server_side=True)
        f = tls.makefile('rwb')
        ws.recv_json(f, 4096)
        ws.send_json(f, {'ns': 'bb' * 8, 'proof_s': '0' * 64})
        tls.settimeout(2.0)
        try:
            got.append(f.readline())              # the client must NOT answer with a proof
        except OSError:
            got.append(b'')                       # silence until the timeout also means no proof
    t = threading.Thread(target=fake, daemon=True)
    t.start()
    with pytest.raises(ws.WsError, match='does not know the token'):
        ws.join_remote('127.0.0.1', srv.getsockname()[1], 'real-token', 'me')
    t.join(3)
    assert got == [b'']                         # exactly: the thread ran and saw no proof_c


def test_relaying_man_in_the_middle_fails_on_channel_binding(served, home):
    """A relay that terminates TLS with ITS cert and forwards to the real server: the real
    server's proof covers the real cert, the client saw the relay's cert -> mismatch."""
    adm, real_port = served
    d = str(home)
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'ec', '-pkeyopt', 'ec_paramgen_curve:prime256v1',
                    '-nodes', '-keyout', 'rk.pem', '-out', 'rc.pem', '-days', '2', '-subj', '/CN=relay'],
                   cwd=d, check=True, capture_output=True)
    sctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    sctx.load_cert_chain(os.path.join(d, 'rc.pem'), os.path.join(d, 'rk.pem'))
    cctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    cctx.check_hostname, cctx.verify_mode = False, ssl.CERT_NONE
    relay = socket.socket()
    relay.bind(('127.0.0.1', 0))
    relay.listen(1)
    forwarded = []

    def run():
        c, _ = relay.accept()
        a = sctx.wrap_socket(c, server_side=True).makefile('rwb')
        up = cctx.wrap_socket(socket.create_connection(('127.0.0.1', real_port)))
        b = up.makefile('rwb')
        try:
            for src, dst in ((a, b), (b, a), (a, b), (b, a)):
                line = src.readline()
                if not line:
                    break
                dst.write(line)
                dst.flush()
                forwarded.append(json.loads(line))
        except OSError:
            pass
    t = threading.Thread(target=run, daemon=True)
    t.start()
    with pytest.raises(ws.WsError, match='does not know the token'):
        ws.join_remote('127.0.0.1', relay.getsockname()[1], adm.secret['token'], 'victim')
    t.join(3)
    assert ('admitted', 'victim') not in adm.log
    # control: the relay really worked (client hello and the real server's proof went through),
    # so the client refused because of the certificate mismatch and not a broken relay
    assert [list(m)[0] for m in forwarded[:2]] == ['op', 'ns']
    assert 'files' not in json.dumps(forwarded)


def test_pinned_fingerprint_mismatch_is_refused(served):
    adm, port = served
    with pytest.raises(ws.WsError, match='not the one'):
        ws.join_remote('127.0.0.1', port, adm.secret['token'], 'x', 'deadbeef')


def test_discovery_finds_an_announcing_workspace(served):
    adm, port = served
    hits = ws.discover(timeout=4.0, port=47999, bind='127.0.0.1')
    assert any(h['name'] == 'lab' and h['port'] == port and h['fp'] == adm.fp for h in hits)


def test_role_policies():
    import re
    full, op, obs = (ws.policy_xml('lab', 'h', r) for r in ('full', 'operator', 'observer'))

    def pub(xml):
        return set(re.findall(r'<topic>([^<]*)</topic>', re.search(r'<topics publish="ALLOW">(.*?)</topics>', xml).group(1)))
    assert pub(full) == {'/*'}
    assert '/bt/task' in pub(op) and '/teleop/cmd_vel' in pub(op) and '/cmd_vel' not in pub(op)
    assert pub(obs) == {'/rosout', '/parameter_events'}                 # observers publish only housekeeping
    assert 'path="/lab/h"' in obs
    for r in ('operator', 'observer'):                                    # every role may serve its own parameter services
        assert '/*/get_parameters' in ws.ROLES[r]['reply']
    with pytest.raises(ws.WsError):
        ws.issue_enclave('/nonexistent', 'lab', 40, 'h', 'root')


def test_invite_roles_roundtrip():
    inv = ws.make_invite('lab', '10.0.0.5', 47610, 'ab' * 32, 'tok', 'observer')
    assert inv.endswith('~observer') and ws.parse_invite(inv)['role'] == 'observer'
    assert ws.parse_invite(ws.make_invite('lab', '10.0.0.5', 47610, 'ab' * 32, 'tok'))['role'] == 'full'
    with pytest.raises(ws.WsError):
        ws.parse_invite(inv.replace('observer', 'root'))
    assert ws.role_tokens({'token': 'T', 'tokens': {'observer': 'O'}}) == {'full': 'T', 'observer': 'O'}
