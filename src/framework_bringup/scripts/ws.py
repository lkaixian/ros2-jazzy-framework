#!/usr/bin/python3
"""ws: workspaces. A workspace = an SROS2 keystore (authentication + encryption for every ROS
node), a DDS domain, an LLM API key and a join token. Machines on the same network discover it and
join with the token; a machine without the keys cannot see or talk to the nodes.

  ros2 run framework_bringup ws create lab             # on the first machine; prints the invite
  ros2 run framework_bringup ws serve lab              # keeps admitting machines + announces itself
  ros2 run framework_bringup ws discover               # on another machine: what is on this LAN?
  ros2 run framework_bringup ws join lab --token ...   # (or --invite <string>)
  source $(ros2 run framework_bringup ws env lab)      # every terminal that should be in it
  ros2 run framework_bringup ws list | members lab | invite lab

Join protocol (TLS, self-signed cert; both sides prove they know the token, bound to the TLS cert
the client actually saw, so a man in the middle that terminates TLS fails):
  C->S {op,host,nc}   S->C {ns, proof_s}   (client aborts here if proof_s is wrong)
  C->S {proof_c}      S->C {ok, domain, llm_key, peer, files{enclave files}}
Authentication + encryption only: every member may publish/subscribe every topic (per-topic
permissions are not generated), and removing one member means recreating the workspace.
"""
import argparse
import base64
import getpass
import hashlib
import hmac
import json
import os
import re
import secrets
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import zlib

import yaml

PORT, ANNOUNCE_PORT = 47610, 47611
NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,31}$')
ENCLAVE_FILES = ('cert.pem', 'key.pem', 'identity_ca.cert.pem', 'permissions_ca.cert.pem',
                 'governance.p7s', 'permissions.p7s', 'permissions.xml')
CYCLONE_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><AllowMulticast>false</AllowMulticast></General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers><Peer address="${ROBOCUP_PEER:-localhost}"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""


class WsError(Exception):
    pass


# ------------------------------------------------------------------ pure helpers
def check_name(name):
    if not NAME_RE.match(name or ''):
        raise WsError(f'workspace name {name!r}: use a-z 0-9 _ - (1-32 chars, start with a letter/digit)')
    return name


def domain_for(name):
    """Stable DDS domain in 40..89 (away from 0 and the shell default)."""
    return 40 + zlib.crc32(name.encode()) % 50


def host_label(name):
    s = re.sub(r'[^a-z0-9-]', '-', (name or '').lower()).strip('-')[:32]
    if not s:
        raise WsError('host name has no usable characters')
    return s


def proof(token, role, nc, ns, fp, host):
    """HMAC over the whole transcript, bound to the TLS certificate fingerprint."""
    return hmac.new(token.encode(), f'{role}|{nc}|{ns}|{fp}|{host}'.encode(),
                    hashlib.sha256).hexdigest()


def make_invite(name, host, port, fp, token):
    return '~'.join(['rcw1', name, host, str(port), fp[:16], token])   # '~': IPs contain dots


def parse_invite(text):
    parts = (text or '').strip().split('~', 5)
    if len(parts) != 6 or parts[0] != 'rcw1':
        raise WsError('not an invite string (expected rcw1~<name>~<host>~<port>~<fp>~<token>)')
    _, name, host, port, fp, token = parts
    return {'name': check_name(name), 'host': host, 'port': int(port), 'fp': fp, 'token': token}


def fingerprint(der):
    return hashlib.sha256(der).hexdigest()


def env_text(d, ws, host, peer, key_file):
    return f"""# workspace '{ws}' on {host}: source this in every terminal that should be part of it
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID={d['domain']}
export ROS_SECURITY_KEYSTORE={d['dir']}/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ENCLAVE_OVERRIDE=/{ws}/{host}
export ROBOCUP_PEER={peer}
export CYCLONEDDS_URI=file://{d['dir']}/cyclonedds.xml
export LLM_API_KEY="$(cat {key_file})"
export LLAMA_API_KEY="$LLM_API_KEY"
"""


def base_dirs():
    root = os.path.expanduser(os.environ.get('ROBOCUP_WS_HOME', '~/.robocup'))
    return os.path.join(root, 'workspaces'), os.path.join(root, 'joined')


def write_private(path, data):
    mode = 0o600
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    with os.fdopen(fd, 'wb' if isinstance(data, bytes) else 'w') as f:
        f.write(data)


# ------------------------------------------------------------------ owner side
def ros2_security(args, domain, cwd):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))     # permissions are bound to this domain
    r = subprocess.run(['ros2', 'security'] + args, cwd=cwd, env=env, capture_output=True,
                       text=True, timeout=60)
    if r.returncode:
        raise WsError(f"ros2 security {' '.join(args)} failed: {r.stderr.strip()[-300:]}")


def issue_enclave(wdir, ws, domain, host):
    """Create (or reuse) the enclave /<ws>/<host>; returns {filename: base64}."""
    edir = os.path.join(wdir, 'keystore', 'enclaves', ws, host)
    if not os.path.isdir(edir):
        ros2_security(['create_enclave', 'keystore', f'/{ws}/{host}'], domain, wdir)
    return {n: base64.b64encode(open(os.path.join(edir, n), 'rb').read()).decode()
            for n in ENCLAVE_FILES}


def cmd_create(a):
    ws = check_name(a.name)
    wroot, _ = base_dirs()
    wdir = os.path.join(wroot, ws)
    if os.path.exists(wdir):
        raise WsError(f'workspace {ws} already exists in {wdir}')
    domain = a.domain if a.domain is not None else domain_for(ws)
    if not 1 <= domain <= 232:
        raise WsError('domain must be 1..232')
    os.makedirs(wdir, mode=0o700)
    me = host_label(a.host_name or socket.gethostname())
    ros2_security(['create_keystore', 'keystore'], domain, wdir)
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'ec', '-pkeyopt',
                    'ec_paramgen_curve:prime256v1', '-nodes', '-keyout', 'server.key', '-out',
                    'server.pem', '-days', '825', '-subj', f'/CN=robocup-ws-{ws}'], cwd=wdir,
                   check=True, capture_output=True)
    os.chmod(os.path.join(wdir, 'server.key'), 0o600)
    secret = {'token': secrets.token_urlsafe(16), 'llm_key': secrets.token_urlsafe(24)}
    write_private(os.path.join(wdir, 'secret.yaml'), yaml.safe_dump(secret))
    meta = {'name': ws, 'domain': domain, 'port': a.port, 'created': time.strftime('%F %T'),
            'owner_host': me}
    open(os.path.join(wdir, 'workspace.yaml'), 'w').write(yaml.safe_dump(meta))
    issue_enclave(wdir, ws, domain, me)
    open(os.path.join(wdir, 'members.yaml'), 'w').write(
        yaml.safe_dump([{'host': me, 'joined': meta['created'], 'role': 'owner'}]))
    open(os.path.join(wdir, 'cyclonedds.xml'), 'w').write(CYCLONE_XML)
    write_private(os.path.join(wdir, 'llm.key'), secret['llm_key'])
    write_private(os.path.join(wdir, 'env.sh'), env_text(
        {'domain': domain, 'dir': wdir}, ws, me, 'localhost', os.path.join(wdir, 'llm.key')))
    fp = fingerprint(ssl.PEM_cert_to_DER_cert(open(os.path.join(wdir, 'server.pem')).read()))
    me_addr = a.address or local_address()
    print(f'workspace {ws}: domain {domain}, keystore + enclave for {me} in {wdir}')
    print(f'  1. source {wdir}/env.sh        (this machine, every terminal)')
    print(f'  2. ros2 run framework_bringup ws serve {ws}     (admit other machines)')
    print('  invite (secret: anyone with it can join):')
    print('   ', make_invite(ws, me_addr, a.port, fp, secret['token']))


def load_ws(name):
    wdir = os.path.join(base_dirs()[0], check_name(name))
    if not os.path.isdir(wdir):
        raise WsError(f'no workspace {name} here (created workspaces: {list_dir(base_dirs()[0])})')
    meta = yaml.safe_load(open(os.path.join(wdir, 'workspace.yaml')))
    secret = yaml.safe_load(open(os.path.join(wdir, 'secret.yaml')))
    fp = fingerprint(ssl.PEM_cert_to_DER_cert(open(os.path.join(wdir, 'server.pem')).read()))
    return wdir, meta, secret, fp


def list_dir(p):
    return sorted(os.listdir(p)) if os.path.isdir(p) else []


def local_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def recv_json(f, limit):
    line = f.readline(limit + 1)
    if not line or len(line) > limit:
        raise WsError('connection closed or message too large')
    return json.loads(line)


def send_json(f, obj):
    f.write(json.dumps(obj).encode() + b'\n')
    f.flush()


class Admitter:
    def __init__(self, name):
        self.wdir, self.meta, self.secret, self.fp = load_ws(name)
        self.lock = threading.Lock()          # ros2 security / members.yaml are not re-entrant
        self.log = []

    def handle(self, tls):
        f = tls.makefile('rwb')
        try:
            req = recv_json(f, 4096)
            host = host_label(req.get('host', ''))
            nc = str(req.get('nc', ''))[:64]
            if req.get('op') != 'join' or not re.fullmatch(r'[0-9a-f]{8,64}', nc):
                raise WsError('bad request')
            ns = secrets.token_hex(16)
            token = self.secret['token']
            send_json(f, {'ns': ns, 'proof_s': proof(token, 'S', nc, ns, self.fp, host)})
            got = str(recv_json(f, 4096).get('proof_c', ''))
            if not hmac.compare_digest(got, proof(token, 'C', nc, ns, self.fp, host)):
                time.sleep(1.0)                # slows guessing; the token has 128 bits anyway
                send_json(f, {'ok': False, 'error': 'bad proof'})
                self.log.append(('refused', host))
                return
            with self.lock:
                files = issue_enclave(self.wdir, self.meta['name'], self.meta['domain'], host)
                members = yaml.safe_load(open(os.path.join(self.wdir, 'members.yaml'))) or []
                if host not in [m['host'] for m in members]:
                    members.append({'host': host, 'joined': time.strftime('%F %T'),
                                    'role': 'member'})
                    open(os.path.join(self.wdir, 'members.yaml'), 'w').write(yaml.safe_dump(members))
            send_json(f, {'ok': True, 'domain': self.meta['domain'], 'llm_key': self.secret['llm_key'],
                          'peer': self.meta.get('address') or local_address(), 'files': files})
            self.log.append(('admitted', host))
        except (WsError, ValueError, OSError, KeyError) as e:
            try:
                send_json(f, {'ok': False, 'error': str(e)[:100]})
            except OSError:
                pass
        finally:
            tls.close()


def serve_forever(adm, host, port, announce_to, stop=None, ready=None, announce_port=ANNOUNCE_PORT):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(os.path.join(adm.wdir, 'server.pem'), os.path.join(adm.wdir, 'server.key'))
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    srv.settimeout(0.5)
    port = srv.getsockname()[1]
    if ready:
        ready(port)
    ann = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ann.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    msg = json.dumps({'proto': 1, 'name': adm.meta['name'], 'port': port,
                      'fp': adm.fp}).encode()
    last = 0.0
    while not (stop and stop.is_set()):
        if time.monotonic() - last > 2.0:
            last = time.monotonic()
            try:
                ann.sendto(msg, (announce_to, announce_port))
            except OSError:
                pass
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        conn.settimeout(10)
        try:
            tls = ctx.wrap_socket(conn, server_side=True)
        except (ssl.SSLError, OSError):
            conn.close()
            continue
        threading.Thread(target=adm.handle, args=(tls,), daemon=True).start()
    srv.close()
    ann.close()


def cmd_serve(a):
    adm = Admitter(a.name)
    adm.meta['address'] = a.address or local_address()
    print(f'serving workspace {a.name} on {a.bind}:{adm.meta["port"]} (announcing on UDP '
          f'{a.announce_to}:{ANNOUNCE_PORT}); Ctrl-C to stop')
    try:
        serve_forever(adm, a.bind, adm.meta['port'], a.announce_to)
    except KeyboardInterrupt:
        pass


# ------------------------------------------------------------------ joining side
def discover(timeout=4.0, port=ANNOUNCE_PORT, bind='0.0.0.0'):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((bind, port))
    s.settimeout(0.3)
    found, end = {}, time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            data, (ip, _) = s.recvfrom(2048)
            m = json.loads(data)
            if m.get('proto') == 1 and NAME_RE.match(m.get('name', '')):
                found[(m['name'], ip)] = {'name': m['name'], 'host': ip, 'port': int(m['port']),
                                          'fp': m['fp']}
        except (socket.timeout, ValueError, KeyError):
            pass
    s.close()
    return list(found.values())


def join_remote(host, port, token, me, expect_fp=None, timeout=10):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE          # identity comes from the token proof, bound to this cert
    with socket.create_connection((host, port), timeout=timeout) as raw, \
            ctx.wrap_socket(raw) as tls:
        fp = fingerprint(tls.getpeercert(binary_form=True))
        if expect_fp and not fp.startswith(expect_fp):
            raise WsError('the server certificate is not the one announced/invited: not joining')
        f = tls.makefile('rwb')
        nc = secrets.token_hex(16)
        send_json(f, {'op': 'join', 'host': me, 'nc': nc})
        r = recv_json(f, 4096)
        if not hmac.compare_digest(str(r.get('proof_s', '')), proof(token, 'S', nc, r.get('ns', ''), fp, me)):
            raise WsError('the server does not know the token (wrong token, or not the real '
                          'workspace host): nothing was sent')
        send_json(f, {'proof_c': proof(token, 'C', nc, r['ns'], fp, me)})
        bundle = recv_json(f, 1 << 20)
    if not bundle.get('ok'):
        raise WsError(f"refused: {bundle.get('error', 'unknown')}")
    return bundle


def install_joined(ws, me, bundle):
    _, jroot = base_dirs()
    d = os.path.join(jroot, ws)
    edir = os.path.join(d, 'keystore', 'enclaves', ws, me)
    os.makedirs(edir, mode=0o700, exist_ok=True)
    for n, b in bundle['files'].items():
        if n not in ENCLAVE_FILES:
            raise WsError(f'unexpected file {n} in the bundle')
        (write_private if n == 'key.pem' else lambda p, x: open(p, 'wb').write(x))(
            os.path.join(edir, n), base64.b64decode(b))
    write_private(os.path.join(d, 'llm.key'), bundle['llm_key'])
    open(os.path.join(d, 'cyclonedds.xml'), 'w').write(CYCLONE_XML)
    open(os.path.join(d, 'workspace.yaml'), 'w').write(yaml.safe_dump(
        {'name': ws, 'domain': bundle['domain'], 'peer': bundle['peer'], 'host': me}))
    write_private(os.path.join(d, 'env.sh'), env_text(
        {'domain': bundle['domain'], 'dir': d}, ws, me, bundle['peer'], os.path.join(d, 'llm.key')))
    return d


def cmd_join(a):
    me = host_label(a.host_name or socket.gethostname())
    if a.invite:
        inv = parse_invite(a.invite)
        ws, host, port, fp, token = inv['name'], inv['host'], inv['port'], inv['fp'], inv['token']
    else:
        ws = check_name(a.name)
        hits = [h for h in discover(a.timeout) if h['name'] == ws]
        if not hits:
            raise WsError(f'workspace {ws} not announced on this network (is `ws serve {ws}` '
                          f'running there? same subnet?) - or use --invite')
        host, port, fp = hits[0]['host'], hits[0]['port'], hits[0]['fp']
        token = a.token or getpass.getpass('workspace token: ')
    bundle = join_remote(host, port, token, me, fp)
    d = install_joined(ws, me, bundle)
    print(f'joined {ws} as {me} (domain {bundle["domain"]}); source {d}/env.sh')


def cmd_discover(a):
    hits = discover(a.timeout)
    if not hits:
        print('no workspace announced on this network')
    for h in hits:
        print(f"{h['name']:<20} {h['host']}:{h['port']}  cert {h['fp'][:12]}")


def cmd_list(_a):
    w, j = base_dirs()
    print('created here :', ', '.join(list_dir(w)) or '-')
    print('joined       :', ', '.join(list_dir(j)) or '-')


def cmd_env(a):
    w, j = base_dirs()
    for root in (w, j):
        p = os.path.join(root, check_name(a.name), 'env.sh')
        if os.path.exists(p):
            print(p)
            return
    raise WsError(f'no workspace {a.name}')


def cmd_members(a):
    wdir, _, _, _ = load_ws(a.name)
    for m in yaml.safe_load(open(os.path.join(wdir, 'members.yaml'))) or []:
        print(f"{m['host']:<24} {m['role']:<7} {m['joined']}")


def cmd_invite(a):
    _, meta, secret, fp = load_ws(a.name)
    print(make_invite(meta['name'], a.address or local_address(), meta['port'], fp, secret['token']))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('create'); c.add_argument('name'); c.add_argument('--domain', type=int)  # noqa: E702
    c.add_argument('--port', type=int, default=PORT); c.add_argument('--host-name'); c.add_argument('--address')  # noqa: E702
    c.set_defaults(fn=cmd_create)
    s = sub.add_parser('serve'); s.add_argument('name'); s.add_argument('--bind', default='0.0.0.0')  # noqa: E702
    s.add_argument('--announce-to', default='255.255.255.255'); s.add_argument('--address')  # noqa: E702
    s.set_defaults(fn=cmd_serve)
    d = sub.add_parser('discover'); d.add_argument('--timeout', type=float, default=4.0)  # noqa: E702
    d.set_defaults(fn=cmd_discover)
    j = sub.add_parser('join'); j.add_argument('name', nargs='?'); j.add_argument('--invite')  # noqa: E702
    j.add_argument('--token'); j.add_argument('--host-name'); j.add_argument('--timeout', type=float, default=5.0)  # noqa: E702
    j.set_defaults(fn=cmd_join)
    sub.add_parser('list').set_defaults(fn=cmd_list)
    for nm, fn in (('env', cmd_env), ('members', cmd_members), ('invite', cmd_invite)):
        p = sub.add_parser(nm); p.add_argument('name'); p.set_defaults(fn=fn)  # noqa: E702
        if nm == 'invite':
            p.add_argument('--address')
    a = ap.parse_args(argv)
    try:
        a.fn(a)
    except (WsError, ValueError, OSError, subprocess.SubprocessError, ssl.SSLError) as e:
        print(f'ws: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
