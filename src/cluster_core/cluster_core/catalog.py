"""Local job catalog: the ONLY source of commands an agent will ever run.

Assignments received over DDS carry job ids, never commands. Commands are argv lists (no shell).
The catalog hash covers everything that affects scheduling or execution, so a scheduler and an
agent with different catalogs refuse to cooperate.
"""
import hashlib
import json
import re

import yaml

ID_RE = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')
VAR_RE = re.compile(r'\$\{([A-Za-z0-9_]+)\}')


class CatalogError(ValueError):
    pass


DEFAULT_RESTART = {'max_restarts': 5, 'window_sec': 60.0, 'backoff_base': 1.0, 'backoff_max': 30.0}


def _tags(v, what):
    if v is None or v == '':
        return []
    if isinstance(v, str):
        raise CatalogError(f'{what} must be a list, not a string: {v!r}')
    if not isinstance(v, (list, tuple)) or not all(isinstance(x, str) and x for x in v):
        raise CatalogError(f'{what} must be a list of non-empty strings')
    return sorted(set(v))


def _argv(cmd, jid):
    if isinstance(cmd, str):
        raise CatalogError(f"job {jid}: command must be an argv list, not a shell string: {cmd!r}")
    if (not isinstance(cmd, (list, tuple)) or not cmd
            or not all(isinstance(a, (str, int, float)) and not isinstance(a, bool) for a in cmd)):
        raise CatalogError(f'job {jid}: command must be a non-empty list of strings')
    return [str(a) for a in cmd]


def _num(spec, key, default, jid, lo=0.0):
    v = spec.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v < lo:
        raise CatalogError(f'job {jid}: {key} must be a number >= {lo}, got {v!r}')
    return float(v)


def parse_job(jid, spec):
    if not ID_RE.match(jid):
        raise CatalogError(f'invalid job id {jid!r} (use A-Z a-z 0-9 _ . -, max 64)')
    if not isinstance(spec, dict):
        raise CatalogError(f'job {jid}: definition must be a mapping')
    if 'cmd' in spec and 'cmds' in spec:
        raise CatalogError(f"job {jid}: give either 'cmd' or 'cmds', not both")
    raw = [spec['cmd']] if 'cmd' in spec else spec.get('cmds')
    if not raw:
        raise CatalogError(f"job {jid}: needs 'cmd' (one argv list) or 'cmds' (several)")
    if isinstance(raw, str):
        raise CatalogError(f"job {jid}: 'cmds' must be a list of argv lists")
    cmds = [_argv(c, jid) for c in raw]
    role = spec.get('role', '') or ''
    host = spec.get('host', '') or ''
    if not isinstance(role, str) or not isinstance(host, str):
        raise CatalogError(f'job {jid}: role and host must be strings')
    movable = spec.get('movable', True)
    if not isinstance(movable, bool):
        raise CatalogError(f'job {jid}: movable must be true/false')
    restart = dict(DEFAULT_RESTART)
    restart.update(spec.get('restart') or {})
    for k in restart:
        if k not in DEFAULT_RESTART:
            raise CatalogError(f'job {jid}: unknown restart key {k!r}')
    endpoint = spec.get('endpoint')
    if endpoint is not None:
        if (not isinstance(endpoint, dict) or set(endpoint) - {'kind', 'port', 'priority'}
                or not isinstance(endpoint.get('kind'), str) or not endpoint['kind']
                or isinstance(endpoint.get('port'), bool)
                or not isinstance(endpoint.get('port'), int) or not 1 <= endpoint['port'] <= 65535
                or isinstance(endpoint.get('priority', 0), bool)
                or not isinstance(endpoint.get('priority', 0), int)):
            raise CatalogError(f"job {jid}: endpoint must be {{kind: str, port: 1-65535, "
                               f"priority: int (optional)}}, got {endpoint!r}")
        endpoint = {'kind': endpoint['kind'], 'port': endpoint['port'],
                    'priority': endpoint.get('priority', 0)}
    env = spec.get('env') or {}
    if not isinstance(env, dict) or not all(isinstance(k, str) for k in env):
        raise CatalogError(f'job {jid}: env must be a mapping')
    out = {
        'id': jid,
        'cmds': cmds,
        'role': role,
        'host': host,
        'tags': _tags(spec.get('tags'), f'job {jid}: tags'),
        'prefer_tags': _tags(spec.get('prefer_tags'), f'job {jid}: prefer_tags'),
        'cpu': _num(spec, 'cpu', 0.5, jid),
        'mem_mb': _num(spec, 'mem_mb', 256.0, jid),
        'gpu_mem_mb': _num(spec, 'gpu_mem_mb', 0.0, jid),
        'movable': movable,
        'priority': int(_num(spec, 'priority', 0, jid, lo=-1000)),
        'stop_on_orphan': spec.get('stop_on_orphan', movable),
        'grace_sec': _num(spec, 'grace_sec', 5.0, jid),
        'restart': {k: float(v) for k, v in restart.items()},
        'env': {k: str(v) for k, v in env.items()},
        'endpoint': endpoint,
    }
    if not isinstance(out['stop_on_orphan'], bool):
        raise CatalogError(f'job {jid}: stop_on_orphan must be true/false')
    return out


class Catalog:
    def __init__(self, jobs):
        self.jobs = jobs                  # id -> normalized dict
        blob = json.dumps(jobs, sort_keys=True, separators=(',', ':')).encode()
        self.hash = hashlib.sha256(blob).hexdigest()[:16]

    def __contains__(self, jid):
        return jid in self.jobs

    def __getitem__(self, jid):
        return self.jobs[jid]

    def ids(self):
        return sorted(self.jobs)


def load_catalog(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return parse_catalog(data)


def parse_catalog(data):
    raw = data.get('jobs')
    if not isinstance(raw, dict) or not raw:
        raise CatalogError("catalog needs a non-empty 'jobs' mapping")
    return Catalog({jid: parse_job(str(jid), spec) for jid, spec in raw.items()})


def jobdef_from_spec(spec):
    """Scheduling view of a normalized catalog entry."""
    from cluster_core.placement import JobDef
    return JobDef(id=spec['id'], role=spec['role'], host=spec['host'],
                  tags=frozenset(spec['tags']), prefer_tags=frozenset(spec['prefer_tags']),
                  cpu=spec['cpu'], mem_mb=spec['mem_mb'], gpu_mem_mb=spec['gpu_mem_mb'],
                  movable=spec['movable'], priority=spec['priority'])


def expand(argv, variables):
    """Replace ${NAME} using ONLY the given variables; an unknown name is an error."""
    def sub(m):
        if m.group(1) not in variables:
            raise CatalogError(f'unknown variable ${{{m.group(1)}}} in {argv!r}')
        return variables[m.group(1)]
    return [VAR_RE.sub(sub, a) for a in argv]
