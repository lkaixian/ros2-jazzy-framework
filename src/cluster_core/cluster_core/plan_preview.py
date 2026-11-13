"""plan_preview: compute a placement offline from a hosts file and a jobs catalog.

    ros2 run cluster_core plan_preview --hosts hosts.yaml --jobs cluster_jobs.yaml [--dead pi]
                                       [--current current.yaml]

hosts.yaml:
    hosts:
      pi:     {roles: core,    tags: camera,      cores: 4,  cpu_load: 0.1, mem_total_mb: 4096, mem_free_mb: 3000}
      laptop: {roles: compute, tags: "gpu,audio", cores: 12, cpu_load: 0.2, mem_total_mb: 16000, mem_free_mb: 9000, gpu_mem_free_mb: 3500}
current.yaml (optional): {job_id: host_id, ...}   what is assigned now (shows stickiness/failover)
Prints the placement, a per-host load summary and whether a second pass changes anything.
"""
import argparse
import sys

import yaml

from cluster_core import placement
from cluster_core.catalog import jobdef_from_spec, load_catalog


def _set(v):
    if isinstance(v, str):
        return frozenset(t.strip() for t in v.split(',') if t.strip())
    return frozenset(v or [])


def load_hosts(path, dead=()):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    hosts = []
    for hid, h in (data.get('hosts') or {}).items():
        h = h or {}
        hosts.append(placement.HostInfo(
            id=hid, roles=_set(h.get('roles')), tags=_set(h.get('tags')), cores=int(h.get('cores', 1)),
            cpu_load=float(h.get('cpu_load', 0.0)), mem_total_mb=float(h.get('mem_total_mb', 0.0)),
            mem_free_mb=float(h.get('mem_free_mb', h.get('mem_total_mb', 0.0))),
            gpu_mem_free_mb=float(h.get('gpu_mem_free_mb', -1.0)), alive=hid not in set(dead),
            catalog_ok=bool(h.get('catalog_ok', True))))
    if not hosts:
        raise SystemExit(f"{path}: needs a non-empty 'hosts' mapping")
    return hosts


def run(hosts, catalog, current=None, now=0.0):
    jobs = [jobdef_from_spec(catalog[j]) for j in catalog.ids()]
    state = placement.State()
    first = placement.plan(jobs, hosts, current or {}, now, state)
    # Second pass = the next scheduler cycle. The hosts' load numbers here do not contain the
    # jobs of the first pass yet (nothing is running), so their cost stays reserved, exactly as
    # the live scheduler does for jobs that were assigned but have not started.
    second = placement.plan(jobs, hosts, first.assignment, now, state,
                            unsettled=frozenset(first.assignment))
    return jobs, first, second


def render(jobs, hosts, first, second):
    lines = [f'{"JOB":<22}{"HOST":<12}{"STATE":<12}REASON']
    for j in jobs:
        p = first.placements[j.id]
        lines.append(f'{j.id:<22}{p.host or "-":<12}{p.state:<12}{p.reason}')
    lines += ['', f'{"HOST":<12}{"STATE":<8}{"CPU ADDED":<12}{"MEM ADDED":<12}JOBS']
    for h in hosts:
        mine = [j for j in jobs if first.assignment.get(j.id) == h.id]
        lines.append(f'{h.id:<12}{"up" if h.alive else "DOWN":<8}{sum(j.cpu for j in mine):<12.2f}'
                     f'{sum(j.mem_mb for j in mine):<12.0f}{", ".join(j.id for j in mine) or "-"}')
    lines += ['', 'events: ' + ('; '.join(first.events) or 'none'),
              'second pass (steady state): ' + ('identical (stable)' if second.assignment == first.assignment
                                                else 'DIFFERS: ' + str(second.assignment))]
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--hosts', required=True)
    ap.add_argument('--jobs', required=True)
    ap.add_argument('--dead', default='', help='comma-separated host ids to treat as down')
    ap.add_argument('--current', help='YAML {job: host}: the placement in force before')
    a = ap.parse_args(argv)
    catalog = load_catalog(a.jobs)
    hosts = load_hosts(a.hosts, [d for d in a.dead.split(',') if d])
    current = {}
    if a.current:
        with open(a.current) as f:
            current = yaml.safe_load(f) or {}
    jobs, first, second = run(hosts, catalog, current)
    print(render(jobs, hosts, first, second))
    bad = [j for j, p in first.placements.items() if p.state != 'placed']
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
