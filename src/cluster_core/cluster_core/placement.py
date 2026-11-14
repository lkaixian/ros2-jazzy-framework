"""Job placement: pure, deterministic, no ROS and no clock (time is passed in).

Hard constraints: pinned host / role, required tags, resource fit, host usable (alive and same
catalog), host not quarantined for this job. Soft: prefer_tags, spread by free capacity.
Policies: STICKINESS (a healthy job never moves because a "better" host appeared), failover only
for movable jobs on unusable hosts, immovable jobs never leave their host, overload rebalancing
with hysteresis + cooldown (one migration per call).
"""
from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Tuple


@dataclass(frozen=True)
class JobDef:
    id: str
    role: str = ''
    host: str = ''
    tags: FrozenSet[str] = frozenset()
    prefer_tags: FrozenSet[str] = frozenset()
    cpu: float = 0.5               # cores
    mem_mb: float = 256.0
    gpu_mem_mb: float = 0.0
    movable: bool = True
    priority: int = 0


@dataclass
class HostInfo:
    id: str
    roles: FrozenSet[str] = frozenset()
    tags: FrozenSet[str] = frozenset()
    cores: int = 1
    cpu_load: float = 0.0          # observed, 0..1
    mem_total_mb: float = 0.0
    mem_free_mb: float = 0.0       # observed
    gpu_mem_free_mb: float = -1.0  # observed; < 0 means no GPU
    alive: bool = True
    catalog_ok: bool = True
    failed_jobs: FrozenSet[str] = frozenset()   # jobs that reported FAILED on this host


@dataclass
class Config:
    overload: float = 0.85         # host counts as overloaded above this cpu_load
    underload: float = 0.6         # migration targets must be below this
    hysteresis_sec: float = 10.0   # overload must persist this long before migrating
    cooldown_sec: float = 60.0     # minimum time between migrations of the same job
    quarantine_sec: float = 60.0   # how long a host is avoided for a job that FAILED there


@dataclass
class State:
    """Memory the planner needs between calls (owned by the caller)."""
    over_since: Dict[str, float] = field(default_factory=dict)
    last_migrated: Dict[str, float] = field(default_factory=dict)
    quarantine: Dict[Tuple[str, str], float] = field(default_factory=dict)


@dataclass
class Placement:
    state: str                     # placed | pending | unplaceable
    host: str = ''
    reason: str = ''


@dataclass
class Plan:
    assignment: Dict[str, str]
    placements: Dict[str, Placement]
    events: List[str]


def _eligible(job, h):
    """Static constraints only (pins and tags)."""
    return ((not job.host or job.host == h.id)
            and (not job.role or job.role in h.roles)
            and job.tags <= h.tags)


class _Budget:
    def __init__(self, h):
        self.cpu = max(0.0, h.cores * (1.0 - h.cpu_load))
        self.mem = h.mem_free_mb
        self.gpu = max(0.0, h.gpu_mem_free_mb)

    def fits(self, j):
        return self.cpu + 1e-9 >= j.cpu and self.mem + 1e-9 >= j.mem_mb and self.gpu + 1e-9 >= j.gpu_mem_mb

    def take(self, j):
        self.cpu -= j.cpu
        self.mem -= j.mem_mb
        self.gpu -= j.gpu_mem_mb


def _score(j, h, b):
    s = (b.cpu - j.cpu) / max(1, h.cores) + 0.5 * (b.mem - j.mem_mb) / max(1.0, h.mem_total_mb)
    if j.prefer_tags:
        s += 0.3 * len(j.prefer_tags & h.tags) / len(j.prefer_tags)
    return s


def _best(j, candidates, budgets):
    return min(candidates, key=lambda h: (-_score(j, h, budgets[h.id]), h.id))


def plan(jobs, hosts, current, now, state, cfg=None, unsettled=frozenset()):
    """Compute the desired placement.

    jobs: iterable[JobDef]; hosts: iterable[HostInfo]; current: {job_id: host_id} (what is
    assigned now); unsettled: ids of jobs assigned but not yet running (their cost is reserved
    because the host's observed load does not contain them yet).
    """
    cfg = cfg or Config()
    jobs = sorted(jobs, key=lambda j: (-j.priority, j.id))
    by_id = {h.id: h for h in hosts}
    events: List[str] = []

    # quarantine: a host that reports FAILED for a job is avoided for that job for a while
    for h in by_id.values():
        for jid in sorted(h.failed_jobs):
            if (jid, h.id) not in state.quarantine or state.quarantine[(jid, h.id)] < now:
                events.append(f'quarantine {jid} on {h.id} for {cfg.quarantine_sec:.0f}s (FAILED)')
            state.quarantine[(jid, h.id)] = now + cfg.quarantine_sec
    state.quarantine = {k: t for k, t in state.quarantine.items() if t >= now}

    usable = {h.id: h for h in by_id.values() if h.alive and h.catalog_ok}
    budgets = {hid: _Budget(h) for hid, h in usable.items()}
    job_by_id = {j.id: j for j in jobs}
    assignment: Dict[str, str] = {}
    placements: Dict[str, Placement] = {}

    def host_ok(j, h):
        return _eligible(j, h) and (j.id, h.id) not in state.quarantine

    def why_not(j):
        pool = list(by_id.values())
        if j.host and j.host not in by_id:
            return f'pinned host {j.host} unknown'
        if not any(_eligible(j, h) for h in pool):
            return 'no known host satisfies ' + _constraint_text(j)
        if not any(_eligible(j, h) for h in usable.values()):
            down = sorted(h.id for h in pool if _eligible(j, h))
            return 'eligible host(s) down or excluded: ' + ', '.join(down)
        return ''

    for j in jobs:
        cur = current.get(j.id)
        ch = by_id.get(cur) if cur else None

        # 1. stay where we are (stickiness / immovability)
        if ch is not None and ch.id in usable and host_ok(j, ch):
            assignment[j.id] = ch.id
            placements[j.id] = Placement('placed', ch.id)
            if j.id in unsettled:
                budgets[ch.id].take(j)               # not in the observed load yet
            continue
        if cur and not j.movable:
            # hardware-bound: keep the desired state on its host, never fall back elsewhere
            assignment[j.id] = cur
            if ch is None or ch.id not in usable:
                state_txt = 'unknown' if ch is None else ('excluded (catalog mismatch)' if ch.alive else 'down')
                placements[j.id] = Placement('unplaceable', cur,
                                             f'immovable job pinned to {cur}, which is {state_txt}')
            else:
                placements[j.id] = Placement('placed', cur, 'immovable; quarantined on its only host')
            continue

        # 2. fresh placement (first time, or failover of a movable job)
        cands = [h for h in usable.values()
                 if host_ok(j, h) and budgets[h.id].fits(j) and h.cpu_load < cfg.overload]
        if cands:
            best = _best(j, cands, budgets)
            budgets[best.id].take(j)
            assignment[j.id] = best.id
            placements[j.id] = Placement('placed', best.id,
                                         f'failover from {cur}' if cur and cur != best.id else '')
            if cur and cur != best.id:
                events.append(f'failover {j.id}: {cur} -> {best.id}')
            continue
        reason = why_not(j)
        if reason:
            placements[j.id] = Placement('unplaceable', '', reason)
        else:
            quarantined = sorted(h.id for h in usable.values()
                                 if _eligible(j, h) and (j.id, h.id) in state.quarantine)
            if quarantined and len(quarantined) == sum(1 for h in usable.values() if _eligible(j, h)):
                placements[j.id] = Placement('unplaceable', '',
                                             'quarantined on: ' + ', '.join(quarantined))
            else:
                placements[j.id] = Placement('pending', '', 'no capacity on eligible hosts')

    _rebalance(job_by_id, usable, budgets, assignment, placements, unsettled, now, state, cfg,
               events, host_ok)
    return Plan(assignment, placements, events)


def _constraint_text(j):
    bits = []
    if j.host:
        bits.append(f'host={j.host}')
    if j.role:
        bits.append(f'role={j.role}')
    if j.tags:
        bits.append('tags=' + ','.join(sorted(j.tags)))
    return ' '.join(bits) or 'constraints'


def _rebalance(job_by_id, usable, budgets, assignment, placements, unsettled, now, state, cfg,
               events, host_ok):
    for hid, h in sorted(usable.items()):
        if h.cpu_load > cfg.overload:
            state.over_since.setdefault(hid, now)
        else:
            state.over_since.pop(hid, None)
    state.over_since = {k: v for k, v in state.over_since.items() if k in usable}

    over = [hid for hid, t in state.over_since.items() if now - t >= cfg.hysteresis_sec]
    over.sort(key=lambda hid: (-usable[hid].cpu_load, hid))
    for hid in over:
        movers = [job_by_id[jid] for jid, h in assignment.items()
                  if h == hid and jid in job_by_id and job_by_id[jid].movable
                  and jid not in unsettled
                  and now - state.last_migrated.get(jid, float('-inf')) >= cfg.cooldown_sec]
        movers.sort(key=lambda j: (-j.cpu, j.id))
        for j in movers:
            cands = [t for t in usable.values()
                     if t.id != hid and host_ok(j, t) and t.cpu_load < cfg.underload
                     and budgets[t.id].fits(j)]
            if not cands:
                continue
            best = _best(j, cands, budgets)
            budgets[best.id].take(j)
            assignment[j.id] = best.id
            placements[j.id] = Placement('placed', best.id, f'migrated from overloaded {hid}')
            state.last_migrated[j.id] = now
            state.over_since[hid] = now              # restart the hysteresis window
            events.append(f'migrate {j.id}: {hid} -> {best.id} (load {usable[hid].cpu_load:.2f})')
            return                                   # one migration per call


def adopt(jobs_ids, reported):
    """Initial `current` after a scheduler (re)start, from heartbeats.

    reported: {host_id: iterable[job_id]} of jobs the hosts actually run. If several hosts run
    the same job the lowest host id is kept (the others get told to stop by the next set).
    """
    known = set(jobs_ids)
    out: Dict[str, str] = {}
    for hid in sorted(reported):
        for jid in reported[hid]:
            if jid in known and jid not in out:
                out[jid] = hid
    return out
