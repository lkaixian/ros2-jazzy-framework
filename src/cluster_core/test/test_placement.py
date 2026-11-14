import random

import pytest

from cluster_core.placement import Config, HostInfo, JobDef, State, adopt, plan


def H(hid, roles='', tags='', cores=4, load=0.1, mem=8000.0, free=6000.0, gpu=-1.0, alive=True,
      ok=True, failed=()):
    return HostInfo(hid, frozenset(r for r in roles.split(',') if r), frozenset(t for t in tags.split(',') if t),
                    cores, load, mem, free, gpu, alive, ok, frozenset(failed))


def J(jid, role='', host='', tags='', prefer='', cpu=0.5, mem=256.0, gpu=0.0, movable=True, prio=0):
    return JobDef(jid, role, host, frozenset(t for t in tags.split(',') if t),
                  frozenset(t for t in prefer.split(',') if t), cpu, mem, gpu, movable, prio)


def run(jobs, hosts, current=None, now=0.0, state=None, cfg=None, unsettled=()):
    return plan(jobs, hosts, current or {}, now, state or State(), cfg, frozenset(unsettled))


# ------------------------------------------------------------------ constraints
def test_role_tags_and_host_pins_are_hard():
    hosts = [H('pi', 'core', 'camera'), H('lap', 'compute', 'gpu,audio')]
    jobs = [J('odom', role='core', movable=False), J('det', tags='camera'), J('tts', tags='audio'),
            J('pinned', host='lap'), J('free')]
    p = run(jobs, hosts)
    assert p.assignment['odom'] == 'pi' and p.assignment['det'] == 'pi'
    assert p.assignment['tts'] == 'lap' and p.assignment['pinned'] == 'lap'
    assert set(p.assignment) == {j.id for j in jobs}
    assert all(v.state == 'placed' for v in p.placements.values())


def test_tags_must_all_be_present():
    p = run([J('both', tags='camera,gpu')], [H('a', tags='camera'), H('b', tags='gpu')])
    assert 'both' not in p.assignment
    assert p.placements['both'].state == 'unplaceable' and 'tags=camera,gpu' in p.placements['both'].reason


def test_unplaceable_reasons():
    p = run([J('x', role='core'), J('y', host='ghost')], [H('lap', 'compute')])
    assert 'no known host satisfies role=core' in p.placements['x'].reason
    assert 'pinned host ghost unknown' in p.placements['y'].reason
    p = run([J('x', role='core')], [H('pi', 'core', alive=False)])
    assert p.placements['x'].state == 'unplaceable' and 'down or excluded: pi' in p.placements['x'].reason


# -------------------------------------------------------------------- resources
def test_cpu_is_never_overcommitted_in_one_plan():
    hosts = [H('a', cores=2, load=0.0), H('b', cores=2, load=0.0)]
    jobs = [J(f'j{i}', cpu=1.5) for i in range(3)]
    p = run(jobs, hosts)
    assert len(p.assignment) == 2 and sorted(p.assignment.values()) == ['a', 'b']
    pend = [v for v in p.placements.values() if v.state == 'pending']
    assert len(pend) == 1 and 'no capacity' in pend[0].reason


def test_observed_load_reduces_capacity():
    p = run([J('j', cpu=2.0)], [H('busy', cores=4, load=0.6), H('idle', cores=4, load=0.0)])
    assert p.assignment['j'] == 'idle'                          # busy has only 1.6 cores free


def test_memory_and_gpu_memory_fit():
    hosts = [H('small', free=500.0, gpu=-1.0), H('gpu', free=9000.0, gpu=3000.0)]
    p = run([J('big', mem=4000.0), J('needs_gpu', gpu=2500.0, mem=100.0)], hosts)
    assert p.assignment == {'big': 'gpu', 'needs_gpu': 'gpu'}
    p = run([J('g', gpu=1.0)], [H('nogpu', gpu=-1.0)])
    assert p.placements['g'].state == 'pending'                  # a GPU-less host has 0 GPU MB


def test_overloaded_host_gets_no_new_jobs():
    p = run([J('j', cpu=0.1)], [H('hot', cores=64, load=0.9), H('ok', cores=2, load=0.5)])
    assert p.assignment['j'] == 'ok'


def test_priority_wins_scarce_capacity():
    hosts = [H('a', cores=2, load=0.0)]
    p = run([J('low', cpu=1.5, prio=0), J('high', cpu=1.5, prio=10)], hosts)
    assert p.assignment == {'high': 'a'} and p.placements['low'].state == 'pending'


def test_unsettled_jobs_reserve_capacity():
    hosts = [H('a', cores=2, load=0.0), H('b', cores=2, load=0.0)]
    jobs = [J('first', cpu=1.5, prio=5), J('second', cpu=1.5)]
    p = run(jobs, hosts, current={'first': 'a'}, unsettled={'first'})
    assert p.assignment == {'first': 'a', 'second': 'b'}         # 'a' still has 1.5 reserved
    p = run(jobs, hosts, current={'first': 'a'})                  # once settled it is in the load
    assert p.assignment['second'] in ('a', 'b')                   # no reservation any more


def test_prefer_tags_and_spreading():
    p = run([J('llm', prefer='gpu')], [H('cpu_only'), H('gpu_box', tags='gpu')])
    assert p.assignment['llm'] == 'gpu_box'
    p = run([J('x', cpu=2.0), J('y', cpu=2.0)], [H('a', cores=8, load=0.0), H('b', cores=8, load=0.0)])
    assert sorted(p.assignment.values()) == ['a', 'b']            # spread, not stacked


# ----------------------------------------------------- determinism / stickiness
def test_deterministic_and_order_independent():
    hosts = [H('a', tags='gpu'), H('b'), H('c', 'core')]
    jobs = [J(f'j{i}', cpu=0.3 + 0.1 * (i % 3), prefer='gpu' if i % 2 else '') for i in range(9)]
    base = run(jobs, hosts).assignment
    for seed in range(5):
        r = random.Random(seed)
        j2, h2 = jobs[:], hosts[:]
        r.shuffle(j2)
        r.shuffle(h2)
        assert run(j2, h2).assignment == base


def test_idempotent_steady_state():
    hosts = [H('a', tags='gpu'), H('b'), H('c', 'core')]
    jobs = [J('c1', role='core', movable=False), J('g', prefer='gpu'), J('x'), J('y')]
    first = run(jobs, hosts)
    again = run(jobs, hosts, current=first.assignment)
    assert again.assignment == first.assignment and not again.events


def test_stickiness_better_host_appearing_does_not_move_a_healthy_job():
    jobs = [J('llm', prefer='gpu')]
    p = run(jobs, [H('old')], current={})
    assert p.assignment['llm'] == 'old'
    p = run(jobs, [H('old'), H('new_gpu', tags='gpu', cores=32, load=0.0)], current=p.assignment)
    assert p.assignment['llm'] == 'old'


# ---------------------------------------------------------------------- failover
def test_movable_job_fails_over_when_host_is_down():
    jobs = [J('slm')]
    p = run(jobs, [H('a', alive=False), H('b')], current={'slm': 'a'})
    assert p.assignment['slm'] == 'b'
    assert p.events == ['failover slm: a -> b'] and 'failover from a' in p.placements['slm'].reason


def test_failover_only_to_eligible_hosts():
    p = run([J('det', tags='camera')], [H('a', tags='camera', alive=False), H('b')], current={'det': 'a'})
    assert 'det' not in p.assignment and p.placements['det'].state == 'unplaceable'


def test_immovable_job_on_dead_host_is_unplaceable_not_failed_over():
    hosts = [H('pi', 'core', alive=False), H('pi2', 'core')]
    p = run([J('odom', role='core', movable=False)], hosts, current={'odom': 'pi'})
    assert p.assignment == {'odom': 'pi'}                         # desired state stays on its hardware
    assert p.placements['odom'].state == 'unplaceable' and 'pinned to pi' in p.placements['odom'].reason
    # the host returns: it is simply placed again, nothing was ever moved
    p = run([J('odom', role='core', movable=False)], [H('pi', 'core'), H('pi2', 'core')],
            current={'odom': 'pi'})
    assert p.assignment == {'odom': 'pi'} and p.placements['odom'].state == 'placed'


def test_immovable_job_first_placement_picks_a_role_host_and_never_a_stranger():
    p = run([J('odom', role='core', movable=False)], [H('lap', 'compute')])
    assert 'odom' not in p.assignment and p.placements['odom'].state == 'unplaceable'


# --------------------------------------------------------------- catalog mismatch
def test_hosts_with_a_different_catalog_are_excluded():
    p = run([J('new')], [H('bad', ok=False, cores=64, load=0.0), H('good')])
    assert p.assignment['new'] == 'good'
    p = run([J('j')], [H('bad', ok=False), H('good')], current={'j': 'bad'})
    assert p.assignment['j'] == 'good'                            # moved off the excluded host
    p = run([J('only')], [H('bad', ok=False)])
    assert p.placements['only'].state == 'unplaceable'


# --------------------------------------------------------------------- quarantine
def test_quarantine_after_failed_report_then_expires():
    st = State()
    cfg = Config(quarantine_sec=60.0)
    jobs = [J('det')]
    hosts = [H('a', failed=['det']), H('b')]
    p = run(jobs, hosts, current={'det': 'a'}, now=0.0, state=st, cfg=cfg)
    assert p.assignment['det'] == 'b' and any('quarantine det on a' in e for e in p.events)
    # a cleared the failure; still quarantined for 60 s: a new job goes to b, not a
    p = run([J('det'), J('other', cpu=0.1)], [H('a'), H('b', load=0.5)], current={}, now=30.0, state=st, cfg=cfg)
    assert p.assignment['det'] == 'b'
    p = run([J('det')], [H('a'), H('b', load=0.5)], current={}, now=61.0, state=st, cfg=cfg)
    assert p.assignment['det'] == 'a'                             # allowed again (a is emptier)


def test_quarantined_on_every_eligible_host_is_unplaceable_until_it_expires():
    st = State()
    p = run([J('det', tags='camera')], [H('a', tags='camera', failed=['det'])], current={'det': 'a'},
            now=0.0, state=st)
    assert 'det' not in p.assignment                              # agent stops the FAILED job
    assert p.placements['det'].state == 'unplaceable' and 'quarantined on: a' in p.placements['det'].reason
    p = run([J('det', tags='camera')], [H('a', tags='camera')], current={}, now=61.0, state=st)
    assert p.assignment == {'det': 'a'}                           # automatic retry later


# ---------------------------------------------------------------------- rebalance
def hot_cluster(load_a=0.95):
    jobs = [J('big', cpu=2.0), J('small', cpu=0.5), J('hw', cpu=3.0, movable=False)]
    return jobs, [H('a', cores=8, load=load_a), H('b', cores=8, load=0.2)], {'big': 'a', 'small': 'a', 'hw': 'a'}


def test_no_migration_before_hysteresis_then_exactly_one():
    jobs, hosts, cur = hot_cluster()
    st, cfg = State(), Config(hysteresis_sec=10.0, cooldown_sec=60.0)
    for t in (0.0, 5.0, 9.9):
        assert run(jobs, hosts, cur, t, st, cfg).assignment == cur
    p = run(jobs, hosts, cur, 10.0, st, cfg)
    assert p.assignment['big'] == 'b' and p.assignment['small'] == 'a'       # one job, the biggest
    assert p.assignment['hw'] == 'a'                                         # immovable stays
    assert len(p.events) == 1 and p.events[0].startswith('migrate big: a -> b')


def test_hysteresis_window_restarts_and_cooldown_blocks_pingpong():
    jobs, hosts, cur = hot_cluster()
    st, cfg = State(), Config(hysteresis_sec=10.0, cooldown_sec=60.0)
    run(jobs, hosts, cur, 0.0, st, cfg)
    p = run(jobs, hosts, cur, 10.0, st, cfg)
    cur = p.assignment
    assert run(jobs, hosts, cur, 10.5, st, cfg).assignment == cur            # window restarted
    p = run(jobs, hosts, cur, 20.0, st, cfg)
    assert p.assignment['small'] == 'b'                                      # next job, new window
    cur = p.assignment
    # now b is the overloaded one: 'big' (migrated at t=10) may not move back before t=70
    flipped = [H('a', cores=8, load=0.1), H('b', cores=8, load=0.95)]
    for t in (21.0, 32.0, 45.0, 69.0):
        got = run(jobs, flipped, cur, t, st, cfg).assignment
        assert got['big'] == 'b'
    assert run(jobs, flipped, cur, 71.0, st, cfg).assignment['big'] == 'a'


def test_overload_that_ends_resets_the_timer():
    jobs, hosts, cur = hot_cluster()
    st, cfg = State(), Config(hysteresis_sec=10.0)
    run(jobs, hosts, cur, 0.0, st, cfg)
    run(jobs, [H('a', cores=8, load=0.3), hosts[1]], cur, 8.0, st, cfg)       # calmed down
    assert run(jobs, hosts, cur, 12.0, st, cfg).assignment == cur             # timer restarted at 12
    assert run(jobs, hosts, cur, 22.0, st, cfg).assignment['big'] == 'b'


def test_no_migration_without_an_underloaded_target():
    jobs, _, cur = hot_cluster()
    hosts = [H('a', cores=8, load=0.95), H('b', cores=8, load=0.7)]           # b not below 0.6
    st, cfg = State(), Config(hysteresis_sec=0.0)
    assert run(jobs, hosts, cur, 0.0, st, cfg).assignment == cur
    assert run(jobs, hosts, cur, 5.0, st, cfg).assignment == cur


def test_migration_respects_target_constraints():
    jobs = [J('det', cpu=2.0, tags='camera')]
    hosts = [H('a', tags='camera', cores=8, load=0.95), H('b', cores=8, load=0.1)]
    st, cfg = State(), Config(hysteresis_sec=0.0)
    run(jobs, hosts, {'det': 'a'}, 0.0, st, cfg)
    assert run(jobs, hosts, {'det': 'a'}, 1.0, st, cfg).assignment == {'det': 'a'}   # b lacks the camera


# --------------------------------------------------------------------------- adopt
def test_adopt_from_heartbeats():
    got = adopt(['a', 'b', 'c'], {'h2': ['a', 'b', 'zzz'], 'h1': ['b'], 'h3': []})
    assert got == {'a': 'h2', 'b': 'h1'}                          # lowest id wins, unknown ignored
    assert adopt(['a'], {}) == {}


# ---------------------------------------------------------- randomized invariants
def random_cluster(r):
    hosts = [H(f'h{i}', ','.join(r.sample(['core', 'compute', 'edge'], r.randint(0, 2))),
               ','.join(r.sample(['camera', 'audio', 'gpu'], r.randint(0, 3))),
               cores=r.choice([1, 2, 4, 8, 12]), load=round(r.random(), 2),
               mem=16000.0, free=float(r.randint(0, 16000)), gpu=r.choice([-1.0, 0.0, 1500.0, 4000.0]),
               alive=r.random() > 0.2, ok=r.random() > 0.1,
               failed=[f'j{k}' for k in range(12) if r.random() < 0.05])
             for i in range(r.randint(2, 5))]
    jobs = [J(f'j{i}', role=r.choice(['', '', 'core', 'compute']), host=r.choice(['', '', '', 'h0', 'h9']),
              tags=','.join(r.sample(['camera', 'audio', 'gpu'], r.randint(0, 2))),
              prefer=r.choice(['', 'gpu']), cpu=round(r.uniform(0.05, 3.0), 2),
              mem=float(r.randint(10, 6000)), gpu=r.choice([0.0, 0.0, 500.0, 3000.0]),
              movable=r.random() > 0.3, prio=r.randint(0, 3)) for i in range(r.randint(3, 12))]
    current = {j.id: r.choice([h.id for h in hosts]) for j in jobs if r.random() < 0.6}
    unsettled = {j for j in current if r.random() < 0.3}
    return jobs, hosts, current, unsettled


def static_ok(job, h):
    return ((not job.host or job.host == h.id) and (not job.role or job.role in h.roles)
            and job.tags <= h.tags)


@pytest.mark.parametrize('seed', range(400))
def test_random_clusters_keep_every_invariant(seed):
    r = random.Random(seed)
    jobs, hosts, current, unsettled = random_cluster(r)
    cfg = Config()
    state = State()
    p = plan(jobs, hosts, current, 100.0, state, cfg, frozenset(unsettled))
    by_job, by_host = {j.id: j for j in jobs}, {h.id: h for h in hosts}
    usable = {h.id for h in hosts if h.alive and h.catalog_ok}

    assert set(p.assignment) <= set(by_job)                     # never invents a job
    assert set(p.placements) == set(by_job)                     # every job is accounted for
    new = {h: [0.0, 0.0, 0.0] for h in by_host}
    for jid, hid in p.assignment.items():
        j, h = by_job[jid], by_host[hid]
        stays = current.get(jid) == hid
        if not j.movable and jid in current:
            assert hid == current[jid]                           # immovable jobs never leave
            continue
        if stays:
            if p.placements[jid].state == 'placed':
                assert hid in usable and static_ok(j, h) and (jid, hid) not in state.quarantine
            continue
        assert p.placements[jid].state == 'placed'
        assert hid in usable and static_ok(j, h), 'new placement on an ineligible/unusable host'
        assert (jid, hid) not in state.quarantine
        assert h.cpu_load < cfg.overload
        new[hid][0] += j.cpu
        new[hid][1] += j.mem_mb
        new[hid][2] += j.gpu_mem_mb
    for hid, (cpu, mem, gpu) in new.items():          # no overcommit by newly placed jobs
        h = by_host[hid]
        assert cpu <= max(0.0, h.cores * (1 - h.cpu_load)) + 1e-6
        assert mem <= h.mem_free_mb + 1e-6
        assert gpu <= max(0.0, h.gpu_mem_free_mb) + 1e-6


@pytest.mark.parametrize('seed', range(150))
def test_random_clusters_are_deterministic_and_idempotent(seed):
    r = random.Random(1000 + seed)
    jobs, hosts, current, unsettled = random_cluster(r)
    a = plan(jobs, hosts, current, 100.0, State(), Config(), frozenset(unsettled)).assignment
    j2, h2 = jobs[:], hosts[:]
    r.shuffle(j2)
    r.shuffle(h2)
    assert plan(j2, h2, current, 100.0, State(), Config(), frozenset(unsettled)).assignment == a
    # re-planning with the result as `current` never moves a job that sits on a healthy,
    # eligible host (stickiness); jobs that were pending may only be ADDED, never shuffled
    again = plan(jobs, hosts, a, 100.0, State(), Config(), frozenset(unsettled)).assignment
    quarantined = {(f, hh.id) for hh in hosts for f in hh.failed_jobs}
    for jid, hid in a.items():
        j = next(x for x in jobs if x.id == jid)
        h = next(x for x in hosts if x.id == hid)
        if h.alive and h.catalog_ok and static_ok(j, h) and (jid, hid) not in quarantined:
            assert again.get(jid) == hid
