import copy
import os

import pytest
import yaml

from cluster_core.catalog import CatalogError, expand, load_catalog, parse_catalog

SHIPPED = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param',
                       'cluster_jobs.yaml')


def cat(**jobs):
    return parse_catalog({'jobs': jobs})


def test_shipped_catalog_loads_and_has_the_expected_shape():
    c = load_catalog(SHIPPED)
    for core in ('cmd_vel_mux', 'odom_extension', 'pid_extension'):
        assert c[core]['role'] == 'core' and c[core]['movable'] is False
        assert c[core]['stop_on_orphan'] is False          # hardware-bound: fail-static
    big, small = c['llm_big'], c['llm_small']
    assert big['tags'] == ['llm_big'] and small['tags'] == ['robot']
    assert big['endpoint']['kind'] == small['endpoint']['kind'] == 'llm'
    assert big['endpoint']['priority'] < small['endpoint']['priority']       # big PC preferred
    assert big['endpoint']['port'] != small['endpoint']['port']
    assert small['movable'] is False                                         # stays with the robot
    assert c['llm_bridge']['role'] == 'core' and c['llm_bridge']['movable'] is False
    assert 'use_static_urls:=false' in c['llm_bridge']['cmds'][0]            # cluster-only endpoints
    for j in (big, small):                                                   # reachable from other PCs
        assert '0.0.0.0' in j['cmds'][0]
    assert c['stt']['tags'] == ['audio'] and c['tts']['tags'] == ['audio']
    assert 'camera' in c['person_detection']['tags']
    for jid in c.ids():
        for argv in c[jid]['cmds']:
            expand(argv, {'PARAMS_DIR': '/p'})             # every ${VAR} is a known one


def test_string_commands_are_rejected_no_shell():
    with pytest.raises(CatalogError, match='argv list'):
        cat(a={'cmd': 'ros2 run x y; rm -rf /'})


def test_cmds_must_be_lists_of_argv_lists():
    with pytest.raises(CatalogError):
        cat(a={'cmds': 'echo hi'})
    with pytest.raises(CatalogError, match='argv list'):
        cat(a={'cmds': ['echo hi']})
    with pytest.raises(CatalogError):
        cat(a={'cmd': []})
    with pytest.raises(CatalogError, match='either'):
        cat(a={'cmd': ['x'], 'cmds': [['y']]})
    with pytest.raises(CatalogError):
        cat(a={})


@pytest.mark.parametrize('bad', ['', 'a b', 'a/b', '../x', 'x' * 65, 'é'])
def test_bad_job_ids(bad):
    with pytest.raises(CatalogError, match='invalid job id'):
        cat(**{bad: {'cmd': ['true']}})


def test_bad_values():
    for spec in ({'cmd': ['x'], 'cpu': -1}, {'cmd': ['x'], 'cpu': 'lots'},
                 {'cmd': ['x'], 'movable': 'yes'}, {'cmd': ['x'], 'tags': 'camera'},
                 {'cmd': ['x'], 'restart': {'bogus': 1}}, {'cmd': ['x'], 'env': [1]}):
        with pytest.raises(CatalogError):
            cat(a=spec)


def test_defaults_and_orphan_rule():
    j = cat(a={'cmd': ['x']}, b={'cmd': ['x'], 'movable': False})['a']
    assert j['movable'] and j['stop_on_orphan'] and j['cpu'] == 0.5 and j['restart']['max_restarts'] == 5
    assert cat(b={'cmd': ['x'], 'movable': False})['b']['stop_on_orphan'] is False
    assert cat(b={'cmd': ['x'], 'movable': False, 'stop_on_orphan': True})['b']['stop_on_orphan']


def test_hash_is_stable_and_sensitive():
    base = {'a': {'cmd': ['x', '1'], 'cpu': 1}, 'b': {'cmd': ['y']}}
    h = parse_catalog({'jobs': base}).hash
    assert parse_catalog({'jobs': copy.deepcopy(base)}).hash == h
    assert parse_catalog({'jobs': dict(reversed(list(base.items())))}).hash == h   # order-free
    for mutate in (lambda d: d['a']['cmd'].append('--evil'), lambda d: d['a'].update(cpu=2),
                   lambda d: d['b'].update(tags=['gpu']), lambda d: d['b'].update(movable=False),
                   lambda d: d.pop('b')):
        d = copy.deepcopy(base)
        mutate(d)
        assert parse_catalog({'jobs': d}).hash != h


def test_expand_only_known_variables():
    assert expand(['--p', '${PARAMS_DIR}/x.yaml'], {'PARAMS_DIR': '/etc'}) == ['--p', '/etc/x.yaml']
    with pytest.raises(CatalogError, match='unknown variable'):
        expand(['${HOME}/x'], {'PARAMS_DIR': '/etc'})        # no environment leakage


def test_yaml_loader(tmp_path):
    p = tmp_path / 'c.yaml'
    p.write_text(yaml.safe_dump({'jobs': {'a': {'cmd': ['true']}}}))
    assert load_catalog(str(p)).ids() == ['a']
