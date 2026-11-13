import os

import yaml

from cluster_core import plan_preview as pp
from cluster_core import status_cli
from cluster_core.agent_node import split_csv
from cluster_core.catalog import load_catalog

SHIPPED = os.path.join(os.path.dirname(__file__), '..', '..', 'framework_bringup', 'param',
                       'cluster_jobs.yaml')

HOSTS = {'hosts': {
    # big enough for everything the shipped catalog pins to camera/audio hosts (a 4-core board is
    # NOT: the planner would correctly leave stt/tts pending)
    'pi': {'roles': 'core', 'tags': 'camera,audio', 'cores': 16, 'cpu_load': 0.1,
           'mem_total_mb': 16000, 'mem_free_mb': 12000},
    'laptop': {'roles': 'compute', 'tags': ['gpu', 'robot'], 'cores': 12, 'cpu_load': 0.2,
               'mem_total_mb': 16000, 'mem_free_mb': 9000, 'gpu_mem_free_mb': 3500},
    'strongpc': {'roles': 'compute', 'tags': ['gpu', 'llm_big'], 'cores': 16, 'cpu_load': 0.1,
                 'mem_total_mb': 32000, 'mem_free_mb': 24000, 'gpu_mem_free_mb': 10000},
}}


def hosts_file(tmp_path, data=HOSTS):
    p = tmp_path / 'hosts.yaml'
    p.write_text(yaml.safe_dump(data))
    return str(p)


def test_split_csv():
    assert split_csv('core, camera ,,core') == ['camera', 'core']
    assert split_csv('') == [] and split_csv(None) == []


def test_preview_on_the_shipped_catalog(tmp_path):
    cat = load_catalog(SHIPPED)
    jobs, first, second = pp.run(pp.load_hosts(hosts_file(tmp_path)), cat)
    a = first.assignment
    assert {a[j] for j in ('cmd_vel_mux', 'odom_extension', 'pid_extension')} == {'pi'}   # role core
    assert a['stt'] == a['tts'] == 'pi'                                                   # tag audio
    assert a['llm_big'] == 'strongpc' and a['llm_small'] == 'laptop'                      # by tag
    assert a['llm_bridge'] == 'pi'                                                        # role core
    assert second.assignment == first.assignment                                          # stable
    text = pp.render(jobs, pp.load_hosts(hosts_file(tmp_path)), first, second)
    assert 'identical (stable)' in text and 'llm_big' in text


def test_preview_with_the_core_host_down(tmp_path):
    cat = load_catalog(SHIPPED)
    hosts = pp.load_hosts(hosts_file(tmp_path), dead=['pi'])
    _, first, _ = pp.run(hosts, cat)
    for jid in ('cmd_vel_mux', 'odom_extension', 'pid_extension', 'stt', 'tts'):
        assert first.placements[jid].state == 'unplaceable'                              # never moved to the laptop
        assert first.assignment.get(jid) is None
    assert first.assignment['llm_big'] == 'strongpc' and first.assignment['llm_small'] == 'laptop'
    assert first.placements['llm_bridge'].state == 'unplaceable'    # bridge is core-bound too


def test_strong_pc_down_leaves_llm_big_unplaced_and_small_untouched(tmp_path):
    cat = load_catalog(SHIPPED)
    hosts = pp.load_hosts(hosts_file(tmp_path), dead=['strongpc'])
    _, first, _ = pp.run(hosts, cat, current={'llm_big': 'strongpc', 'llm_small': 'laptop'})
    assert first.assignment.get('llm_big') is None           # never moved onto the 4 GB laptop
    assert first.assignment['llm_small'] == 'laptop'


def test_preview_failover_of_a_movable_job(tmp_path):
    cat = load_catalog(SHIPPED)
    hosts = pp.load_hosts(hosts_file(tmp_path), dead=['laptop'])
    _, first, _ = pp.run(hosts, cat, current={'validator': 'laptop', 'odom_extension': 'pi'})
    assert first.assignment['validator'] == 'pi' and 'failover validator: laptop -> pi' in first.events
    assert first.assignment['odom_extension'] == 'pi'


def test_preview_cli_exit_status(tmp_path, capsys):
    args = ['--hosts', hosts_file(tmp_path), '--jobs', SHIPPED]
    assert pp.main(args) in (0, 1)                       # 1 only if something does not fit
    out = capsys.readouterr().out
    assert 'JOB' in out and 'second pass' in out
    assert pp.main(args + ['--dead', 'pi']) == 1         # unplaceable core jobs -> non-zero


def status(**over):
    s = {'scheduler_id': 's1', 'epoch': 7, 'alive': ['pi', 'laptop'], 'dead': [], 'excluded': [],
         'events': ['assign llm -> laptop'],
         'jobs': [{'job_id': 'llm_big', 'host_id': 'laptop', 'state': 'placed', 'reason': '', 'job_state': 'running'}]}
    s.update(over)
    return s


HOST_ROWS = {'pi': {'roles': ['core'], 'tags': ['camera'], 'cores': 4, 'cpu_load': 0.25,
                    'mem_free_mb': 3000.0, 'gpu_mem_free_mb': -1.0, 'njobs': 3},
             'laptop': {'roles': ['compute'], 'tags': ['gpu'], 'cores': 12, 'cpu_load': 0.5,
                        'mem_free_mb': 9000.0, 'gpu_mem_free_mb': 3500.0, 'njobs': 5}}


def test_status_render_healthy():
    text, healthy = status_cli.render(status(), HOST_ROWS)
    assert healthy
    for needle in ('scheduler s1  epoch 7', 'laptop', '3500MB', '25%/4c', 'llm', 'assign llm -> laptop'):
        assert needle in text


def test_status_render_flags_trouble():
    bad = status(dead=['pi'], alive=['laptop'], jobs=[
        {'job_id': 'odom', 'host_id': 'pi', 'state': 'unplaceable', 'reason': 'pinned to pi, which is down',
         'job_state': 'unknown'}])
    text, healthy = status_cli.render(bad, HOST_ROWS)
    assert not healthy and 'DEAD' in text and 'pinned to pi' in text
    text, healthy = status_cli.render(status(excluded=['pi'], alive=['laptop']), HOST_ROWS)
    assert not healthy and 'EXCLUDED(catalog)' in text
    failing = status(jobs=[{'job_id': 'x', 'host_id': 'pi', 'state': 'placed', 'reason': '', 'job_state': 'failed'}])
    assert not status_cli.render(failing, HOST_ROWS)[1]
