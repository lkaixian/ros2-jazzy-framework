import pytest
from cluster_core.catalog import CatalogError, parse_catalog
from cluster_core.endpoints import RUNNING, advertised, local_address

JOBS = {'jobs': {
    'llm_big': {'cmd': ['llama-server'], 'tags': ['llm_big'],
                'endpoint': {'kind': 'llm', 'port': 8080, 'priority': 10}},
    'plain': {'cmd': ['true']},
}}


def test_endpoint_parsed_and_in_hash():
    c = parse_catalog(JOBS)
    assert c['llm_big']['endpoint'] == {'kind': 'llm', 'port': 8080, 'priority': 10}
    assert c['plain']['endpoint'] is None
    other = {'jobs': {**JOBS['jobs'], 'llm_big': {**JOBS['jobs']['llm_big'],
                                                  'endpoint': {'kind': 'llm', 'port': 9090}}}}
    assert parse_catalog(other).hash != c.hash          # a different port must not look identical


@pytest.mark.parametrize('bad', [
    'http://x', {'kind': 'llm'}, {'port': 80}, {'kind': 'llm', 'port': 0},
    {'kind': 'llm', 'port': 70000}, {'kind': 'llm', 'port': True},
    {'kind': 'llm', 'port': 80, 'extra': 1}, {'kind': '', 'port': 80},
    {'kind': 'llm', 'port': 80, 'priority': 'high'}])
def test_bad_endpoint_rejected(bad):
    with pytest.raises(CatalogError):
        parse_catalog({'jobs': {'j': {'cmd': ['true'], 'endpoint': bad}}})


def test_only_running_jobs_advertise():
    c = parse_catalog(JOBS)
    st = lambda jid, state: {'job_id': jid, 'state': state}  # noqa: E731
    assert advertised(c, [st('llm_big', RUNNING), st('plain', RUNNING)], '10.0.0.5') == [
        ('llm_big', 'llm', 'http://10.0.0.5:8080', 10)]
    for state in (0, 2, 3, 4):      # stopped, backoff, failed, stopping
        assert advertised(c, [st('llm_big', state)], '10.0.0.5') == []
    assert advertised(c, [st('unknown_job', RUNNING)], '10.0.0.5') == []


def test_local_address_is_an_ip_string():
    a = local_address()
    assert a.count('.') == 3
