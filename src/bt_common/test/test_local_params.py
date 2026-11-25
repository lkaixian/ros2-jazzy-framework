from bt_common import launch_utils as lu


def test_local_params_env_override(tmp_path, monkeypatch):
    f = tmp_path / 'local.yaml'
    f.write_text('stt_node:\n  ros__parameters:\n    device: cpu\nllama_server:\n  ros__parameters:\n    hf_repo: x/y\n')
    monkeypatch.setenv('ROBOCUP_LOCAL_PARAMS', str(f))
    path, cfg = lu.local_params()
    assert path == str(f) and cfg['llama_server']['ros__parameters']['hf_repo'] == 'x/y'


def test_local_params_absent_is_empty(monkeypatch):
    monkeypatch.setenv('ROBOCUP_LOCAL_PARAMS', '/nonexistent/local.yaml')
    monkeypatch.setattr(lu.os.path, 'isfile', lambda p: False)
    assert lu.local_params() == ('', {})
