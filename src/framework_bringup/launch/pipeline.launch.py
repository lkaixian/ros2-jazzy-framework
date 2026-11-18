"""llama.cpp (C++ bridge) -> SLM -> validator -> bt_executor. Parameters: param/pipeline.yaml,
param/llama_server.yaml. dry_run defaults to true (pipeline.yaml)."""
from bt_common.launch_utils import (cast_like, declare_overrides, declare_params_dir, given,
                                    load_yaml, make_node, param_path)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction

ARGS = ('start_llama_server', 'llama_model', 'llama_hf_repo', 'llama_port', 'llama_url',
        'slm_model_path', 'max_attempts', 'dry_run', 'dry_run_exempt', 'auto_run', 'groot2', 'mode_gate', 'wicl_memory', 'wicl_verbose')


def _nodes(context):
    srv = load_yaml(param_path(context, 'llama_server.yaml'))

    def pick(arg, key):
        v = given(context, arg)
        return cast_like(v, srv.get(key)) if v is not None else srv.get(key)

    port = pick('llama_port', 'port')
    out = []
    if pick('start_llama_server', 'start'):
        model, repo = pick('llama_model', 'model'), pick('llama_hf_repo', 'hf_repo')
        cmd = ['llama-server'] + (['-m', str(model)] if model else ['-hf', str(repo)])
        cmd += ['--port', str(port)] + [str(a) for a in srv.get('extra_args', [])]
        out.append(ExecuteProcess(cmd=cmd, output='screen'))

    # a bare llama_port:= should also repoint the bridge, unless llama_url:= was given
    derived = {}
    if given(context, 'llama_url') is None and given(context, 'llama_port') is not None:
        derived['server_url'] = f'http://127.0.0.1:{port}'
    # WICL: with task memory on, the orchestrator republishes the task plus examples on bt/task_ctx
    mem = given(context, 'wicl_memory')
    wicl = (mem.strip().lower() == 'true' if mem is not None else bool(
        (load_yaml(param_path(context, 'pipeline.yaml')).get('orchestrator_node') or {})
        .get('ros__parameters', {}).get('memory_enabled', False)))
    out.append(make_node(context, 'llm_bridge', 'llm_bridge_node', 'llm_bridge_node',
                         'pipeline.yaml', {'llama_url': 'server_url'}, extra=derived,
                         **({'remappings': [('bt/task', 'bt/task_ctx')]} if wicl else {})))
    P = 'pipeline.yaml'
    exempt = given(context, 'dry_run_exempt')   # comma list; a name no extension has (e.g. none) = nothing runs for real
    out += [
        make_node(context, 'bt_slm', 'bt_slm_node', 'bt_slm_node', P, {'slm_model_path': 'model_path'}),
        make_node(context, 'bt_validator', 'bt_validator_node', 'bt_validator_node', P),
        make_node(context, 'bt_orchestrator', 'orchestrator_node', 'orchestrator_node', P,
                  {'max_attempts': 'max_attempts', 'wicl_memory': 'memory_enabled', 'wicl_verbose': 'memory_verbose'}),
        make_node(context, 'bt_orchestrator', 'scene_summarizer_node', 'scene_summarizer_node', P),
        make_node(context, 'bt_orchestrator', 'robot_mode_node', 'robot_mode_node', P),
        make_node(context, 'bt_executor', 'bt_executor_node', 'bt_executor_node', P,
                  {'dry_run': 'dry_run', 'auto_run': 'auto_run', 'groot2': 'groot2', 'mode_gate': 'mode_gate'},
                  extra={'dry_run_exempt': exempt.split(',')} if exempt else None),
    ]
    return out


def generate_launch_description():
    return LaunchDescription([declare_params_dir(), *declare_overrides(*ARGS),
                              OpaqueFunction(function=_nodes)])
