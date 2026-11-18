#!/usr/bin/python3
"""check_params: verify the param/*.yaml files against the real nodes.

Starts each node with its section of the file on an isolated DDS domain, then compares what the
node actually loaded with the file. A key the node does not declare (typo) or a value that did not
load (wrong type) is reported; ROS itself ignores such keys silently.

  ros2 run framework_bringup check_params [--params-dir DIR] [--only NODE ...] [--perception]
Exit status 1 if any problem is found. Perception nodes need cameras/weights and are skipped
unless --perception is given.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import yaml

# node name -> (package, executable, param file, overrides applied only to make it start safely)
NODES = {
    'llm_bridge_node': ('llm_bridge', 'llm_bridge_node', 'pipeline.yaml', {}),
    'bt_slm_node': ('bt_slm', 'bt_slm_node', 'pipeline.yaml', {}),
    'bt_validator_node': ('bt_validator', 'bt_validator_node', 'pipeline.yaml', {}),
    'orchestrator_node': ('bt_orchestrator', 'orchestrator_node', 'pipeline.yaml', {}),
    'scene_summarizer_node': ('bt_orchestrator', 'scene_summarizer_node', 'pipeline.yaml', {}),
    'bt_executor_node': ('bt_executor', 'bt_executor_node', 'pipeline.yaml', {}),
    'battery_guard_node': ('bt_common', 'battery_guard_node', 'extensions.yaml', {}),
    'cmd_vel_mux_node': ('bt_common', 'cmd_vel_mux_node', 'extensions.yaml', {}),
    'odom_extension_node': ('odom_extension', 'odom_extension_node', 'extensions.yaml', {}),
    'pid_extension_node': ('pid_extension', 'pid_extension_node', 'extensions.yaml', {}),
    'pose_extension_node': ('pose_extension', 'pose_extension_node', 'extensions.yaml', {}),
    'arm_extension_node': ('arm_extension', 'arm_extension_node', 'extensions.yaml', {}),
    'social_extension_node': ('social_extension', 'social_extension_node', 'extensions.yaml', {}),
    'nav_extension_node': ('nav_extension', 'nav_extension_node', 'extensions.yaml', {}),
    'nav_sim_node': ('nav_sim', 'nav_sim_node', 'nav_sim.yaml', {}),
    'person_depth_node': ('person_detection', 'person_depth_node', 'perception.yaml', {}),
    'audio_capture_node': ('audio_capture', 'audio_capture_node', 'audio.yaml',
                           {'source': 'file', 'file': '/nonexistent.wav'}),
    'vad_node': ('stt_pipeline', 'vad_node', 'audio.yaml', {}),
    'sound_event_node': ('stt_pipeline', 'sound_event_node', 'audio.yaml', {}),
    'stt_node': ('stt_pipeline', 'stt_node', 'audio.yaml', {'engine': 'mock'}),
    'transcript_to_task_node': ('stt_pipeline', 'transcript_to_task_node', 'audio.yaml', {}),
    'tts_node': ('tts_pipeline', 'tts_node', 'audio.yaml', {'engine': 'mock'}),
    'plan_announcer_node': ('tts_pipeline', 'plan_announcer_node', 'audio.yaml', {}),
    'cluster_agent_node': ('cluster_core', 'cluster_agent_node', 'cluster.yaml', {}),
    'cluster_scheduler_node': ('cluster_core', 'cluster_scheduler_node', 'cluster.yaml', {}),
}
PERCEPTION = {
    'object_detection_node': ('object_detection', 'object_detection_node', 'perception.yaml', {}),
    'person_detection_node': ('person_detection', 'person_detection_node', 'perception.yaml', {}),
    'face_recognition_node': ('face_recognition', 'face_recognition_node', 'perception.yaml', {}),
    'auto_register_node': ('face_recognition', 'auto_register_node', 'perception.yaml', {}),
}


def wanted(node, params_dir, fname):
    path = os.path.join(params_dir, fname)
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get(node, {}).get('ros__parameters', {}), path


def same(a, b):
    if isinstance(a, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return type(a) is type(b) and abs(a - b) < 1e-9
    return a == b


def value_of(pv):
    from rcl_interfaces.msg import ParameterType as T
    return {T.PARAMETER_BOOL: lambda: pv.bool_value, T.PARAMETER_INTEGER: lambda: pv.integer_value,
            T.PARAMETER_DOUBLE: lambda: pv.double_value, T.PARAMETER_STRING: lambda: pv.string_value,
            T.PARAMETER_STRING_ARRAY: lambda: list(pv.string_array_value),
            T.PARAMETER_DOUBLE_ARRAY: lambda: list(pv.double_array_value),
            T.PARAMETER_INTEGER_ARRAY: lambda: list(pv.integer_array_value),
            T.PARAMETER_BOOL_ARRAY: lambda: list(pv.bool_array_value),
            T.PARAMETER_NOT_SET: lambda: None}[pv.type]()


def died_message(proc, fallback):
    if proc.poll() is None:
        return fallback
    err = proc.stderr.read().decode(errors='replace').strip().splitlines()[-2:]
    return f'node crashed: {" | ".join(err) or fallback}'


def check_node(rclpy, probe, node, spec, params_dir, wait):
    from rcl_interfaces.srv import GetParameters, ListParameters
    pkg, exe, fname, overrides = spec
    want, path = wanted(node, params_dir, fname)
    if not want:
        return [f'{fname} has no section for {node}']
    cmd = ['ros2', 'run', pkg, exe, '--ros-args', '--params-file', path]
    for k, v in overrides.items():
        cmd += ['-p', f'{k}:={v}']
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                            start_new_session=True)
    problems = []
    try:
        lst = probe.create_client(ListParameters, f'/{node}/list_parameters')
        get = probe.create_client(GetParameters, f'/{node}/get_parameters')
        end = time.monotonic() + wait
        while not lst.service_is_ready() and time.monotonic() < end and proc.poll() is None:
            rclpy.spin_once(probe, timeout_sec=0.1)
        if proc.poll() is not None or not lst.service_is_ready():
            err = proc.stderr.read().decode(errors='replace').strip().splitlines()[-3:] \
                if proc.poll() is not None else ['did not come up in time']
            return [f'node did not start: {" | ".join(err)}']
        fut = lst.call_async(ListParameters.Request())
        rclpy.spin_until_future_complete(probe, fut, timeout_sec=30.0)   # a node loading a model answers only after it
        if fut.result() is None:
            return [died_message(proc, 'parameter service did not answer')]
        declared = set(fut.result().result.names)
        for key in sorted(want):
            if key not in declared:
                problems.append(f"'{key}' is not a parameter of {node} (typo? ignored by ROS)")
        keys = [k for k in want if k in declared and k not in overrides]
        if keys:
            req = GetParameters.Request(names=keys)
            fut = get.call_async(req)
            rclpy.spin_until_future_complete(probe, fut, timeout_sec=30.0)   # a node loading a model answers only after it
            if fut.result() is None:
                return problems + [died_message(proc, 'get_parameters did not answer')]
            for k, pv in zip(keys, fut.result().values):
                got = value_of(pv)
                if not same(want[k], got):
                    problems.append(f"'{k}': file has {want[k]!r}, node loaded {got!r}")
    finally:
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)     # the whole group: ros2 run spawns a child
            except ProcessLookupError:       # already gone (e.g. it crashed at startup)
                break
            try:
                proc.wait(5)
                break
            except subprocess.TimeoutExpired:
                continue
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    default = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'param')
    try:
        from ament_index_python.packages import get_package_share_directory
        default = os.path.join(get_package_share_directory('framework_bringup'), 'param')
    except Exception:  # noqa: BLE001
        pass
    ap.add_argument('--params-dir', default=default)
    ap.add_argument('--only', nargs='*', help='node names to check')
    ap.add_argument('--perception', action='store_true', help='also start the YOLO nodes')
    ap.add_argument('--wait', type=float, default=15.0, help='seconds to wait per node')
    a = ap.parse_args(argv)
    # isolate from any real robot/network: private domain, localhost only
    os.environ['ROS_DOMAIN_ID'] = '97'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    rclpy.init()
    probe = rclpy.create_node('check_params_probe')
    nodes = dict(NODES, **(PERCEPTION if a.perception else {}))
    if a.only:
        nodes = {k: v for k, v in nodes.items() if k in a.only}
    bad = 0
    try:
        for node, spec in nodes.items():
            probs = check_node(rclpy, probe, node, spec, a.params_dir, a.wait)
            print(f'{"FAIL" if probs else "ok  "} {node}')
            for p in probs:
                print(f'       - {p}')
            bad += bool(probs)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
    print(f'{len(nodes) - bad}/{len(nodes)} nodes consistent with {a.params_dir}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
