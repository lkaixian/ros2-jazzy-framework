"""Helpers for framework launch files: param files + explicit-only launch overrides."""
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

UNSET = '__unset__'
_INT = re.compile(r'^[+-]?\d+$')
_FLOAT = re.compile(r'^[+-]?(\d+\.\d*|\.\d+|\d+)([eE][+-]?\d+)?$')


def declare_params_dir():
    default = os.path.join(get_package_share_directory('framework_bringup'), 'param')
    return DeclareLaunchArgument('params_dir', default_value=default,
                                 description='folder with the *.yaml parameter files')


def declare_overrides(*names):
    """Launch arguments that override the param files only when given on the command line."""
    return [DeclareLaunchArgument(n, default_value=UNSET) for n in names]


def given(context, name):
    v = LaunchConfiguration(name).perform(context)
    return None if v == UNSET else v


def flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('true', '1', 'yes')


def param_path(context, filename):
    d = LaunchConfiguration('params_dir').perform(context)
    path = os.path.join(d, filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'parameter file {path} not found (params_dir={d})')
    return path


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def cast_like(text, reference):
    """Convert a command-line string to the type the param file uses for that key."""
    if isinstance(reference, bool):
        if text.strip().lower() in ('true', 'false'):
            return text.strip().lower() == 'true'
        raise ValueError(f'expected true/false, got {text!r}')
    if isinstance(reference, int):
        return int(text)
    if isinstance(reference, float):
        return float(text)
    if isinstance(reference, str):
        return text
    t = text.strip()
    if t.lower() in ('true', 'false'):
        return t.lower() == 'true'
    if _INT.match(t):
        return int(t)
    if _FLOAT.match(t):
        return float(t)
    return text


def overrides(context, path, node, mapping):
    """{launch_arg: param} -> {param: value} for args the user actually passed."""
    ref = (load_yaml(path).get(node) or {}).get('ros__parameters', {})
    out = {}
    for arg, param in mapping.items():
        v = given(context, arg)
        if v is not None:
            try:
                out[param] = cast_like(v, ref.get(param))
            except ValueError as e:
                raise ValueError(f'launch argument {arg}:={v} for {node}.{param}: {e}') from None
    return out


def make_node(context, pkg, exe, node, filename, mapping=None, extra=None, **kw):
    """Node loading `filename`, plus explicitly-given launch args (and `extra` programmatic values)."""
    path = param_path(context, filename)
    params = [path]
    merged = overrides(context, path, node, mapping or {})
    merged.update(extra or {})
    if merged:
        params.append(merged)
    return Node(package=pkg, executable=exe, output='screen', parameters=params, **kw)
