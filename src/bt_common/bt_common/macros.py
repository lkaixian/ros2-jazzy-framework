"""Macros: named, parameterised step lists the LLM can call instead of writing every node.

A spec step {"macro": "go_to_place", "args": {"place": "door"}} is expanded (here, in plain code,
before the SLM / rule generator sees the spec) into the real nodes of param/macros.yaml. The LLM
writes a handful of tokens instead of a whole sub-tree, and the expansion is deterministic and
validated by tests, so it cannot invent ports.

  macros:
    go_to_place:
      description: "..."                       # shown to the LLM
      params: {place: {required: true}, speed: {default: "0.2"}}
      example_args: {place: door}              # used by the test that validates every macro
      steps: [ {node: NavigateToPlace, ports: {place: "${place}"}}, ... ]   # spec steps; may call other macros
"""
import copy
import os
import re

import yaml

MAX_DEPTH = 4
_REF = re.compile(r'\$\{([A-Za-z_]\w*)\}')


class MacroError(ValueError):
    pass


def default_path():
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return os.path.join(get_package_share_directory('framework_bringup'), 'param',
                            'macros.yaml')
    except PackageNotFoundError:
        return ''


def _check_steps(name, steps):
    if not isinstance(steps, list) or not steps:
        raise MacroError(f'macro {name}: steps must be a non-empty list')
    for s in steps:
        if not isinstance(s, dict) or sum(k in s for k in ('node', 'group', 'macro')) != 1:
            raise MacroError(f"macro {name}: every step needs exactly one of node/group/macro: {s!r}")
        if 'group' in s:
            _check_steps(name, s.get('steps'))


def load_macros(path=None):
    """{name: {description, params, steps, example_args}}; {} if the file does not exist."""
    path = default_path() if path is None else path
    if not path or not os.path.exists(path):
        return {}
    data = (yaml.safe_load(open(path)) or {}).get('macros') or {}
    for name, m in data.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]*', str(name)):
            raise MacroError(f'macro name {name!r}: use lower-case letters, digits, _')
        _check_steps(name, m.get('steps'))
        m.setdefault('description', '')
        m['params'] = m.get('params') or {}
        m['example_args'] = m.get('example_args') or {}
    return data


def _subst(value, args, macro):
    if isinstance(value, str):
        def one(mt):
            if mt.group(1) not in args:
                raise MacroError(f'macro {macro}: unknown parameter ${{{mt.group(1)}}}')
            return args[mt.group(1)]
        return _REF.sub(one, value)
    if isinstance(value, dict):
        return {k: _subst(v, args, macro) for k, v in value.items()}
    if isinstance(value, list):
        return [_subst(v, args, macro) for v in value]
    return value


def _expand_steps(steps, lib, stack):
    out = []
    for s in steps:
        if 'macro' in s:
            out.extend(_call(s, lib, stack))
        elif 'group' in s:
            out.append(dict(s, steps=_expand_steps(s.get('steps', []), lib, stack)))
        else:
            out.append(s)
    return out


def _call(step, lib, stack):
    name = step['macro']
    extra = set(step) - {'macro', 'args'}
    if extra:
        raise MacroError(f'macro call {name}: unexpected keys {sorted(extra)} (only macro, args)')
    if name not in lib:
        raise MacroError(f'unknown macro {name!r}; known: {", ".join(sorted(lib)) or "none"}')
    if name in stack:
        raise MacroError(f'macro {name} calls itself ({" -> ".join(stack + [name])})')
    if len(stack) >= MAX_DEPTH:
        raise MacroError(f'macros nested deeper than {MAX_DEPTH}: {" -> ".join(stack + [name])}')
    m = lib[name]
    given = step.get('args') or {}
    if not isinstance(given, dict):
        raise MacroError(f'macro {name}: args must be an object')
    unknown = set(given) - set(m['params'])
    if unknown:
        raise MacroError(f'macro {name}: unknown args {sorted(unknown)}; '
                         f'it takes {sorted(m["params"]) or "no args"}')
    args = {}
    for p, d in m['params'].items():
        d = d or {}
        if p in given:
            args[p] = str(given[p])
        elif 'default' in d:
            args[p] = str(d['default'])
        elif d.get('required', True):
            raise MacroError(f'macro {name}: missing required arg {p!r}')
    body = _subst(copy.deepcopy(m['steps']), args, name)
    return _expand_steps(body, lib, stack + [name])


def expand(spec, lib):
    """spec (dict) -> a new spec with every macro step replaced by its nodes. Raises MacroError."""
    if not any('macro' in ks for ks in _key_sets(spec)):
        return spec
    out = dict(spec)
    out['steps'] = _expand_steps(spec.get('steps', []), lib, [])
    return out


def _key_sets(spec):
    """All dict-key sets in a spec (to cheaply detect whether it uses macros at all)."""
    stack = [spec]
    while stack:
        x = stack.pop()
        if isinstance(x, dict):
            yield set(x)
            stack.extend(x.values())
        elif isinstance(x, list):
            stack.extend(x)


def describe(lib):
    """One line per macro for the LLM prompt: name(args) - description. `arg?` = optional."""
    lines = []
    for name, m in sorted(lib.items()):
        ps = ','.join(p if (d or {}).get('required', 'default' not in (d or {})) else p + '?'
                      for p, d in m['params'].items())
        lines.append(f'{name}({ps}) - {m["description"]}')
    return '\n'.join(lines)
