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
        if isinstance(s, dict) and 'foreach' in s:
            raise MacroError(f'macro {name}: foreach is for task specs, not macro bodies (use a macro parameter)')
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


_VAR = re.compile(r'[a-z][a-z0-9_]*')
MAX_FOREACH = 8


def _unroll(step, lib, stack):
    """{"foreach": [v1, v2], "as": "x", "steps": [... ${x} ...]} -> the steps once per value (unrolled at plan time)."""
    vals, var, body = step.get('foreach'), step.get('as'), step.get('steps')
    extra = set(step) - {'foreach', 'as', 'steps'}
    if extra:
        raise MacroError(f'foreach: unexpected keys {sorted(extra)} (only foreach, as, steps)')
    if not isinstance(vals, list) or not 1 <= len(vals) <= MAX_FOREACH or not all(isinstance(v, (str, int, float)) for v in vals):
        raise MacroError(f'foreach needs a list of 1 to {MAX_FOREACH} plain values')
    if not isinstance(var, str) or not _VAR.fullmatch(var):
        raise MacroError("foreach needs 'as': a lower-case variable name, used as ${name} inside steps")
    if not isinstance(body, list) or not body:
        raise MacroError('foreach needs a non-empty steps list')
    out = []
    for v in vals:
        out.extend(_expand_steps(_subst(copy.deepcopy(body), {var: str(v)}, 'foreach'), lib, stack))
    return out


def _expand_steps(steps, lib, stack):
    out = []
    for s in steps:
        if 'foreach' in s:
            out.extend(_unroll(s, lib, stack))
        elif 'macro' in s:
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


def expand(spec, lib, allowed=None):
    """spec (dict) -> a new spec with every macro step replaced by its nodes. Raises MacroError.
    allowed (a list of names, None = all): the macros the spec itself may call (the whitelist)."""
    if allowed is not None:
        bad = sorted(_macro_names(spec) - set(allowed))
        if bad:
            raise MacroError(f'macro {bad[0]!r} is not available; use one of: {", ".join(sorted(allowed))}')
    if not any('macro' in ks or 'foreach' in ks for ks in _key_sets(spec)):
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


def load_whitelist(path=None):
    """The optional top-level `whitelist: [macro, ...]` of macros.yaml: the only macros the LLM/SLM may call
    in a task spec (macros may still call any other macro inside their own steps). None = no whitelist."""
    path = default_path() if path is None else path
    if not path or not os.path.exists(path):
        return None
    wl = (yaml.safe_load(open(path)) or {}).get('whitelist')
    return None if wl is None else [str(x) for x in wl]


def callable_macros(lib, whitelist):
    return lib if whitelist is None else {k: v for k, v in lib.items() if k in whitelist}


def spec_schema(nodes, lib):
    """JSON schema of a task_spec for llama-server `response_format: json_schema`. Constrained decoding can only emit
    allowed node/macro names with their real port/arg names (string values), so call-syntax slips, invented names and
    invented ports are impossible. nodes: NodeSpec list (leaf Actions/Conditions); lib: the callable macros."""
    def obj(names, required):
        return {'type': 'object', 'additionalProperties': False, 'required': sorted(required),
                'properties': {n: {'type': 'string'} for n in names}}
    leaf = [{'type': 'object', 'required': ['node'] + (['ports'] if n.required else []), 'additionalProperties': False,
             'properties': {'node': {'const': n.id}, 'ports': obj(n.required + n.optional, n.required)}}
            for n in nodes if n.category in ('Action', 'Condition')]
    mac = []
    for name, m in sorted(lib.items()):
        req = [p for p, d in m['params'].items() if (d or {}).get('required', 'default' not in (d or {}))]
        mac.append({'type': 'object', 'required': ['macro'] + (['args'] if req else []), 'additionalProperties': False,
                    'properties': {'macro': {'const': name}, 'args': obj(list(m['params']), req)}})
    group = {'type': 'object', 'required': ['group', 'steps'], 'additionalProperties': False,
             'properties': {'group': {'enum': ['Sequence', 'Fallback', 'ReactiveSequence', 'ReactiveFallback', 'Parallel']},
                            'retry': {'type': 'integer'}, 'repeat': {'type': 'integer'},
                            'success_count': {'type': 'integer'}, 'failure_count': {'type': 'integer'},
                            'steps': {'type': 'array', 'minItems': 1, 'maxItems': 12, 'items': {'$ref': '#/$defs/step'}}}}
    each = {'type': 'object', 'required': ['foreach', 'as', 'steps'], 'additionalProperties': False,
            'properties': {'foreach': {'type': 'array', 'minItems': 1, 'maxItems': MAX_FOREACH, 'items': {'type': 'string'}},
                           'as': {'type': 'string'},
                           'steps': {'type': 'array', 'minItems': 1, 'maxItems': 12, 'items': {'$ref': '#/$defs/step'}}}}
    return {'$defs': {'step': {'anyOf': mac + leaf + [group, each]}}, 'type': 'object', 'required': ['goal', 'steps'],
            'properties': {'goal': {'type': 'string'}, 'retry': {'type': 'integer'},
                           'steps': {'type': 'array', 'minItems': 1, 'maxItems': 12, 'items': {'$ref': '#/$defs/step'}}}}


def _macro_names(spec):
    out, stack = set(), [spec]
    while stack:
        x = stack.pop()
        if isinstance(x, dict):
            if 'macro' in x:
                out.add(str(x['macro']))
            stack.extend(x.values())
        elif isinstance(x, list):
            stack.extend(x)
    return out


def describe(lib):
    """One line per macro for the LLM prompt: name(args) - description. `arg?` = optional."""
    lines = []
    for name, m in sorted(lib.items()):
        ps = ','.join(p if (d or {}).get('required', 'default' not in (d or {})) else p + '?'
                      for p, d in m['params'].items())
        lines.append(f'{name}({ps}) - {m["description"]}')
    return '\n'.join(lines)
