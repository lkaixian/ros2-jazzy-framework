"""Pure-Python BT XML validation (no ROS needed, unit-testable)."""
import math
import re
import xml.etree.ElementTree as ET

BB_REF = re.compile(r'^\{[A-Za-z_][\w]*\}$')
COMMON_ATTRS = {'name'}
# shortcut: numeric ports are listed by name (manifests carry no port types); add a name here when an extension gets a new numeric port
NUMERIC_PORTS = {'distance', 'kd', 'ki', 'kp', 'meters', 'min_conf', 'pitch', 'radians', 'roll', 'speed', 'target_height', 'timeout_ms',
                 'tolerance', 'width', 'x', 'y', 'yaw', 'z', 'num_cycles', 'num_attempts', 'success_count', 'failure_count'}


def _is_number(v):
    try:
        return math.isfinite(float(v))
    except ValueError:
        return False


def _check_parallel_extensions(par, catalog, tid, errors):
    """An extension runs one goal at a time and rejects the second: two branches of a Parallel must not both use one extension (T-25)."""
    owner = {n: ext for ext, nodes in catalog.extensions.items() for n in nodes}
    seen = {}
    for i, child in enumerate(par):
        for ext in {owner[e.tag] for e in child.iter() if e.tag in owner}:
            if ext in seen and seen[ext] != i:
                errors.append(f"[{tid}] <Parallel> starts two nodes of extension '{ext}' at once; it accepts one goal at a time "
                              f"(put them in a Sequence)")
            seen.setdefault(ext, i)


def validate_xml(xml, catalog, max_depth=12, max_nodes=64, vocab=None):
    """Return a list of human/LLM-readable error strings; empty means valid.

    vocab: optional {(node, port): set(allowed literal values)}, e.g. the detector's classes for
    (DetectObject, label). Blackboard references {x} are not checked.
    """
    errors = []
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as e:
        return [f'XML is not well-formed: {e}']
    if root.tag != 'root':
        return [f"top-level element must be <root>, found <{root.tag}>"]
    if root.get('BTCPP_format') != '4':
        errors.append('<root> must have BTCPP_format="4"')

    trees = {}
    for t in root:
        if t.tag == 'BehaviorTree':
            tid = t.get('ID')
            if not tid:
                errors.append('<BehaviorTree> is missing ID')
            elif tid in trees:
                errors.append(f"duplicate BehaviorTree ID '{tid}'")
            else:
                trees[tid] = t
        elif t.tag != 'TreeNodesModel':
            errors.append(f'unexpected element <{t.tag}> under <root>')
    if not trees:
        return errors + ['no <BehaviorTree> defined']
    main = root.get('main_tree_to_execute')
    if not main:
        errors.append('<root> is missing main_tree_to_execute')
    elif main not in trees:
        errors.append(f"main_tree_to_execute '{main}' does not match any BehaviorTree")

    specs = catalog.all()
    refs = {tid: set() for tid in trees}
    count = [0]

    def walk(el, tid, depth):
        count[0] += 1
        if depth > max_depth:
            errors.append(f'[{tid}] nesting deeper than {max_depth}')
            return
        if el.tag == 'SubTree':
            target = el.get('ID')
            if not target:
                errors.append(f'[{tid}] <SubTree> missing ID')
            elif target not in trees:
                errors.append(f"[{tid}] <SubTree ID='{target}'> is not defined")
            else:
                refs[tid].add(target)
            if len(el):
                errors.append(f'[{tid}] <SubTree> must not have children')
            return
        spec = specs.get(el.tag)
        if spec is None:
            errors.append(f"[{tid}] unknown node <{el.tag}> (not in core or any extension)")
            for c in el:
                walk(c, tid, depth + 1)
            return
        n = len(el)
        if spec.category == 'Control' and n < 1:
            errors.append(f'[{tid}] control node <{el.tag}> needs at least 1 child')
        elif spec.category == 'Decorator' and n != 1:
            errors.append(f'[{tid}] decorator <{el.tag}> needs exactly 1 child, has {n}')
        elif spec.category in ('Action', 'Condition') and n:
            errors.append(f'[{tid}] leaf <{el.tag}> must not have children')
        if el.tag == 'Parallel':
            _check_parallel_extensions(el, catalog, tid, errors)
        allowed = set(spec.required) | set(spec.optional) | COMMON_ATTRS
        for p in spec.required:
            if p not in el.attrib:
                errors.append(f"[{tid}] <{el.tag}> missing required port '{p}'")
        for a, v in el.attrib.items():
            if a not in allowed:
                errors.append(f"[{tid}] <{el.tag}> has unknown port '{a}'")
            elif v.startswith('{') and not BB_REF.match(v):
                errors.append(f"[{tid}] <{el.tag}> port '{a}' has malformed blackboard ref '{v}'")
            elif v == '':
                errors.append(f"[{tid}] <{el.tag}> port '{a}' is empty")
            elif a in NUMERIC_PORTS and not BB_REF.match(v) and not _is_number(v):
                errors.append(f"[{tid}] <{el.tag}> port '{a}'='{v}' is not a number")
            elif vocab and (el.tag, a) in vocab and not BB_REF.match(v) \
                    and v not in vocab[(el.tag, a)]:
                known = ', '.join(sorted(vocab[(el.tag, a)])[:30])
                errors.append(f"[{tid}] <{el.tag}> port '{a}'='{v}' is not a known value "
                              f"(known: {known})")
        for c in el:
            walk(c, tid, depth + 1)

    for tid, t in trees.items():
        if len(t) != 1:
            errors.append(f'[{tid}] <BehaviorTree> must contain exactly 1 root node, has {len(t)}')
        for c in t:
            walk(c, tid, 1)
    if count[0] > max_nodes:
        errors.append(f'tree has {count[0]} nodes, limit is {max_nodes}')

    # SubTree cycle detection
    state = {}

    def dfs(t, path):
        state[t] = 1
        for r in refs[t]:
            if state.get(r) == 1:
                errors.append('SubTree cycle: ' + ' -> '.join(path + [t, r]))
            elif r not in state:
                dfs(r, path + [t])
        state[t] = 2
    for t in trees:
        if t not in state:
            dfs(t, [])
    return errors
