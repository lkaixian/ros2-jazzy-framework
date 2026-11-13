"""validate_bt: check hand-written BT XML files against the installed node catalog."""
import argparse
import sys

from bt_common.catalog import discover_catalog

from bt_validator.validate import validate_xml


def list_nodes(cat):
    out = []
    for n in sorted(cat.all().values(), key=lambda s: (s.category, s.id)):
        ports = ', '.join(n.required + [p + '?' for p in n.optional]) or '-'
        out.append(f'{n.category:<9} {n.id:<22} ports: {ports}' +
                   (f'\n{"":<10}{n.description}' if n.description else ''))
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('files', nargs='*', help='BT XML files to check')
    ap.add_argument('--list-nodes', action='store_true',
                    help='print every usable node and its ports (trailing ? = optional)')
    ap.add_argument('--extension-config', action='append', default=[],
                    help='extra extension manifest YAML (repeatable)')
    ap.add_argument('--labels', default='',
                    help="comma-separated DetectObject labels to accept (your model's classes)")
    ap.add_argument('--max-depth', type=int, default=12)
    ap.add_argument('--max-nodes', type=int, default=64)
    a = ap.parse_args(argv)
    cat = discover_catalog(a.extension_config)
    if a.list_nodes:
        print(list_nodes(cat))
    if not a.files and not a.list_nodes:
        ap.error('give at least one XML file or --list-nodes')
    labels = {s.strip() for s in a.labels.split(',') if s.strip()}
    vocab = {('DetectObject', 'label'): labels} if labels else None
    bad = 0
    for path in a.files:
        try:
            with open(path) as f:
                xml = f.read()
        except OSError as e:
            print(f'{path}: cannot read: {e}')
            bad += 1
            continue
        errors = validate_xml(xml, cat, a.max_depth, a.max_nodes, vocab)
        if errors:
            bad += 1
            print(f'{path}: INVALID ({len(errors)} error(s))')
            for e in errors:
                print(f'  - {e}')
        else:
            print(f'{path}: OK')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
