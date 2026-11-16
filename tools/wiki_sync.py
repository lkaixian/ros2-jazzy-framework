#!/usr/bin/env python3
"""Publish docs/*.md to the GitHub wiki (docs/ stays the source of truth; the wiki is a generated copy).

One-time: create any first page in the repo's Wiki tab (GitHub makes the wiki repo only then). Then: python3 tools/wiki_sync.py [--push]
Without --push it writes the pages to a temp clone and shows the diff only."""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = 'lkaixian/ros2-jazzy-framework'
RAW = f'https://raw.githubusercontent.com/{REPO}/main/docs/'
TITLE = {'operator-guide': 'Operator guide', 'slm-benchmark': 'SLM benchmark', 'task-spec-grammar': 'Task spec grammar',
         'llm-and-models': 'LLM and models', 'workspaces-security': 'Workspaces and security', 'node-catalog': 'BT node catalog',
         'slm-training': 'SLM training', 'testing-and-ci': 'Testing and CI', 'faq': 'FAQ', 'concepts-and-glossary': 'Concepts and glossary',
         'tutorial-first-plan': 'Tutorial first plan', 'start-here': 'Start here'}
SIDEBAR = [('Start', ['start-here', 'concepts-and-glossary', 'tutorial-first-plan', 'faq']),
           ('Understand', ['architecture', 'interfaces', 'robot-mode-and-recovery', 'safety', 'status']),
           ('Plan', ['task-spec-grammar', 'macros', 'node-catalog', 'validator-rules', 'behavior-trees', 'llm-and-models']),
           ('Operate', ['operator-guide', 'configuration', 'parameters', 'dashboard', 'troubleshooting']),
           ('Robot', ['perception', 'navigation', 'audio', 'writing-an-extension']),
           ('Models and evaluation', ['evaluation', 'slm-training', 'slm-benchmark']),
           ('Multi-machine', ['cluster', 'workspaces-security']),
           ('Reference', ['messages', 'testing-and-ci'])]


def name(stem):
    return TITLE.get(stem, stem.replace('-', ' ').replace('_', ' ').capitalize())


def convert(text):
    text = re.sub(r'\]\((?:docs/)?([\w-]+)\.md(#[\w-]+)?\)', lambda m: f"]({name(m.group(1)).replace(' ', '-')}{m.group(2) or ''})", text)
    text = re.sub(r'\]\((?:\.\./)?(notes|tools|src)/', rf'](https://github.com/{REPO}/blob/main/\1/', text)
    return re.sub(r'\]\((?:docs/)?(img/[^)]+)\)', lambda m: f']({RAW}{m.group(1)})', text)


def main():
    push = '--push' in sys.argv
    d = tempfile.mkdtemp()
    r = subprocess.run(['git', 'clone', '-q', f'https://github.com/{REPO}.wiki.git', d], capture_output=True, text=True)
    if r.returncode:
        sys.exit('no wiki repo yet: create a first page in the Wiki tab on GitHub, then run this again\n' + r.stderr)
    docs = os.path.join(ROOT, 'docs')
    stems = sorted(f[:-3] for f in os.listdir(docs) if f.endswith('.md'))
    for s in stems:
        out = os.path.join(d, name(s).replace(' ', '-') + '.md')
        open(out, 'w').write(convert(open(os.path.join(docs, s + '.md')).read()))
    link = lambda s: f"[{name(s)}]({name(s).replace(' ', '-')})"
    open(os.path.join(d, '_Sidebar.md'), 'w').write('**[Home](Home)**\n\n' + '\n\n'.join(
        f'**{g}**\n\n' + '\n'.join('* ' + link(s) for s in ss if s in stems) for g, ss in SIDEBAR) + '\n')
    start = convert(open(os.path.join(docs, 'start-here.md')).read()).split('\n', 1)[1]
    open(os.path.join(d, 'Home.md'), 'w').write(
        f'![ros2-jazzy-framework]({RAW}img/banner.svg)\n\n> **Speak a task. Get a validated behaviour tree. Watch the robot run it.**\n> '
        f'Overview, badges and quick start: [README](https://github.com/{REPO}#readme) · '
        f'[Source](https://github.com/{REPO}) · [Notes](https://github.com/{REPO}/tree/main/notes)\n\n'
        '*These pages are generated from `docs/` in the repository (`tools/wiki_sync.py`); edit them there, not here.*\n' + start)
    open(os.path.join(d, '_Footer.md'), 'w').write(
        f'[Repository](https://github.com/{REPO}) · [Actions](https://github.com/{REPO}/actions) · Generated from `docs/`\n')
    subprocess.run(['git', '-C', d, 'add', '-A'], check=True)
    print(subprocess.run(['git', '-C', d, 'status', '--short'], capture_output=True, text=True).stdout)
    if push:
        who = [subprocess.run(['git', 'config', k], capture_output=True, text=True, cwd=ROOT).stdout.strip()
               for k in ('user.name', 'user.email')]
        subprocess.run(['git', '-C', d, '-c', f'user.name={who[0]}', '-c', f'user.email={who[1]}',
                        'commit', '-qm', 'Sync from docs/'], check=True)
        subprocess.run(['git', '-C', d, 'push', '-q'], check=True)
        print('pushed')


if __name__ == '__main__':
    main()
