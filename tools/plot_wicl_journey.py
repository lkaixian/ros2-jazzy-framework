#!/usr/bin/env python3
"""docs/img/wicl-journey.png: every benchmark iteration of the WICL feature. Numbers are copied from docs/wicl.md
(each panel is a DIFFERENT test set: compare bars inside a panel, never across panels)."""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

INK, MUTED, GRID, BG = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
ARMS = [('No memory', '#8a8984'), ('One fixed example', '#2a78d6'), ('Memory', '#eb6834'), ('Memory + lessons', '#1baf7a'),
        ('Lessons only (no memory)', '#1baf7a')]
HATCH = {4: '////'}

fig = plt.figure(figsize=(14, 10.2), facecolor=BG)
gs = fig.add_gridspec(2, 3, height_ratios=[0.95, 1], hspace=0.50, wspace=0.22, left=0.055, right=0.985, top=0.80, bottom=0.14)


def panel(ax, title, groups, series, label_arms=(0, 3), width=0.8, note=None):
    """groups: x labels; series: {arm index: [values %]}"""
    n = len(series)
    w = width / n
    for k, (arm, vals) in enumerate(series.items()):
        xs = [i - width / 2 + w * (k + 0.5) for i in range(len(groups))]
        ax.bar(xs, vals, w * 0.86, color=ARMS[arm][1], zorder=3, linewidth=0, hatch=HATCH.get(arm),
               edgecolor='#fcfcfb' if arm in HATCH else None)
        if arm in label_arms:
            for x, v in zip(xs, vals):
                ax.text(x, v + 1.8, f'{v:g}', ha='center', va='bottom', fontsize=8, color=INK)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels(groups, fontsize=8.5, color=INK)
    ax.set_ylim(0, 112)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_yticklabels(['0', '25', '50', '75', '100%'], fontsize=8, color=MUTED)
    ax.grid(axis='y', color=GRID, linewidth=0.8, zorder=0)
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)
    ax.spines['bottom'].set_color(GRID)
    ax.tick_params(length=0)
    ax.set_title(title, loc='left', fontsize=10.5, color=INK, fontweight='bold', pad=8)
    if note:
        ax.text(0, -0.22, note, transform=ax.transAxes, fontsize=8.5, color=MUTED, va='top', clip_on=False)


# A: the four development iterations (12 tasks, bare prompt, 1 run each)
ax = fig.add_subplot(gs[0, :])
stages = ['1  first run\n(1 of 12 memories learned:\n600-token cap, generic lesson)',
          '2  rerun, same result\n(stale installed code:\nfixes never ran)',
          '3  fixes in, still weak\n(model\'s vague lesson\nreplaced the concrete one)',
          '4  concrete lesson kept\n(12 of 12 memories learned)']
panel(ax, 'A.  Development iterations: 12 tasks, bare prompt, success rate', stages,
      {0: [33, 33, 50, 58], 2: [25, 25, 50, 92], 3: [0, 8, 25, 92]}, label_arms=(0, 2, 3), width=0.62)
ax.set_ylim(0, 118)
ax.annotate('The fix was in my code, not the idea:\nmemory + lessons went from 0% to 92%', xy=(3.2, 92),
            xytext=(1.55, 104), fontsize=9, color=INK, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8), va='center')

# B: bare prompt, held-out tasks, 5 repeats, with the fixed-example control
ax = fig.add_subplot(gs[1, 0])
panel(ax, 'B.  Bare prompt, 8 new tasks (5 runs)', ['new goal\ncompositions'],
      {0: [35], 1: [100], 2: [92.5], 3: [100]}, label_arms=(0, 1, 2, 3),
      note='One fixed unrelated example already gives 100%:\nthe gain was prompt priming, not memory.')

# C: production prompt, 5 repeats
ax = fig.add_subplot(gs[1, 1])
panel(ax, 'C.  Production prompt (5 runs)', ['reuse\n(12)', 'new tasks\n(8)', 'unfamiliar names\n(8)'],
      {0: [100, 100, 75], 1: [100, 100, 52.5], 2: [100, 100, 100], 3: [100, 100, 100]}, label_arms=(),
      note='At the ceiling except unfamiliar names, the one case\nwhere memory helped and one generic example hurt.')
ax.text(2, 106, '75 → 100', ha='center', fontsize=8.5, color=INK)

# D: three independent AI writers, first repeat only
ax = fig.add_subplot(gs[1, 2])
panel(ax, 'D.  Three AI-written test sets (2 to 5 runs)', ['Gemini', 'ChatGPT', 'Claude'],
      {0: [64, 49, 69], 1: [76, 62, 66], 2: [65, 57, 80], 4: [74, 67, 78], 3: [80, 75, 88]}, label_arms=(0, 4, 3),
      note='30 tasks each (1 task = 3.3 points). Lessons alone\n(hatched) give most of the gain; memory adds a\nsmaller extra on top.')

handles = [plt.Rectangle((0, 0), 1, 1, facecolor=c, hatch=HATCH.get(i), edgecolor='#fcfcfb') for i, (_, c) in enumerate(ARMS)]
fig.legend(handles, [a for a, _ in ARMS], loc='upper left', bbox_to_anchor=(0.05, 0.855), ncol=5, frameon=False, fontsize=10)
fig.text(0.055, 0.955, 'WICL task memory: how the result changed, step by step', fontsize=15, fontweight='bold', color=INK)
fig.text(0.055, 0.925, 'Planner: Gemma 4 E2B (2B) in a 2D kinematic nav sim on the sample map. Sim only: nothing here ran on the robot.\n'
         'Each panel is a different test set: compare bars inside a panel, not across panels.', fontsize=9.5, color=MUTED, va='top')
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img', 'wicl-journey.png')
fig.savefig(out, dpi=150, facecolor=BG)
print(out)
