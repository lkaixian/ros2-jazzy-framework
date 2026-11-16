#!/usr/bin/env python3
"""docs/img/wicl-summary.png: average success of each arm over the three AI-written test sets (numbers from docs/wicl.md)."""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

INK, MUTED, GRID, BG = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
# per-writer success (%), docs/wicl.md table: Gemini, ChatGPT, Claude
ROWS = [('No memory', '#8a8984', None, (64, 49, 69)),
        ('One fixed example', '#2a78d6', None, (76, 62, 66)),
        ('Memory only', '#eb6834', None, (65, 57, 80)),
        ('Lessons only (no memory)', '#1baf7a', '////', (74, 67, 78)),
        ('Memory + lessons', '#1baf7a', None, (80, 75, 88))]
avg = [sum(r[3]) / 3 for r in ROWS]
base = avg[0]

fig, ax = plt.subplots(figsize=(10, 5.4), facecolor=BG)
ax.set_facecolor(BG)
ys = list(range(len(ROWS)))[::-1]
for y, (name, col, hatch, vals), a in zip(ys, ROWS, avg):
    ax.barh(y, a, height=0.56, color=col, hatch=hatch, edgecolor=BG, linewidth=0, zorder=3)
    gain = '' if name == 'No memory' else f'   {a - base:+.0f} vs no memory'
    ax.text(a + 1.2, y, f'{a:.0f}%', va='center', fontsize=12, fontweight='bold', color=INK)
    if gain:
        ax.text(a + 8.2, y, gain.strip(), va='center', fontsize=9.5, color=MUTED)
ax.set_yticks(ys)
ax.set_yticklabels([r[0] for r in ROWS], fontsize=11, color=INK)
ax.set_xlim(0, 112)
ax.set_xticks([0, 25, 50, 75, 100])
ax.set_xticklabels(['0', '25', '50', '75', '100%'], fontsize=9, color=MUTED)
ax.grid(axis='x', color=GRID, linewidth=0.8, zorder=0)
for s in ('top', 'right', 'left'):
    ax.spines[s].set_visible(False)
ax.spines['bottom'].set_color(GRID)
ax.tick_params(length=0)
fig.text(0.04, 0.95, 'Lessons help most; memory adds a smaller extra on top', fontsize=14.5, fontweight='bold',
         color=INK, va='top')
fig.text(0.04, 0.885, 'Average success over three AI-written test sets (30 tasks each). Planner: Gemma 4 E2B (2B), 2D nav sim.\n'
         'A single run moves 4 to 10 points, so smaller gaps are not meaningful. Sim only: nothing here ran on the robot.',
         fontsize=9, color=MUTED, va='top')
plt.subplots_adjust(left=0.24, right=0.97, top=0.76, bottom=0.1)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img', 'wicl-summary.png')
fig.savefig(out, dpi=150, facecolor=BG)
print(out)
