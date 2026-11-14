#!/usr/bin/env python3
"""Draw docs/img/slm-benchmark.png (+ -dark.png) from docs/slm-benchmark.json: held-out accuracy per fine-tune run and the v4 loss curve.

  python3 tools/slm/plot_benchmark.py
Colours: validated categorical slots 1-2 (blue = valid XML, orange = exact match) on the reference palette, light and dark steps.
Add a run to docs/slm-benchmark.json after each fine-tune and re-run; the numbers come from `finetune_bt_slm`'s eval line.
"""
import json
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import BoxStyle, FancyBboxPatch  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
THEMES = {
    'light': dict(surface='#fcfcfb', primary='#0b0b0b', secondary='#52514e', grid='#e4e3df', blue='#2a78d6', orange='#eb6834'),
    'dark': dict(surface='#1a1a19', primary='#f0efec', secondary='#c3c2b7', grid='#33332f', blue='#3987e5', orange='#d95926'),
}


def rounded_bar(ax, x, w, h, color, surface, px_x, px_y, r=4):
    """Bar from the baseline up to h with a 4 px rounded data end (bottom corners are hidden below the axis), 2 px surface gap."""
    rx, ry = r / px_x, r / px_y
    box = FancyBboxPatch((x, -0.1), w, h + 0.1, boxstyle=BoxStyle('Round', pad=0, rounding_size=rx), mutation_aspect=ry / rx,
                         facecolor=color, edgecolor=surface, linewidth=1.5, clip_on=True)
    ax.add_patch(box)


def draw(data, theme, out):
    c = THEMES[theme]
    plt.rcParams.update({'font.size': 10, 'text.color': c['primary'], 'axes.labelcolor': c['secondary'],
                         'xtick.color': c['secondary'], 'ytick.color': c['secondary']})
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11.5, 4.6), gridspec_kw={'width_ratios': [1.35, 1]}, facecolor=c['surface'])
    fig.subplots_adjust(left=0.06, right=0.985, top=0.80, bottom=0.2, wspace=0.22)
    runs = data['runs']
    n = len(runs)
    w = 0.36
    ax.set_xlim(-0.6, n - 0.4)
    ax.set_ylim(0, 1.1)
    bbox = ax.get_window_extent()
    px_x, px_y = bbox.width / (n + 0.2), bbox.height / 1.1
    for i, r in enumerate(runs):
        for j, (key, col) in enumerate((('valid', c['blue']), ('exact', c['orange']))):
            x = i + (j - 0.5) * (w + 0.03) - w / 2 + w / 2
            rounded_bar(ax, x - w / 2 + w / 2 - w / 2, w, r[key], col, c['surface'], px_x, px_y)
            ax.text(x, r[key] + 0.012, f'{r[key] * 100:.1f}%', ha='center', va='bottom', fontsize=8.5, color=c['primary'])
    ax.set_xticks(range(n))
    ax.set_xticklabels([f"{r['label']}\n{r['rows']:,} rows\n{r['catalog']}" for r in runs], fontsize=8.5)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(['0', '25%', '50%', '75%', '100%'])
    ax.grid(axis='y', color=c['grid'], linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)
    ax.spines['bottom'].set_color(c['grid'])
    ax.tick_params(length=0)
    ax.set_facecolor(c['surface'])
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor=c['blue'], label='valid XML'), Patch(facecolor=c['orange'], label='exact match')],
              loc='lower left', frameon=False, ncol=2, fontsize=9, bbox_to_anchor=(0.0, 1.0))
    ax.set_title('Held-out accuracy by training run', loc='left', fontsize=11.5, fontweight='bold', pad=26)
    pts = data['loss_v4']['points']
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    k = 5
    sm = [sum(ys[max(0, i - k + 1):i + 1]) / len(ys[max(0, i - k + 1):i + 1]) for i in range(len(ys))]
    bx.plot(xs, ys, color=c['grid'], linewidth=1.2, zorder=1)                      # raw: one example per logged step, very noisy
    bx.plot(xs, sm, color=c['secondary'], linewidth=2, solid_capstyle='round', zorder=2)
    bx.set_ylim(0, 0.05)
    bx.set_xlim(0, xs[-1] + 60)
    bx.set_xlabel('training step (one example per step)')
    bx.set_ylabel('loss')
    bx.grid(axis='y', color=c['grid'], linewidth=0.8)
    bx.set_axisbelow(True)
    for s_ in ('top', 'right', 'left'):
        bx.spines[s_].set_visible(False)
    bx.spines['bottom'].set_color(c['grid'])
    bx.tick_params(length=0)
    bx.set_facecolor(c['surface'])
    bx.set_title('Training loss, v4', loc='left', fontsize=11.5, fontweight='bold', pad=26)
    bx.text(0.0, 1.03, f'bold line: {k}-point average; faint line: raw, one example every 50 steps\n(raw peaks above 0.05 are cut off)',
            transform=bx.transAxes, fontsize=8, color=c['secondary'], va='bottom', linespacing=1.3)
    tail = sum(ys[-20:]) / 20
    bx.annotate(f'first logged step: {ys[0]:.3f}', (xs[0], min(ys[0], 0.05)), (xs[0] + 300, 0.044), fontsize=8.5, color=c['primary'],
                arrowprops=dict(arrowstyle='-', color=c['secondary'], lw=0.8))
    bx.text(0.98, 0.3, f'last 20 logged steps:\nmean loss {tail:.4f}', transform=bx.transAxes, ha='right', fontsize=8.5,
            color=c['primary'], linespacing=1.3)
    fig.text(0.06, 0.945, 'bt_slm: task spec -> BehaviorTree.CPP XML (230M, fine-tuned from LFM2.5-230M-Base)', fontsize=12.5,
             fontweight='bold', ha='left')
    fig.text(0.06, 0.03, 'Each run is scored on its own 200 held-out synthetic specs against a rule-based generator; catalogs differ, so runs are not strictly comparable. '
             'Not evaluated on real LLM output.', fontsize=8, color=c['secondary'], ha='left')
    fig.savefig(out, dpi=200, facecolor=c['surface'])
    plt.close(fig)


def main():
    data = json.load(open(os.path.join(ROOT, 'docs', 'slm-benchmark.json')))
    os.makedirs(os.path.join(ROOT, 'docs', 'img'), exist_ok=True)
    for theme, suffix in (('light', ''), ('dark', '-dark')):
        out = os.path.join(ROOT, 'docs', 'img', f'slm-benchmark{suffix}.png')
        draw(data, theme, out)
        print('wrote', os.path.relpath(out, ROOT))


if __name__ == '__main__':
    main()
