"""Compare PDF figures with actual rerun data in the paper's plot conventions."""
import subprocess
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run-dir', type=Path, default=ROOT / 'build/results/paper_2609_14208_reproduction')
parser.add_argument('--cache-fixed', action='store_true', help='Label a complete cache-fixed IR rerun')
args = parser.parse_args()
RUN = args.run_dir.resolve()
OUT = RUN / 'paper_comparison'
OUT.mkdir(exist_ok=True)
df = pd.read_csv(RUN / 'rows.csv', dtype={'scenario': str})
paired = pd.read_csv(RUN / 'paired_comparisons.csv', dtype={'scenario': str})
maps = ['random-32-32-10', 'symbotic', 'den312d', 'maze-32-32-2',
        'room-64-64-8', 'warehouse-10-20-10-2-1', 'orz900d', 'Boston_0_256']
titles = ['random-32-32-10', 'Symbotic', 'den312d', 'maze-32-32-2',
          'room', 'warehouse', 'orz900d', 'Boston']
methods = {'lacam_dfs': ('ITA-LaCAM', '#ff6666'),
           'ir': ('IR-TAPF', '#2ca02c'), 'itacbs': ('ITA-CBS', '#3498db')}
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .25})

def save(fig, stem):
    for ext in ['png', 'pdf']:
        fig.savefig(OUT / f'{stem}.{ext}', dpi=220, bbox_inches='tight')
    plt.close(fig)

for number, suite in [(2, 'group'), (3, 'common')]:
    fig, axes = plt.subplots(2, 4, figsize=(12, 6), sharey=True)
    for ax, name, title in zip(axes.flat, maps, titles):
        for method, (label, color) in methods.items():
            scenarios = [('G', '-')] if suite == 'group' else [('000', '-'), ('030', '--'), ('060', ':'), ('100', '-.')]
            for scenario, style in scenarios:
                g = df[(df['map'] == name) & (df.suite == suite) & (df.method == method) & (df.scenario == scenario)]
                y = g.groupby('num_agents').valid_solution.mean()
                ax.plot(y.index, y, color=color, linestyle=style,
                        marker={'lacam_dfs': 'v', 'ir': 'D', 'itacbs': 'o'}[method], markersize=3, linewidth=1)
        ax.set(title=title, xlabel='# agents', ylim=(0, 1.03), yticks=[0, .5, 1])
        if suite == 'group':
            ax.set(xlim=(10, 200), xticks=[10, 80, 200])
        elif name == 'maze-32-32-2':
            ax.set(xlim=(5, 60), xticks=[5, 20, 35, 60])
        else:
            ax.set(xlim=(15, 60), xticks=[15, 30, 45, 60])
        if ax in axes[:, 0]: ax.set_ylabel('Success rate')
    handles = [Line2D([], [], color=c, label=l) for l, c in methods.values()]
    if number == 3:
        handles += [Line2D([], [], color='black', linestyle=s, label=l) for l, s in [('0%', '-'), ('30%', '--'), ('60%', ':'), ('100%', '-.')]]
    fig.legend(handles=handles, loc='upper center', ncol=len(handles), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, .94))
    save(fig, f'rerun_figure{number}')

x = paired.ir_initial_solution_time_ms.where(paired.ir_valid_solution == 1, 10000).fillna(10000).clip(lower=.01) / 1000
y = paired.lacam_first_ms.where(paired.lacam_valid == 1, 10000).fillna(10000).clip(lower=.01) / 1000
fig, ax = plt.subplots(figsize=(5, 4.5))
ax.scatter(x, y, color='#ff6666', s=3, alpha=.3)
ax.plot([1e-3, 10], [1e-3, 10], 'k--', linewidth=1)
ax.set(xscale='log', yscale='log', xlim=(1e-3, 10), ylim=(1e-3, 10),
       xlabel='IR-TAPF time to first solution (s)', ylabel='ITA-LaCAM time to first solution (s)')
under = int(((x < 1e-3) | (y < 1e-3)).sum())
ax.text(.02, .98, f'{under} points below the paper axis range\nare retained in the source CSV',
        transform=ax.transAxes, va='top', fontsize=8)
save(fig, 'rerun_figure4')

for number, method in [(5, 'ir'), (6, 'itacbs')]:
    g = paired[(paired.lacam_valid == 1) & (paired[f'{method}_valid_solution'] == 1)].copy()
    x = g[f'{method}_soc'].astype(float)
    ratio = x / g.lacam_soc if method == 'ir' else g.lacam_soc / x
    fig, ax = plt.subplots(figsize=(5, 4.5))
    ax.scatter(x, ratio, color='#ff6666', s=3, alpha=.3)
    bins = pd.cut(np.log10(x.clip(lower=1)), 20)
    grouped = pd.DataFrame({'x': x, 'ratio': ratio, 'bin': bins}).groupby('bin', observed=True)
    med = grouped.median(numeric_only=True)
    lower = grouped.ratio.quantile(.1)
    upper = grouped.ratio.quantile(.9)
    ax.fill_between(med.x.to_numpy(), lower.to_numpy(), upper.to_numpy(), color='#ff6666', alpha=.12)
    ax.plot(med.x, med.ratio, color='#b22222', label='median', linewidth=1.3)
    ax.axhline(1, color='black', linestyle='--', linewidth=.8)
    ax.set(xscale='log', xlabel=f'{methods[method][0]} solution cost (SOC)',
           ylabel='IR-TAPF SOC / ITA-LaCAM SOC' if method == 'ir' else 'ITA-LaCAM SOC / ITA-CBS SOC')
    bounds = (.82, 1.5) if method == 'ir' else (.95, 1.3)
    ax.set_ylim(*bounds)
    outside = int(((ratio < bounds[0]) | (ratio > bounds[1])).sum())
    ax.text(.98, .98, f'n={len(g):,}; median={ratio.median():.4f}\n{outside} points outside displayed y range',
            transform=ax.transAxes, ha='right', va='top', fontsize=8)
    ax.legend(loc='upper left', frameon=False)
    save(fig, f'rerun_figure{number}')

# PDF points, measured from original pages; no reconstructed paper data points.
crops = {2: (5, 100, 52, 410, 226), 3: (6, 100, 52, 410, 226),
         4: (6, 338, 339, 190, 159), 5: (7, 50, 45, 220, 172),
         6: (7, 50, 270, 220, 178)}
for number, (page, x0, y0, width, height) in crops.items():
    scale = 240 / 72
    prefix = OUT / f'paper_figure{number}'
    subprocess.run(['pdftoppm', '-f', str(page), '-l', str(page), '-singlefile', '-r', '240',
                    '-x', str(round(x0 * scale)), '-y', str(round(y0 * scale)),
                    '-W', str(round(width * scale)), '-H', str(round(height * scale)),
                    '-png', str(RUN / 'paper.pdf'), str(prefix)], check=True)
    fig, axes = plt.subplots(1, 2, figsize=(16, 6 if number in (2, 3) else 7))
    for ax, filename, label in zip(axes, [prefix.with_suffix('.png'), OUT / f'rerun_figure{number}.png'],
                                   ['Paper: original PDF figure', 'IR cache fixed; LaCAM/CBS retained' if args.cache_fixed else 'Rerun: after timeout review']):
        ax.imshow(Image.open(filename)); ax.axis('off'); ax.set_title(label, fontsize=15, pad=12)
    fig.suptitle(f'Figure {number}: paper vs rerun', fontsize=18)
    fig.text(.5, .025, 'Different instances and hardware; all IR cases rerun once with cache fix, 10 s budget.' if args.cache_fixed else 'Different instances and hardware; failed cases rerun with the same 10 s budget. Baseline retained.', ha='center', fontsize=11)
    fig.tight_layout(rect=(0, .06, 1, .94))
    save(fig, f'comparison_figure{number}')

metrics = []
for suite, paper_values in [('group', (100, 94.9, 15.9)), ('common', (100, 96.0, 67.8))]:
    for method, paper in zip(methods, paper_values):
        value = 100 * df[(df.suite == suite) & (df.method == method)].valid_solution.mean()
        metrics.append((f'{suite}: {methods[method][0]}', paper, value))
times = paired
tx = times.ir_initial_solution_time_ms.where(times.ir_valid_solution == 1, 10000).fillna(10000)
ty = times.lacam_first_ms.where(times.lacam_valid == 1, 10000).fillna(10000)
joint = paired[(paired.lacam_valid == 1) & (paired.ir_valid_solution == 1)]
metrics += [('ITA-LaCAM faster first solution', 84.0, 100 * (ty < tx).mean()),
            ('ITA-LaCAM lower SOC vs IR-TAPF', 65.0, 100 * (joint.lacam_soc < joint.ir_soc).mean())]
table = pd.DataFrame(metrics, columns=['metric', 'paper_percent', 'rerun_percent'])
table['difference_percentage_points'] = table.rerun_percent - table.paper_percent
table.to_csv(OUT / 'paper_vs_rerun_metrics.csv', index=False)
fig, ax = plt.subplots(figsize=(12, 6))
idx = np.arange(len(table)); ax.barh(idx - .18, table.paper_percent, height=.36, label='Paper', color='#78909c')
ax.barh(idx + .18, table.rerun_percent, height=.36, label='Rerun', color='#d62728')
ax.set(yticks=idx, yticklabels=table.metric, xlabel='Percent (%)', xlim=(0, 112)); ax.invert_yaxis(); ax.legend()
for i, row in table.iterrows():
    ax.text(row.paper_percent + .8, i - .18, f'{row.paper_percent:.1f}', va='center', fontsize=9)
    ax.text(row.rerun_percent + .8, i + .18, f'{row.rerun_percent:.1f}', va='center', fontsize=9)
fig.suptitle('Paper vs rerun: reported aggregate metrics', fontsize=15); fig.tight_layout()
save(fig, 'comparison_summary')
(OUT / 'README.txt').write_text('Left panels are cropped original PDF figures, not digitized curves. Right panels use all current rerun records after authorized timeout review.\nFigure 4 uses the paper axes; sub-millisecond points are counted in its annotation.\nFigures 5 and 6 use the original paper ratio directions and show out-of-range counts.\nMedian curves use 20 equal-width logarithmic SOC bins; shading shows within-bin 10th--90th percentiles. The paper does not specify exact bin boundaries or shading quantiles.\nOriginal instances were lost and regenerated; hardware differs. Baseline results are preserved under before_timeout_review.\nPaper source: https://arxiv.org/abs/2609.14208\n')
print(table.to_string(index=False))
print('Saved comparisons to', OUT)
