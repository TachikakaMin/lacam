#!/usr/bin/env python3
"""Audit coverage and regenerate success-rate, runtime, and SOC figures."""
import argparse
import concurrent.futures
import multiprocessing
import os
import json
import gzip
import hashlib
import tempfile
import time
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from run_paper_2609_14208 import atomic_json, validate

LABELS = {'lacam_dfs': 'ITA-LaCAM', 'ir': 'IR-TAPF (DBS-Hungarian)', 'itacbs': 'ITA-CBS'}
COLORS = {'lacam_dfs': '#d62728', 'ir': '#2ca02c', 'itacbs': '#1f77b4'}

def verify_archived_schedule(task):
    run, row, cpu = task
    folder = run / 'artifacts' / row['case_id'] / row['method']
    cache = run / 'archive_audit_cache' / row['case_id'] / row['method'] / 'validation.json'
    try:
        hashes = {name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                  for name in ('schedule.yaml.gz', 'schedule.yaml.bin.gz') if (folder / name).exists()}
        signature = {'archives': hashes, 'fixture_sha256': row['fixture_sha256'],
                     'soc': row['soc'], 'makespan': row['makespan'], 'validator_version': 'isolated_safe_loader_v1'}
        if cache.exists():
            saved = json.loads(cache.read_text())
            if saved.get('signature') == signature and saved.get('passed'):
                return None
        with tempfile.TemporaryDirectory() as temp:
            schedule = Path(temp) / 'schedule.yaml'
            for filename in ('schedule.yaml', 'schedule.yaml.bin'):
                archive = folder / (filename + '.gz')
                if archive.exists():
                    (Path(temp) / filename).write_bytes(gzip.decompress(archive.read_bytes()))
            argv = ['taskset', '-c', str(cpu), sys.executable, '-X', 'faulthandler',
                    str(Path(__file__).with_name('recover_validation_pool.py')),
                    'validate', '--fixture', row['fixture_file'], '--schedule', str(schedule)]
            attempts = []
            for attempt in range(2):
                process = subprocess.run(argv, capture_output=True, text=True, timeout=600,
                                         env={**os.environ, 'OMP_NUM_THREADS': '1',
                                              'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'})
                attempts.append({'exit_code': process.returncode, 'stderr': process.stderr[-12000:]})
                if process.returncode >= 0:
                    break
            cache.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(cache, {'signature': signature, 'passed': False, 'attempts': attempts})
            if process.returncode:
                raise RuntimeError(f'isolated validator exit {process.returncode}: {process.stderr[-1500:]}')
            metrics = json.loads(process.stdout)
            soc, makespan = metrics['soc'], metrics['makespan']
            if soc != row['soc'] or makespan != row['makespan']:
                raise ValueError('archive metrics disagree with recorded result')
        atomic_json(cache, {'signature': signature, 'passed': True, 'attempts': attempts})
        return None
    except Exception as exc:
        return {'case_id': row['case_id'], 'method': row['method'], 'error': repr(exc)}

def report(run, manifest_path, audit_jobs=8):
    records = [json.loads(p.read_text()) for p in (run / 'artifacts').glob('*/*/*/result.json')]
    df = pd.DataFrame(records)
    for column in ('soc', 'first_solution_time_ms', 'initial_solution_time_ms'):
        if column in df:
            df[column] = pd.to_numeric(df[column], errors='coerce')
    manifest = json.loads(manifest_path.read_text())
    expected = {(c['case_id'], m) for c in manifest['cases'] for m in LABELS}
    actual = {(r['case_id'], r['method']) for r in records}
    errors = [r for r in records if 'infrastructure_error' in r]
    audit = {'expected_rows': len(expected), 'actual_rows': len(records),
             'missing_rows': len(expected - actual), 'unexpected_rows': len(actual - expected),
             'duplicate_rows': len(records) - len(actual), 'infrastructure_errors': len(errors),
             'complete': actual == expected and len(records) == len(expected) and not errors}
    # On a full run, re-open every compressed successful incumbent and check it
    # again, rather than accepting the recorded valid_solution flag as proof.
    audit['archived_solutions_checked'] = 0
    audit['archive_validation_errors'] = []
    audit['input_hash_errors'] = []
    for case in manifest['cases']:
        path = Path(case['fixture_file'])
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != case['fixture_sha256']:
            audit['input_hash_errors'].append(case['case_id'])
    audit['complete'] = audit['complete'] and not audit['input_hash_errors']
    provenance = json.loads((run / 'provenance.json').read_text())
    reserved = set(provenance['reserved_cpus'])
    audit['reserved_cpus'] = sorted(reserved)
    audit['resource_or_binary_errors'] = []
    for record in records:
        folder = run / 'artifacts' / record['case_id'] / record['method']
        command = json.loads((folder / 'command.json').read_text())['argv']
        if command[:2] != ['taskset', '-c'] or int(command[2]) in reserved:
            audit['resource_or_binary_errors'].append(record['case_id'] + ': CPU affinity')
        if record.get('binary_sha256') != provenance['binaries_sha256'][record['method']]:
            audit['resource_or_binary_errors'].append(record['case_id'] + ': binary hash')
    audit['complete'] = audit['complete'] and len(reserved) >= 4 and not audit['resource_or_binary_errors']
    if audit['complete']:
        successful = [row for row in records if row['valid_solution']]
        cpus = sorted(set(os.sched_getaffinity(0)) - reserved)
        if not cpus:
            raise ValueError('no CPU available outside the reserved set')
        jobs = max(1, min(audit_jobs, len(cpus)))
        status_path = run / 'status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        status.update(state='auditing', report_pid=os.getpid(), archive_validation_total=len(successful),
                      archive_validation_completed=0, archive_validation_jobs=jobs)
        atomic_json(status_path, status)
        print(f'Independent archive audit: 0/{len(successful)}, jobs={jobs}', flush=True)
        last_update = time.monotonic()
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as executor:
            futures = [executor.submit(verify_archived_schedule, (run, row, cpus[i % jobs]))
                       for i, row in enumerate(successful)]
            for done, future in enumerate(concurrent.futures.as_completed(futures), 1):
                error = future.result()
                if error is None:
                    audit['archived_solutions_checked'] += 1
                else:
                    audit['archive_validation_errors'].append(error)
                if time.monotonic() - last_update >= 5 or done == len(successful):
                    status.update(archive_validation_completed=done, updated_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
                    atomic_json(status_path, status)
                    print(f'Independent archive audit: {done}/{len(successful)}', flush=True)
                    last_update = time.monotonic()
        audit['complete'] = not audit['archive_validation_errors']
    (run / 'coverage_audit.json').write_text(json.dumps(audit, indent=2))
    if not records:
        raise ValueError('no result artifacts')
    figures = run / 'figures'
    figures.mkdir(exist_ok=True)
    maps = list(dict.fromkeys(c['map'] for c in manifest['cases']))
    summary = df.groupby(['suite', 'map', 'scenario', 'num_agents', 'method']).agg(
        completed=('case_id', 'size'), solved=('valid_solution', 'sum'),
        success_rate=('valid_solution', 'mean')).reset_index()
    summary.to_csv(run / 'success_rates.csv', index=False)

    def save(fig, name):
        fig.tight_layout()
        for extension in ('png', 'pdf'):
            fig.savefig(figures / f'{name}.{extension}', dpi=200, bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(2, 4, figsize=(15, 6), sharey=True)
    for ax, map_name in zip(axes.flat, maps):
        for method in LABELS:
            group = summary[(summary.suite == 'group') & (summary['map'] == map_name) & (summary.method == method)].sort_values('num_agents')
            ax.plot(group.num_agents, group.success_rate * 100, label=LABELS[method], color=COLORS[method])
        ax.set(title=map_name, xlabel='Agents', ylabel='Success (%)', ylim=(-2, 102))
        ax.grid(alpha=.25)
    axes.flat[0].legend(fontsize=7)
    save(fig, 'figure2_group_success_rates')

    fig, axes = plt.subplots(4, 8, figsize=(24, 10), sharey=True)
    for row, scenario in enumerate(('000', '030', '060', '100')):
        for col, map_name in enumerate(maps):
            ax = axes[row, col]
            for method in LABELS:
                group = summary[(summary.suite == 'common') & (summary['map'] == map_name) & (summary.scenario == scenario) & (summary.method == method)].sort_values('num_agents')
                ax.plot(group.num_agents, group.success_rate * 100, color=COLORS[method], label=LABELS[method])
            ax.set(title=f'{map_name}: {int(scenario)}% shared', xlabel='Agents', ylim=(-2, 102))
            if col == 0:
                ax.set_ylabel('Success (%)')
            ax.grid(alpha=.25)
    axes[0, 0].legend(fontsize=6)
    save(fig, 'figure3_common_target_success_rates')

    indexed = {m: df[df.method == m].set_index('case_id') for m in LABELS}
    paired = indexed['lacam_dfs'][['valid_solution', 'soc', 'first_solution_time_ms', 'suite', 'scenario']].rename(columns={'valid_solution': 'lacam_valid', 'soc': 'lacam_soc', 'first_solution_time_ms': 'lacam_first_ms'})
    for method in ('ir', 'itacbs'):
        reference = indexed[method]
        columns = ['valid_solution', 'soc']
        if method == 'ir':
            columns.append('initial_solution_time_ms')
        paired = paired.join(reference[columns].rename(columns={c: f'{method}_{c}' for c in columns}))
    paired.to_csv(run / 'paired_comparisons.csv')
    contradictions = paired[(paired.lacam_valid == 1) & (paired.itacbs_valid_solution == 1) & (paired.lacam_soc < paired.itacbs_soc)]
    audit['optimal_baseline_contradictions'] = contradictions.index.tolist()
    audit['complete'] = audit['complete'] and contradictions.empty
    (run / 'coverage_audit.json').write_text(json.dumps(audit, indent=2))
    times = paired.copy()
    x = times.ir_initial_solution_time_ms.where(times.ir_valid_solution == 1, 10000).fillna(10000).clip(lower=.01) / 1000
    y = times.lacam_first_ms.where(times.lacam_valid == 1, 10000).fillna(10000).clip(lower=.01) / 1000
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(x, y, s=5, alpha=.25)
    ax.plot([1e-5, 10], [1e-5, 10], 'k--', linewidth=1)
    ax.set(xscale='log', yscale='log', xlabel='IR-TAPF first solution (s)', ylabel='ITA-LaCAM first solution (s)')
    save(fig, 'figure4_first_solution_times')
    comparisons = {}
    for method, figure in (('ir', 5), ('itacbs', 6)):
        group = paired[(paired.lacam_valid == 1) & (paired[f'{method}_valid_solution'] == 1)].copy()
        reference_soc = group[f'{method}_soc'].astype(float)
        lacam_soc = group.lacam_soc.astype(float)
        # Reference/ITA ratio > 1 favors ITA-LaCAM; use the same orientation
        # for medians as the paper's reported ratios.
        ratio = reference_soc / lacam_soc.clip(lower=1)
        comparisons[method] = {'paired': len(group), 'lacam_lower_fraction': float((lacam_soc < reference_soc).mean()),
                               'reference_lower_fraction': float((reference_soc < lacam_soc).mean()),
                               'ties_fraction': float((reference_soc == lacam_soc).mean()),
                               'median_reference_over_lacam': float(ratio.median()),
                               'p90_lacam_over_reference': float((lacam_soc / reference_soc.clip(lower=1)).quantile(.9))}
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.scatter(reference_soc, lacam_soc, s=5, alpha=.25)
        if len(group):
            maximum = max(reference_soc.max(), lacam_soc.max())
            ax.plot([1, maximum], [1, maximum], 'k--', linewidth=1)
            if len(group) >= 20:
                bins = pd.qcut(reference_soc, min(20, len(group)), duplicates='drop')
                binned = pd.DataFrame({'x': reference_soc, 'ratio': ratio, 'bin': bins}).groupby('bin', observed=True).median(numeric_only=True)
                right = ax.twinx()
                right.plot(binned.x, binned.ratio, color='#ff7f0e', linewidth=2)
                right.set_ylabel('Median reference SOC / ITA-LaCAM SOC')
            ax.set(xscale='log', yscale='log')
        ax.set(xlabel=f'{LABELS[method]} final-path SOC', ylabel='ITA-LaCAM final-path SOC')
        save(fig, f'figure{figure}_paired_soc_{method}')
    comparisons['first_solution'] = {'comparisons': len(times), 'lacam_faster_fraction': float((y < x).mean()),
                                     'ir_faster_fraction': float((x < y).mean()), 'ties_fraction': float((x == y).mean())}
    (run / 'comparison_metrics.json').write_text(json.dumps(comparisons, indent=2))
    text = [f"Coverage: {len(records)}/{len(expected)} runs; audit complete: {audit['complete']}.",
            'Original lost instance files were not recovered. Solver seeds are test indices 0–19.',
            'Generation policy: ' + json.dumps(manifest.get('generation_policy', manifest.get('generator', 'unspecified'))),
            'All successful runs were independently checked against map, starts, moves, collisions, eligible and unique final targets.',
            'SOC was recomputed from actual returned incumbent paths. Search cost is retained separately in raw results.',
            'Hardware differs from the paper; timings and success rates may therefore differ.',
            'Solve budget is 10 seconds; map/heuristic initialization and artifact writing are recorded in wall_time_s separately.',
            '', json.dumps(comparisons, indent=2)]
    (run / 'report.txt').write_text('\n'.join(text))
    if len(records) == len(expected):
        status_path = run / 'status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        status.update(state='complete' if audit['complete'] else 'needs_review',
                      audit_complete=audit['complete'], report_finished_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        status_path.write_text(json.dumps(status, indent=2))
    print(json.dumps(audit))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, default=Path('data/paper_2609_14208/manifest.json'))
    parser.add_argument('--audit-jobs', type=int, default=32)
    args = parser.parse_args()
    report(args.run_dir.resolve(), args.manifest, args.audit_jobs)
