"""Wait for full audited results, then plot comparisons and archive this run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import time


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def finalize(run, dataset):
    os.sched_setaffinity(0, {120})
    main_pid = int((run / 'run.pid').read_text())
    while True:
        status = json.loads((run / 'status.json').read_text())
        report_pid = status.get('report_pid')
        if not alive(main_pid) and not (report_pid and alive(report_pid)):
            break
        time.sleep(5)
    audit = json.loads((run / 'coverage_audit.json').read_text())
    if status['state'] != 'complete' or not audit['complete'] or audit['actual_rows'] != 29280:
        raise RuntimeError('Full audited experiment is incomplete; refusing final comparisons')
    if (run / 'timeout_review_plan.json').exists():
        plan = json.loads((run / 'timeout_review_plan.json').read_text())
        reviewed = []
        for item in plan['pending']:
            previous = json.loads((run / item['previous_record']).read_text())
            current_path = run / 'artifacts' / item['case_id'] / item['method'] / 'result.json'
            current = json.loads(current_path.read_text())
            assert previous['fixture_sha256'] == current['fixture_sha256']
            assert previous['binary_sha256'] == current['binary_sha256']
            assert previous['time_limit'] == current['time_limit'] == 10
            reviewed.append({'case_id': item['case_id'], 'method': item['method'],
                             'previous_solved': previous['valid_solution'],
                             'review_solved': current['valid_solution'],
                             'exit_code': current['exit_code'], 'timed_out': current['timed_out'],
                             'previous_record': item['previous_record'],
                             'review_record': str(current_path.relative_to(run))})
        by_method = {}
        for method in {item['method'] for item in reviewed}:
            subset = [item for item in reviewed if item['method'] == method]
            by_method[method] = {'reviewed': len(subset),
                                 'newly_solved': sum(item['review_solved'] for item in subset),
                                 'still_unsolved': sum(not item['review_solved'] for item in subset)}
        (run / 'timeout_review_summary.json').write_text(json.dumps(
            {'complete': len(reviewed) == plan['pending_total'], 'by_method': by_method,
             'merged_common_lacam': plan['merged_common_lacam'], 'records': reviewed}, indent=2))
    subprocess.run([sys.executable, str(Path(__file__).with_name('plot_paper_2609_comparison.py')),
                    '--run-dir', str(run)], check=True)
    subprocess.run([sys.executable, str(Path(__file__).with_name('build_paper_comparison_page.py')),
                    '--run-dir', str(run)], check=True)
    page = run / 'index.html'
    html = page.read_text()
    gallery = '<section><h2>论文对比 · 全量结果</h2><p><a href="comparison.html">打开逐图对比网页</a> · <a href="paper_comparison/paper_vs_rerun_metrics.csv">指标对比 CSV</a></p>'
    for number in range(2, 7):
        target = f'paper_comparison/comparison_figure{number}.png'
        gallery += f'<a href="{target}"><img src="{target}" style="width:100%" alt="论文图 {number} 与重跑结果对比"></a>'
    html = html.replace('</main>', gallery + '</section></main>')
    page.write_text(html)
    backup_paths = []
    for folder in (dataset, run):
        target = folder.parent / (folder.name + '.tar')
        if target.exists():
            raise FileExistsError(f'Preserving existing archive: {target}')
        temporary = target.with_suffix('.tar.tmp')
        with tarfile.open(temporary, 'w') as archive:
            archive.add(folder, arcname=folder.name)
        temporary.replace(target)
        digest = hashlib.file_digest(target.open('rb'), 'sha256').hexdigest()
        target.with_suffix('.tar.sha256').write_text(f'{digest}  {target.name}\n')
        backup_paths.append(str(target))
    (run / 'delivery.json').write_text(json.dumps({'state': 'complete', 'archives': backup_paths,
                                                  'coverage_audit_complete': True}, indent=2))
    print('Final comparisons and archives complete', backup_paths, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--dataset-dir', type=Path, required=True)
    args = parser.parse_args()
    finalize(args.run_dir.resolve(), args.dataset_dir.resolve())
