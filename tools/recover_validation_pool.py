"""Recover stored solver outputs with one isolated validator per record."""
import argparse
import concurrent.futures
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from run_paper_2609_14208 import atomic_json, export, validate
import yaml


def recover(item):
    index, run, path = item
    row = json.loads(path.read_text())
    folder = path.parent
    argv = ['taskset', '-c', str(index % 24), sys.executable, '-X', 'faulthandler',
            str(Path(__file__).resolve()), 'validate', '--fixture', row['fixture_file'],
            '--schedule', str(folder / 'schedule.yaml')]
    assert row['exit_code'] == 0
    assert hashlib.sha256(Path(row['fixture_file']).read_bytes()).hexdigest() == row['fixture_sha256']
    proc = subprocess.run(argv, capture_output=True, text=True, timeout=180,
                          env={**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'})
    if proc.returncode:
        atomic_json(folder / 'isolated_validation_failure.json',
                    {'argv': argv, 'exit_code': proc.returncode, 'stderr': proc.stderr[-12000:]})
        return {'case_id': row['case_id'], 'method': row['method'], 'recovered': False}
    metrics = json.loads(proc.stdout)
    soc, makespan = metrics['soc'], metrics['makespan']
    if row['method'] == 'ir':
        assert row['soc'] == soc
    if row['method'] == 'itacbs':
        stats = yaml.load((folder / 'schedule.yaml').read_text(), Loader=yaml.CSafeLoader)['statistics']
        assert stats['cost'] == soc
        row.update(first_solution_time_ms=stats['runtime'] * 1000, runtime_ms=stats['runtime'] * 1000)
    assert row.get('first_solution_time_ms', row.get('initial_solution_time_ms', 0)) <= 10000
    history = run / 'validation_failure_history' / row['case_id'] / row['method']
    history.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(folder, history, copy_function=os.link)
    row.pop('infrastructure_error')
    row.update(solved=1, valid_solution=1, soc=soc, makespan=makespan,
               recovered_from_validation_pool_failure=True,
               validation_failure_record=str((history / 'result.json').relative_to(run)),
               validation_recovered_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    for name in ['schedule.yaml', 'schedule.yaml.bin', 'stdout.log', 'stderr.log']:
        source = folder / name
        if source.exists():
            with gzip.open(str(source) + '.gz', 'wb') as stream:
                stream.write(source.read_bytes())
            source.unlink()
    atomic_json(folder / 'isolated_validation.json', {'argv': argv, 'exit_code': 0, **metrics})
    atomic_json(path, row)
    return {'case_id': row['case_id'], 'method': row['method'], 'recovered': True}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['validate', 'recover'])
    parser.add_argument('--fixture', type=Path)
    parser.add_argument('--schedule', type=Path)
    parser.add_argument('--run-dir', type=Path)
    args = parser.parse_args()
    if args.mode == 'validate':
        soc, makespan = validate(args.fixture, args.schedule)
        print(json.dumps({'soc': soc, 'makespan': makespan}))
    else:
        run = args.run_dir.resolve()
        pid = int((run / 'run.pid').read_text())
        assert not Path(f'/proc/{pid}').exists(), 'Controller must stop before recovery'
        paths = [p for p in (run / 'artifacts').glob('*/*/*/result.json')
                 if 'BrokenProcessPool' in json.loads(p.read_text()).get('infrastructure_error', '')]
        results = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=24) as pool:
            for result in pool.map(recover, [(i, run, p) for i, p in enumerate(paths)]):
                results.append(result)
                if len(results) % 25 == 0:
                    atomic_json(run / 'validation_recovery.json', {'completed': len(results), 'total': len(paths),
                                                                 'recovered': sum(r['recovered'] for r in results)})
                    print(f'Recovery {len(results)}/{len(paths)}', flush=True)
        rows = [json.loads(p.read_text()) for p in (run / 'artifacts').glob('*/*/*/result.json')]
        with (run / 'rows.jsonl').open('w') as stream:
            for row in rows:
                stream.write(json.dumps(row) + '\n')
        export(run, rows)
        atomic_json(run / 'validation_recovery.json', {'completed': len(results), 'total': len(paths),
                                                     'recovered': sum(r['recovered'] for r in results), 'results': results})
        status = json.loads((run / 'status.json').read_text())
        status.update(state='solvers_finished', infrastructure_errors=sum('infrastructure_error' in r for r in rows))
        atomic_json(run / 'status.json', status)
        print('Recovered', sum(r['recovered'] for r in results), '/', len(paths), flush=True)
