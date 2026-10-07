"""Replay crash records once, validate, and replace them while retaining originals.

Run only while the batch controller is stopped. Resume that controller afterward
so CSV, JSONL, status, and summaries are rebuilt from authoritative result files.
"""
import argparse
import concurrent.futures
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from run_paper_2609_14208 import atomic_json, parse_kv, parse_ir_stdout, validate
import yaml


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def replay(item):
    index, run, record_path = item
    original = json.loads(record_path.read_text())
    method = original['method']
    folder = record_path.parent
    stage = run / 'retry_staging' / original['case_id'] / original['method']
    stage.mkdir(parents=True, exist_ok=False)
    argv = json.loads((folder / 'command.json').read_text())['argv']
    binary = Path(argv[3])
    fixture = Path(original['fixture_file'])
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == original['binary_sha256']
    assert hashlib.sha256(fixture.read_bytes()).hexdigest() == original['fixture_sha256']
    assert hashlib.sha256(Path(original['matrix_file']).read_bytes()).hexdigest() == original['matrix_sha256']
    schedule = stage / 'schedule.yaml'
    argv[2] = str(122 + index % 2)
    env = {**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
           'MKL_NUM_THREADS': '1', 'RAYON_NUM_THREADS': '1'}
    assert original['time_limit'] == 10
    if method == 'lacam_dfs':
        argv[7] = str(schedule)
        assert float(argv[6]) == 10
        assert argv[10] == str(original['seed']) and argv[11] == 'dfs'
    elif method == 'itacbs':
        argv[argv.index('-o') + 1] = str(schedule)
        assert float(argv[argv.index('--time-limit-sec') + 1]) == 10
        assert argv[argv.index('--seed') + 1] == str(original['seed'])
    elif method == 'ir':
        env['TAPF_SCHEDULE_OUTPUT'] = str(schedule)
        assert float(argv[argv.index('--time-limit-sec') + 1]) == 10
        assert argv[argv.index('--solver') + 1] == 'dbs_hungarian'
    else:
        raise ValueError(method)
    atomic_json(stage / 'command.json', {'argv': argv, 'timeout': 130.0})
    began = time.monotonic()
    with (stage / 'stdout.log').open('wb') as stdout, (stage / 'stderr.log').open('wb') as stderr:
        process = subprocess.run(argv, stdout=stdout, stderr=stderr, timeout=130,
                                 env=env)
    stdout_text = (stage / 'stdout.log').read_text()
    stats = parse_kv(stdout_text) if method == 'lacam_dfs' else parse_ir_stdout(stdout_text) if method == 'ir' else {}
    normal_timeout = method == 'itacbs' and process.returncode == -signal.SIGALRM
    if process.returncode != 0 and not normal_timeout:
        atomic_json(stage / 'retry_failure.json', {'exit_code': process.returncode, 'stats': stats})
        raise RuntimeError(f'{original["case_id"]}: retry failed, original preserved')
    solved = process.returncode == 0 and schedule.exists()
    soc, makespan = None, None
    if solved:
        soc, makespan = validate(fixture, schedule)
        if method == 'itacbs':
            statistics = yaml.safe_load(schedule.read_text())['statistics']
            assert statistics['cost'] == soc
            stats.update(first_solution_time_ms=statistics['runtime'] * 1000,
                         runtime_ms=statistics['runtime'] * 1000)
        elif method == 'ir':
            assert stats['soc'] == soc
        assert stats.get('first_solution_time_ms', stats.get('initial_solution_time_ms', 0)) <= 10000
    row = {k: v for k, v in original.items() if k != 'infrastructure_error'}
    row.update(stats)
    row.update(solved=int(solved), valid_solution=int(solved), soc=soc, makespan=makespan,
               exit_code=process.returncode, timed_out=int(normal_timeout), external_timed_out=0, wall_time_s=time.monotonic() - began,
               retry_count=original.get('retry_count', 0) + 1,
               retry_reason='User requested rerun and inclusion in existing results',
               retry_completed_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    for name in ['schedule.yaml', 'schedule.yaml.bin', 'stdout.log', 'stderr.log']:
        path = stage / name
        if path.exists():
            with gzip.open(str(path) + '.gz', 'wb') as stream:
                stream.write(path.read_bytes())
            path.unlink()
    history = run / 'retry_history' / original['case_id'] / original['method'] / f'attempt_{row["retry_count"] - 1}'
    if history.exists():
        raise FileExistsError(history)
    row['previous_attempt_record'] = str((history / 'result.json').relative_to(run))
    row['original_exit_code'] = original['exit_code']
    atomic_json(stage / 'result.json', row)
    history.parent.mkdir(parents=True, exist_ok=True)
    folder.rename(history)
    stage.rename(folder)
    print(f'REPLACED {original["case_id"]}: solved={solved}, SOC={soc}, makespan={makespan}', flush=True)
    return {'case_id': original['case_id'], 'previous_attempt_record': row['previous_attempt_record'],
            'new_result': str(record_path.relative_to(run)), 'soc': soc, 'makespan': makespan}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    controller = int((run / 'run.pid').read_text())
    while alive(controller):
        time.sleep(2)
    records = [p for p in (run / 'artifacts').glob('*/*/*/result.json')
               if 'infrastructure_error' in json.loads(p.read_text())]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(replay, [(i, run, p) for i, p in enumerate(records)]))
    all_retried = [json.loads(p.read_text()) for p in (run / 'artifacts').glob('*/*/*/result.json')
                   if json.loads(p.read_text()).get('retry_count')]
    atomic_json(run / 'retry_reconciliation.json', {'replaced': all_retried, 'count': len(all_retried),
                                                  'historical_failures_preserved': True})
