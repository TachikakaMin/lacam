#!/usr/bin/env python3
"""Run all three paper solvers with durable results and CPU/memory monitoring."""
import argparse
import asyncio
import csv
import concurrent.futures
import gzip
from concurrent.futures.process import BrokenProcessPool
import hashlib
import json
import multiprocessing
import os
import signal
import re
import subprocess
import time
from collections import Counter
from pathlib import Path

for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'

import numpy as np
import yaml

from run_full_three_method_experiment import parse_kv
from run_ir_lacam_cross_experiment import parse_ir_stdout
from tapf_schedule_io import read_sparse_binary

METHODS = ('lacam_dfs', 'ir', 'itacbs')
MAP_CACHE = {}

def read_validation_schedule(schedule):
    text = schedule.read_text()
    # This exact format is emitted by IR's dbs_hungarian.rs. Parse every line,
    # reject duplicate agents, and leave all path checks to validate().
    header = re.match(r'\Astatistics:\n  cost: (\d+)\nschedule:\n', text)
    if header is None:
        return yaml.load(text, Loader=yaml.SafeLoader)
    state_pattern = re.compile(r'    - \{t: (-?\d+), x: (-?\d+), y: (-?\d+)\}')
    agent_pattern = re.compile(r'  (agent\d+):')
    paths, path = {}, None
    for line in text[header.end():].splitlines():
        agent = agent_pattern.fullmatch(line)
        if agent:
            name = agent.group(1)
            if name in paths:
                raise ValueError('duplicate IR schedule agent')
            paths[name] = path = []
            continue
        state = state_pattern.fullmatch(line)
        if state is None or path is None:
            raise ValueError('unexpected line in IR schedule')
        t, x, y = map(int, state.groups())
        path.append({'t': t, 'x': x, 'y': y})
    return {'statistics': {'cost': int(header.group(1))}, 'schedule': paths}

def atomic_json(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=True))
    temp.replace(path)

def validate(fixture, schedule):
    """Independently check map, starts, moves, collisions, eligibility and SOC."""
    data = yaml.load(fixture.read_text(), Loader=yaml.SafeLoader)
    # A retained IR schedule reproduces a native CSafeLoader/GC segfault.
    # Pure Python loading keeps schedule parsing out of that native path.
    result = read_validation_schedule(schedule)
    if (result or {}).get('schedule_binary'):
        binary = result['schedule_binary']
        makespan, raw_paths = read_sparse_binary(schedule.parent / binary['path'])
        result['schedule'] = {f'agent{i}': path for i, path in enumerate(raw_paths)}
    paths = (result or {}).get('schedule') or {}
    agents = data['agents']
    if len(paths) != len(agents):
        raise ValueError('schedule agent count mismatch')
    map_path = fixture.parent / data['map']
    if map_path not in MAP_CACHE:
        lines = map_path.read_text().splitlines()
        MAP_CACHE[map_path] = np.array([[c not in '@T' for c in row] for row in lines[4:]])
    grid = MAP_CACHE[map_path]
    horizon = max(int(p[-1]['t']) for p in paths.values())
    positions = np.empty((horizon + 1, len(agents), 2), dtype=np.int32)
    soc = 0
    for i, agent in enumerate(agents):
        path = paths[agent['name']]
        if not path or int(path[0]['t']) != 0 or [path[0]['x'], path[0]['y']] != agent['start']:
            raise ValueError('invalid start')
        previous_t = -1
        for k, state in enumerate(path):
            t = int(state['t'])
            if t <= previous_t or t > horizon:
                raise ValueError('nonmonotonic schedule')
            end = int(path[k + 1]['t']) if k + 1 < len(path) else horizon + 1
            positions[t:end, i] = [state['x'], state['y']]
            previous_t = t
        goal = positions[-1, i].tolist()
        if goal not in agent['potentialGoals']:
            raise ValueError('ineligible final target')
        away = np.flatnonzero(np.any(positions[:, i] != positions[-1, i], axis=1))
        soc += int(away[-1]) + 1 if len(away) else 0
    if np.any(positions < 0) or np.any(positions[:, :, 0] >= grid.shape[0]) or np.any(positions[:, :, 1] >= grid.shape[1]):
        raise ValueError('out of map')
    if not np.all(grid[positions[:, :, 0], positions[:, :, 1]]):
        raise ValueError('obstacle collision')
    if np.any(np.abs(np.diff(positions, axis=0)).sum(axis=2) > 1):
        raise ValueError('invalid move')
    ids = positions[:, :, 0].astype(np.int64) * grid.shape[1] + positions[:, :, 1]
    if np.any(np.diff(np.sort(ids, axis=1), axis=1) == 0):
        raise ValueError('vertex collision')
    for t in range(horizon):
        edges = set()
        for u, v in zip(ids[t], ids[t + 1]):
            if u != v and (v, u) in edges:
                raise ValueError('edge swap collision')
            edges.add((u, v))
    return soc, horizon

def cpu_snapshot():
    vals = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:]))
    return sum(vals), vals[3] + vals[4]

def memory_available_gib():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 1024**2
    return 0

def export(out, rows, methods=METHODS):
    if not rows:
        return
    fields = sorted(set().union(*(r.keys() for r in rows)))
    temp = out / 'rows.csv.tmp'
    with temp.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temp.replace(out / 'rows.csv')
    summary = []
    for suite in ('group', 'common'):
        for method in methods:
            group = [r for r in rows if r['suite'] == suite and r['method'] == method]
            solved = [r for r in group if r['valid_solution']]
            summary.append({'suite': suite, 'method': method, 'completed': len(group),
                            'solved': len(solved), 'success_rate': len(solved) / len(group) if group else None,
                            'mean_soc': np.mean([r['soc'] for r in solved]).item() if solved else None})
    atomic_json(out / 'summary.json', summary)

async def main(args):
    repo = Path(__file__).resolve().parents[1]
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(args.manifest.read_text())
    methods = tuple(getattr(args, 'methods', METHODS))
    cases = manifest['cases']
    # Normalized MovingAI maps give all three solver parsers identical geometry.
    checked_maps = set()
    for case in cases:
        matrix = Path(case['matrix_file'])
        with matrix.open() as stream:
            next(stream)
            map_path = Path(next(stream).split(':', 1)[1].strip())
        if map_path not in checked_maps:
            raw = map_path.read_bytes()
            if set(''.join(raw.decode().splitlines()[4:])) - set('.@T'):
                raise ValueError(f'{map_path}: normalize free cells to . before running all three solvers')
            if case.get('map_sha256') and hashlib.sha256(raw).hexdigest() != case['map_sha256']:
                raise ValueError(f'{map_path}: map hash changed')
            checked_maps.add(map_path)
        for key, path in [('fixture_sha256', Path(case['fixture_file'])), ('matrix_sha256', matrix)]:
            if case.get(key) and hashlib.sha256(path.read_bytes()).hexdigest() != case[key]:
                raise ValueError(f'{path}: input hash changed')
    if args.smoke:
        selected = []
        for name in dict.fromkeys(c['map'] for c in cases):
            for suite in dict.fromkeys(c['suite'] for c in cases if c['map'] == name):
                group = [c for c in cases if c['map'] == name and c['suite'] == suite and c['seed'] == 0]
                key = lambda c: (c['num_agents'], c['scenario'])
                selected.extend((min(group, key=key), max(group, key=key)))
        cases = selected
    allowed = sorted(os.sched_getaffinity(0))
    if len(allowed) <= args.reserve_threads:
        raise ValueError('not enough CPU threads to reserve requested capacity')
    reserved = allowed[-args.reserve_threads:]
    experiment_cpus = allowed[:-args.reserve_threads]
    os.sched_setaffinity(0, experiment_cpus)
    jobs = min(args.jobs, len(experiment_cpus) - 1)
    validation_jobs = max(1, min(args.validation_jobs, len(experiment_cpus) - jobs))
    validation_cpus = experiment_cpus[jobs:jobs + validation_jobs]
    (out / 'run.pid').write_text(str(os.getpid()))
    binaries = {'lacam_dfs': repo / 'build/tapf_benchmark',
                'ir': getattr(args, 'ir_binary', None) or repo.parent / 'ir-tapf/target/release/ir_tapf',
                'itacbs': repo / 'third_party/ITA-CBS2/build/ITACBS_remake'}
    binaries = {k: v for k, v in binaries.items() if k in methods}
    provenance = {'methods': list(methods), 'paper': manifest['paper'], 'jobs': jobs, 'reserved_cpus': reserved,
                  'validation_jobs': validation_jobs, 'validation_cpus': validation_cpus,
                  'experiment_cpus': experiment_cpus, 'time_limit_sec': args.time_limit,
                  'dataset_manifest_sha256': hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
                  'binaries_sha256': {k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in binaries.items()},
                  'hardware': subprocess.check_output(['lscpu'], text=True),
                  'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    atomic_json(out / 'provenance.json', provenance)
    rows, pending = [], []
    for case in cases:
        for method in methods:
            folder = out / 'artifacts' / case['case_id'] / method
            record = folder / 'result.json'
            if record.exists():
                previous = json.loads(record.read_text())
                if (previous.get('fixture_sha256') != case['fixture_sha256'] or
                        previous.get('binary_sha256') != provenance['binaries_sha256'][method] or
                        previous.get('time_limit') != args.time_limit):
                    raise ValueError(f'{record}: incompatible old attempt; use a separate output directory')
                rows.append(previous)
            else:
                pending.append((case, method, folder))
    # Keep the append-only view consistent with the authoritative atomic records
    # when a reviewed attempt was archived before resuming.
    with (out / 'rows.jsonl').open('w') as f:
        for row in rows:
            f.write(json.dumps(row) + '\n')
    # Shuffle reproducibly to distribute large maps and hard CBS runs evenly.
    import random
    random.Random(260914208).shuffle(pending)
    queue = asyncio.Queue()
    for task in pending:
        queue.put_nowait(task)
    active = {}
    start = time.monotonic()
    stopping = False
    loop = asyncio.get_running_loop()
    loop.set_default_executor(concurrent.futures.ThreadPoolExecutor(max_workers=4))
    validation_sem = asyncio.Semaphore(validation_jobs)
    async def isolated_validate(fixture, schedule):
        async with validation_sem:
            command = ['taskset', '-c', ','.join(map(str, validation_cpus)),
                       os.sys.executable, '-X', 'faulthandler',
                       str(repo / 'tools/recover_validation_pool.py'), 'validate',
                       '--fixture', str(fixture), '--schedule', str(schedule)]
            for attempt in range(2):
                proc = await asyncio.create_subprocess_exec(*command, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE, env=dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
                try:
                    stdout, stderr = await asyncio.wait_for(proc.communicate(), 600)
                except asyncio.TimeoutError:
                    proc.kill()
                    await proc.communicate()
                    raise RuntimeError('isolated validator exceeded 600 seconds')
                (schedule.parent / f'validation_attempt_{attempt}.json').write_text(json.dumps({
                    'argv': command, 'exit_code': proc.returncode, 'stderr': stderr.decode(errors='replace')}))
                if proc.returncode == 0:
                    metrics = json.loads(stdout)
                    return metrics['soc'], metrics['makespan']
                if proc.returncode >= 0:
                    break
            raise RuntimeError(f'isolated validator failed: {stderr.decode(errors="replace")[-2000:]}')
    def stop():
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stop)

    async def worker(index):
        while not queue.empty() and not stopping:
            while memory_available_gib() < args.min_free_gib and not stopping:
                await asyncio.sleep(2)
            if stopping:
                break
            case, method, folder = queue.get_nowait()
            folder.mkdir(parents=True, exist_ok=True)
            schedule = folder / 'schedule.yaml'
            fixture = Path(case['fixture_file'])
            seed = str(case['seed'])
            env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', RAYON_NUM_THREADS='1')
            if method == 'lacam_dfs':
                cmd = [str(binaries[method]), str(fixture), '', str(args.time_limit), str(schedule), '1', '0', seed, 'dfs']
                timeout = args.time_limit + 120
            elif method == 'ir':
                cmd = [str(binaries[method]), 'solve', '--matrix', case['matrix_file'], '--solver', 'dbs_hungarian', '--max-iterations', '100000', '--time-limit-sec', str(args.time_limit)]
                env['TAPF_SCHEDULE_OUTPUT'] = str(schedule)
                timeout = args.time_limit + 120
            else:
                cmd = [str(binaries[method]), '-i', str(fixture), '-o', str(schedule), '--seed', seed, '--time-limit-sec', str(args.time_limit)]
                timeout = args.time_limit + 120
            cmd = ['taskset', '-c', str(experiment_cpus[index]), *cmd]
            began = time.monotonic()
            row = {**case, 'method': method, 'solved': 0, 'valid_solution': 0, 'time_limit': args.time_limit,
                   'binary_sha256': provenance['binaries_sha256'][method]}
            previous_timeout = out / 'timeout_review_history' / case['case_id'] / method / 'result.json'
            if previous_timeout.exists():
                row.update(timeout_review_previous_record=str(previous_timeout.relative_to(out)),
                           review_solver_jobs=jobs, review_validation_jobs=validation_jobs)
            atomic_json(folder / 'command.json', {'argv': cmd, 'timeout': timeout})
            try:
                with (folder / 'stdout.log').open('wb') as stdout, (folder / 'stderr.log').open('wb') as stderr:
                    proc = await asyncio.create_subprocess_exec(*cmd, stdout=stdout, stderr=stderr, env=env, start_new_session=True)
                    active[index] = {'pid': proc.pid, 'method': method, 'case_id': case['case_id'], 'cpu': experiment_cpus[index], 'phase': 'solving'}
                    try:
                        await asyncio.wait_for(proc.wait(), timeout)
                        row['exit_code'] = proc.returncode
                        row['timed_out'] = int(proc.returncode == -signal.SIGALRM)
                        row['external_timed_out'] = 0
                    except asyncio.TimeoutError:
                        if proc.returncode is not None:
                            row['exit_code'] = proc.returncode
                            row['external_timed_out'] = 0
                        else:
                            try:
                                os.killpg(proc.pid, signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                            await proc.wait()
                            row['exit_code'] = 124
                            row['external_timed_out'] = 1
                active[index]['phase'] = 'validating'
                active[index]['pid'] = None
                stdout = (folder / 'stdout.log').read_text(errors='replace')
                if method == 'ir' and row['external_timed_out']:
                    raise RuntimeError('IR process exceeded setup/cleanup allowance; requires investigation')
                if row['exit_code'] < 0 and not (method == 'itacbs' and row['exit_code'] == -signal.SIGALRM):
                    raise RuntimeError(f'solver crashed with exit code {row["exit_code"]}')
                stats = parse_kv(stdout) if method == 'lacam_dfs' else parse_ir_stdout(stdout) if method == 'ir' else {}
                row.update(stats)
                has_solution = bool(stats.get('solved')) if method != 'itacbs' else schedule.exists() and bool((yaml.load(schedule.read_text(), Loader=yaml.CSafeLoader) or {}).get('schedule'))
                if has_solution and schedule.exists() and not row['external_timed_out'] and row['exit_code'] == 0:
                    soc, makespan = await isolated_validate(fixture, schedule)
                    if method == 'ir' and stats.get('soc') != soc:
                        raise ValueError(f'IR reported SOC {stats.get("soc")} != path SOC {soc}')
                    row.update(soc=soc, makespan=makespan, solved=1, valid_solution=1)
                    first_ms = row.get('first_solution_time_ms', row.get('initial_solution_time_ms', 0))
                    if first_ms > args.time_limit * 1000:
                        raise ValueError('first solution exceeds solve budget')
                    if method == 'itacbs':
                        stats = yaml.load(schedule.read_text(), Loader=yaml.CSafeLoader)['statistics']
                        if stats['cost'] != soc:
                            raise ValueError('CBS optimal cost disagrees with path SOC')
                        row['runtime_ms'] = stats['runtime'] * 1000
                        row['first_solution_time_ms'] = row['runtime_ms']
                else:
                    row.update(solved=0, valid_solution=0)
                for path in (schedule, Path(str(schedule) + '.bin'), folder / 'stdout.log', folder / 'stderr.log'):
                    if path.exists():
                        with gzip.open(str(path) + '.gz', 'wb') as compressed:
                            compressed.write(path.read_bytes())
                        path.unlink()
            except BrokenProcessPool as exc:
                stop()
                row.update(solved=0, valid_solution=0, infrastructure_error=repr(exc))
                print('VALIDATION POOL FAILED: stopped intake; preserving solver outputs for isolated recovery', flush=True)
            except Exception as exc:
                row.update(solved=0, valid_solution=0, infrastructure_error=repr(exc))
            finally:
                active.pop(index, None)
            row['wall_time_s'] = time.monotonic() - began
            atomic_json(folder / 'result.json', row)
            rows.append(row)
            with (out / 'rows.jsonl').open('a') as f:
                f.write(json.dumps(row) + '\n')
                f.flush()
                os.fsync(f.fileno())
            queue.task_done()

    async def monitor():
        previous = cpu_snapshot()
        last_export = 0
        while True:
            await asyncio.sleep(5)
            current = cpu_snapshot()
            idle = (os.cpu_count() or 1) * (current[1] - previous[1]) / max(1, current[0] - previous[0])
            previous = current
            status = {'pid': os.getpid(), 'updated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                      'completed': len(rows), 'total': len(cases) * len(methods), 'pending': queue.qsize(),
                      'active': len(active), 'active_processes': list(active.values()),
                      'running_solvers': sum(v['phase'] == 'solving' for v in active.values()),
                      'reserved_cpus': reserved, 'idle_threads_equivalent': idle,
                      'memory_available_gib': memory_available_gib(),
                      'elapsed_sec': time.monotonic() - start,
                      'infrastructure_errors': sum('infrastructure_error' in r for r in rows),
                      'valid_solutions': sum(r['valid_solution'] for r in rows), 'state': 'running'}
            atomic_json(out / 'status.json', status)
            with (out / 'monitor.jsonl').open('a') as f:
                f.write(json.dumps({k: v for k, v in status.items() if k != 'active_processes'}) + '\n')
            if time.monotonic() - last_export >= 60:
                export(out, rows, methods)
                last_export = time.monotonic()
            print(f"completed={len(rows)}/{len(cases)*len(methods)} active={len(active)} idle_threads={idle:.1f} available_GiB={status['memory_available_gib']:.1f} errors={status['infrastructure_errors']}", flush=True)

    print(f'cases={len(cases)} jobs={jobs} reserved_cpus={reserved} pending={len(pending)}', flush=True)
    watcher = asyncio.create_task(monitor())
    await asyncio.gather(*(worker(i) for i in range(jobs)))
    watcher.cancel()
    export(out, rows, methods)
    errors = sum('infrastructure_error' in r for r in rows)
    atomic_json(out / 'status.json', {'state': 'interrupted' if stopping else ('complete' if args.smoke else 'solvers_finished') if not errors else 'needs_review',
                                    'completed': len(rows), 'total': len(cases) * len(methods),
                                    'infrastructure_errors': errors, 'reserved_cpus': reserved})
    print(f'finished rows={len(rows)} errors={errors}', flush=True)
    if not stopping and not errors and not args.smoke and not getattr(args, 'skip_report', False):
        process = await asyncio.create_subprocess_exec(
            os.sys.executable, str(repo / 'tools/report_paper_2609_14208.py'),
            '--run-dir', str(out), '--manifest', str(args.manifest.resolve()))
        if await process.wait() != 0:
            raise RuntimeError('final coverage audit or figure generation failed')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=Path('data/paper_2609_14208/manifest.json'))
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=48)
    parser.add_argument('--validation-jobs', type=int, default=8)
    parser.add_argument('--reserve-threads', type=int, default=4)
    parser.add_argument('--time-limit', type=float, default=10)
    parser.add_argument('--min-free-gib', type=float, default=32)
    parser.add_argument('--ir-binary', type=Path, help='Use an explicitly recorded IR binary variant')
    parser.add_argument('--methods', nargs='+', choices=METHODS, default=list(METHODS))
    parser.add_argument('--skip-report', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    asyncio.run(main(parser.parse_args()))
