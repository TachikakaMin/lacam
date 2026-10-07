"""Expose actual failure traces and per-map progress without changing solvers."""
import argparse
import gzip
import json
import os
import time
from collections import defaultdict
from pathlib import Path

def atomic(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2))
    temporary.replace(path)

def live(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False

def monitor(run):
    os.sched_setaffinity(0, {120})
    offset, pending, records, traces = 0, '', {}, {}
    while True:
        source = run / 'rows.jsonl'
        if source.exists():
            if source.stat().st_size < offset:
                offset, pending, records = 0, '', {}
            with source.open() as stream:
                stream.seek(offset)
                pending += stream.read()
                offset = stream.tell()
            lines = pending.split('\n')
            pending = lines.pop()
            for line in lines:
                if line:
                    row = json.loads(line)
                    records[row['case_id'], row['method']] = row
        groups = defaultdict(lambda: {'completed': 0, 'solved': 0})
        failures = []
        for row in records.values():
            group = groups[row['map'], row['method']]
            group['completed'] += 1
            group['solved'] += row['valid_solution']
            if not row['valid_solution']:
                failures.append(row)
        examples = []
        selected = [row for row in failures if row.get('infrastructure_error')]
        selected += [row for row in failures if not row.get('infrastructure_error')][-20:]
        for row in selected:
            key = row['case_id'], row['method']
            folder = run / 'artifacts' / row['case_id'] / row['method']
            if key not in traces:
                trace = {}
                for name in ('stdout', 'stderr'):
                    path = folder / f'{name}.log.gz'
                    plain = folder / f'{name}.log'
                    trace[name] = (gzip.open(path, 'rt', errors='replace').read()[-6000:] if path.exists() else
                                   plain.read_text(errors='replace')[-6000:] if plain.exists() else '')
                traces[key] = trace
            reason = (row['infrastructure_error'] if row.get('infrastructure_error') else
                      '10-second CBS search timer expired' if row['method'] == 'itacbs' and row.get('exit_code') == -14 else
                      'No valid incumbent returned; inspect actual logs')
            examples.append({k: row.get(k) for k in ['case_id', 'method', 'map', 'scenario', 'num_agents', 'seed', 'exit_code', 'wall_time_s']} |
                            {'reason': reason, 'record_path': str((folder / 'result.json').relative_to(run)),
                             'command_path': str((folder / 'command.json').relative_to(run)), **traces[key]})
        atomic(run / 'failure_examples.json', examples)
        atomic(run / 'map_progress.json', [{'map': name, 'method': method, **value} for (name, method), value in groups.items()])
        plan_path = run / 'timeout_review_plan.json'
        if plan_path.exists():
            plan = json.loads(plan_path.read_text())
            progress = {}
            for item in plan['pending']:
                value = progress.setdefault(item['method'], {'total': 0, 'reviewed': 0, 'solved': 0})
                value['total'] += 1
                row = records.get((item['case_id'], item['method']))
                if row and row.get('timeout_review_previous_record'):
                    value['reviewed'] += 1
                    value['solved'] += row['valid_solution']
            atomic(run / 'timeout_review_progress.json', progress)
        status = json.loads((run / 'status.json').read_text())
        main = int((run / 'run.pid').read_text())
        report = status.get('report_pid')
        if not live(main) and not (report and live(report)):
            print('Main/report handles terminal; trace export finished', flush=True)
            break
        time.sleep(5)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    monitor(parser.parse_args().run_dir.resolve())
