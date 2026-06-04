#!/usr/bin/env python3
"""Wait for the 2307 opt run to finish, then render figures with opt rows."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def read_pid(path: Path) -> int | None:
    try:
        text = path.read_text(encoding="utf-8").strip().splitlines()[0]
        return int(text.split()[0])
    except (FileNotFoundError, IndexError, ValueError):
        return None


def count_jsonl(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as f:
        return sum(1 for line in f if line.strip())


def choose_rows_file(run_dir: Path) -> Path:
    rows_jsonl = run_dir / "rows.jsonl"
    if rows_jsonl.exists() and rows_jsonl.stat().st_size > 0:
        return rows_jsonl
    rows_csv = run_dir / "rows.csv"
    if rows_csv.exists() and rows_csv.stat().st_size > 0:
        return rows_csv
    raise FileNotFoundError(f"no rows.csv or rows.jsonl under {run_dir}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("build/results/full_2307_exp1_exp2_opt_dbs_hungarian_10s_split5"),
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("build/results/paper_2307_00663_figures_with_opt"),
    )
    parser.add_argument("--pid-file", type=Path, default=None)
    parser.add_argument("--poll-seconds", type=float, default=60.0)
    parser.add_argument("--min-rows", type=int, default=9360)
    parser.add_argument(
        "--plotter",
        type=Path,
        default=Path("tools/plot_paper_2307_00663_comparison.py"),
    )
    parser.add_argument(
        "--auditor",
        type=Path,
        default=Path("tools/audit_paper_2307_00663_figures.py"),
    )
    args = parser.parse_args()

    run_dir = args.run_dir
    pid_file = args.pid_file or run_dir / "run.pid"
    status_path = run_dir / "finalize_status.json"
    pid = read_pid(pid_file)
    start = time.time()
    print(f"watching pid={pid} run_dir={run_dir}", flush=True)
    while pid is not None and pid_is_alive(pid):
        rows = count_jsonl(run_dir / "rows.jsonl")
        status_path.write_text(
            json.dumps(
                {
                    "status": "waiting",
                    "pid": pid,
                    "rows_jsonl": rows,
                    "elapsed_s": time.time() - start,
                },
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"waiting pid={pid} rows={rows}", flush=True)
        time.sleep(args.poll_seconds)

    rows = count_jsonl(run_dir / "rows.jsonl")
    if rows < args.min_rows:
        message = f"rows_jsonl={rows} below min_rows={args.min_rows}; not rendering final figures"
        print("incomplete: " + message, flush=True)
        try:
            rows_file = choose_rows_file(run_dir)
        except FileNotFoundError:
            rows_file = run_dir / "rows.jsonl"
        status_path.write_text(
            json.dumps(
                {
                    "status": "incomplete",
                    "pid": pid,
                    "rows_file": str(rows_file),
                    "rows_jsonl": rows,
                    "min_rows": args.min_rows,
                    "message": message,
                    "elapsed_s": time.time() - start,
                },
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        return 2
    rows_file = choose_rows_file(run_dir)

    cmd = [
        sys.executable,
        str(args.plotter),
        "--extra-rows",
        str(rows_file),
        "--out-dir",
        str(args.out_dir),
    ]
    print("running " + " ".join(cmd), flush=True)
    cp = subprocess.run(cmd, text=True)
    audit_returncode = None
    if cp.returncode == 0:
        audit_cmd = [
            sys.executable,
            str(args.auditor),
            str(args.out_dir),
            "--require-opt",
        ]
        print("running " + " ".join(audit_cmd), flush=True)
        audit_cp = subprocess.run(audit_cmd, text=True)
        audit_returncode = audit_cp.returncode
    final_returncode = cp.returncode if cp.returncode != 0 else int(audit_returncode or 0)
    status_path.write_text(
        json.dumps(
            {
                "status": "done"
                if final_returncode == 0
                else "plot_failed"
                if cp.returncode != 0
                else "audit_failed",
                "pid": pid,
                "rows_file": str(rows_file),
                "rows_jsonl": rows,
                "out_dir": str(args.out_dir),
                "plot_returncode": cp.returncode,
                "audit_returncode": audit_returncode,
                "returncode": final_returncode,
                "elapsed_s": time.time() - start,
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return final_returncode


if __name__ == "__main__":
    raise SystemExit(main())
