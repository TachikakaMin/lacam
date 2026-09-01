#!/usr/bin/env python3
"""Structural ablations on the protected development cases.

Variants:
  full        integrated search + automatic plan repair
  b0          rollout only, no search
  b1          two-stage frozen shelf plans

The experiment contract is fixed: 10 seconds, 14 jobs, solver seed 0,
unit objective weights, default following semantics, and authoritative
Python replay of every reported success.
"""

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

BENCH = Path(__file__).resolve().parent
sys.path.insert(0, str(BENCH))

from ddbench.instance import load_instance
from run_benchmark import row_carrier

TIME_LIMIT = 10
JOBS = 14

VARIANTS = [
    ("full", "lacam"),
    ("b0", "b0"),
    ("b1", "b1"),
]

FIELDS = [
    "case", "family", "variant", "solved", "executed_makespan",
    "weighted_soc", "loaded_moves", "free_moves", "lift_drop",
    "shelf_switches", "robot_utilization", "reversals",
    "first_solution_ms", "runtime_sec", "status", "raw",
]


def run_one(case, name, mode, work):
    path = BENCH / case
    ins = load_instance(path)
    env = dict(os.environ)
    for key in ("DD_ALPHA", "DD_BETA", "DD_GAMMA", "DD_DELTA",
                "DD_DEBUG_DUMP"):
        env.pop(key, None)
    row = row_carrier(
        ins, path, Path(case).stem, Path(case).parent.name, work,
        TIME_LIMIT, mode, env=env,
    )
    print(f"{Path(case).stem:42s} {name:11s} solved={row['success']}"
          f" mk={row['executed_makespan']}", flush=True)
    return {
        "case": row["instance"],
        "family": row["family"],
        "variant": name,
        "solved": row["success"],
        "executed_makespan": row["executed_makespan"],
        "weighted_soc": row["weighted_soc"],
        "loaded_moves": row["loaded_moves"],
        "free_moves": row["free_moves"],
        "lift_drop": row["lift_drop"],
        "shelf_switches": row["shelf_switches"],
        "robot_utilization": row["robot_utilization"],
        "reversals": row["reversals"],
        "first_solution_ms": row["first_solution_ms"],
        "runtime_sec": row["runtime_sec"],
        "status": row["status"],
        "raw": row["raw"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir", default=str(BENCH / "results_ablation"),
        help="new output directory; do not overwrite a comparison result",
    )
    args = parser.parse_args()

    from concurrent.futures import ThreadPoolExecutor
    cases = [
        line.strip()
        for line in (BENCH / "dev_cases.txt").read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    tasks = [(c, n, m) for c in cases for (n, m) in VARIANTS]
    out = Path(args.out_dir)
    work = out / "work"
    work.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=JOBS) as ex:
        rows = list(ex.map(lambda t: run_one(*t, work), tasks))
    wall = time.monotonic() - started
    rows.sort(key=lambda row: (row["case"], row["variant"]))
    with open(out / "ablation_rows.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    summary = {}
    for name, _ in VARIANTS:
        selected = [row for row in rows if row["variant"] == name]
        summary[name] = {
            "solved": sum(int(row["solved"]) for row in selected),
            "total": len(selected),
        }
    timing = {
        "wall_time_sec": round(wall, 1),
        "jobs": JOBS,
        "n_tasks": len(tasks),
        "timeout_per_run_sec": TIME_LIMIT,
        "solver_seed": 0,
        "objective_weights": {
            "alpha": 1, "beta": 1, "gamma": 1, "delta": 1,
        },
        "following": "allowed",
        "variants": summary,
    }
    (out / "timing.json").write_text(json.dumps(timing, indent=2) + "\n")
    print(f"written {out / 'ablation_rows.csv'}")
    print(f"written {out / 'timing.json'}")


if __name__ == "__main__":
    main()
