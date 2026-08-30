#!/usr/bin/env python3
"""design.md 8.4 ablations on the protected dev cases (debug.md task 13).

Variants (all via dd_benchmark; env knobs are ordering-only switches):
  full        default configuration
  no_macro    DD_MACRO_CAP=0        (B0-vs-full inside the searcher)
  strict_inv  DD_STRICT_INVAL=1     (design-6.2 asymmetric off)
  no_yield    DD_NO_YIELD=1         (head-on carrier yield off)
  b0          MODE=b0               (rollout only, no search)
  b1          MODE=b1               (2-stage frozen shelf plans)

Writes ablation_rows.csv: case, variant, solved, makespan, soc,
first_solution_ms, runtime_sec.
"""

import csv
import os
import subprocess
import sys
import time
from pathlib import Path

BENCH = Path(__file__).resolve().parent
BIN = BENCH.parent / "build/dd_benchmark"
TIME_LIMIT = 10

VARIANTS = [
    ("full", {}, "lacam"),
    ("no_macro", {"DD_MACRO_CAP": "0"}, "lacam"),
    ("strict_inv", {"DD_STRICT_INVAL": "1"}, "lacam"),
    ("no_yield", {"DD_NO_YIELD": "1"}, "lacam"),
    ("b0", {}, "b0"),
    ("b1", {}, "b1"),
]


def main():
    cases = [
        line.strip()
        for line in (BENCH / "dev_cases.txt").read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]
    rows = []
    for case in cases:
        for name, env_extra, mode in VARIANTS:
            env = dict(os.environ, **env_extra)
            plan_out = f"/tmp/abl_{Path(case).stem}_{name}.plan"
            t0 = time.time()
            try:
                p = subprocess.run(
                    [str(BIN), str(BENCH / case), str(TIME_LIMIT), plan_out,
                     "0", mode],
                    capture_output=True, text=True, timeout=TIME_LIMIT + 30,
                    env=env,
                )
                m = dict(l.split("=", 1) for l in p.stdout.splitlines()
                         if "=" in l)
            except subprocess.TimeoutExpired:
                m = {}
            rows.append(dict(
                case=Path(case).stem, variant=name,
                solved=m.get("solved", "0"),
                makespan=m.get("makespan", ""),
                weighted_soc=m.get("weighted_soc", ""),
                first_solution_ms=m.get("first_solution_ms", ""),
                runtime_sec=round(time.time() - t0, 2),
            ))
            print(f"{Path(case).stem:42s} {name:11s} solved={m.get('solved','0')}"
                  f" mk={m.get('makespan','')}", flush=True)
    out = BENCH / "results_ablation"
    out.mkdir(exist_ok=True)
    with open(out / "ablation_rows.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"written {out/'ablation_rows.csv'}")


if __name__ == "__main__":
    main()
