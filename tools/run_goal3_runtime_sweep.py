#!/usr/bin/env python3
"""Run the symbotic_star distribution/K/agent-count runtime sweep."""

from __future__ import annotations

import argparse
import csv
import subprocess
import time
from pathlib import Path


CASES = [
    ("80_20_inbound", 0.2),
    ("50_50", 0.5),
]


def read_single_row(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise RuntimeError(f"expected one row in {path}, got {len(rows)}")
    return rows[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="./build/lifelong_benchmark")
    parser.add_argument("--map", default="./tests/assets/symbotic_star.map")
    parser.add_argument("--out-dir", type=Path, default=Path("./build/results/goal3_runtime_rerun"))
    parser.add_argument("--agent-counts", default="10,20,50,100")
    parser.add_argument("--capacities", default="1,2,3")
    parser.add_argument("--horizon", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--time-limit-sec", type=float, default=2.0)
    parser.add_argument("--goal-set-size", type=int, default=3)
    parser.add_argument("--release-interval", type=int, default=10)
    parser.add_argument("--debug", action="store_true", default=True)
    parser.add_argument("--no-debug", dest="debug", action="store_false")
    parser.add_argument("--timeout-sec", type=int, default=900)
    args = parser.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = out_dir / "runtime_rerun_results.csv"
    cache_path = out_dir / "symbotic_star.distcache"
    agent_counts = [int(v) for v in args.agent_counts.split(",") if v]
    capacities = [int(v) for v in args.capacities.split(",") if v]

    rows: list[dict[str, str]] = []
    for distribution, outbound_prob in CASES:
        for capacity in capacities:
            for agents in agent_counts:
                case_csv = out_dir / f"case_{distribution}_k{capacity}_a{agents}.csv"
                if case_csv.exists():
                    case_csv.unlink()
                trace_csv = Path(str(case_csv) + ".trace.csv")
                if trace_csv.exists():
                    trace_csv.unlink()
                cmd = [
                    args.binary,
                    args.map,
                    str(agents),
                    str(args.horizon),
                    str(args.seed),
                    str(case_csv),
                    str(cache_path),
                    str(args.time_limit_sec),
                    str(args.goal_set_size),
                    str(outbound_prob),
                    str(args.release_interval),
                    "1" if args.debug else "0",
                    "",
                    "0",
                    str(capacity),
                ]
                print("running", " ".join(cmd), flush=True)
                start = time.time()
                result = subprocess.run(cmd, timeout=args.timeout_sec)
                elapsed = time.time() - start
                row = read_single_row(case_csv)
                row["distribution"] = distribution
                row["outbound_prob"] = str(outbound_prob)
                row["multi_carry_capacity"] = str(capacity)
                row["case_elapsed_sec"] = str(elapsed)
                row["returncode"] = str(result.returncode)
                rows.append(row)

                fieldnames = list(rows[0].keys())
                with summary_path.open("w", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
