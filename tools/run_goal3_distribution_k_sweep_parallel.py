#!/usr/bin/env python3
"""Run symbotic_star throughput sweep for distribution, K, and agent count."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path


CASES = [
    ("50_50", 0.5),
    ("80_20_inbound", 0.2),
]


@dataclass(frozen=True)
class Case:
    distribution: str
    outbound_prob: float
    capacity: int
    agents: int


def parse_int_list(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item]


def read_single_row(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise RuntimeError(f"expected one row in {path}, got {len(rows)}")
    return rows[0]


def run_case(args: argparse.Namespace, case: Case) -> dict[str, str]:
    case_name = f"{case.distribution}_k{case.capacity}_a{case.agents}"
    case_csv = args.out_dir / f"case_{case_name}.csv"
    trace_csv = Path(str(case_csv) + ".trace.csv")
    schedule_yaml = args.out_dir / f"case_{case_name}.yaml"
    for path in (case_csv, trace_csv, schedule_yaml):
        if path.exists():
            path.unlink()

    cmd = [
        args.binary,
        args.map,
        str(case.agents),
        str(args.horizon),
        str(args.seed),
        str(case_csv),
        str(args.cache),
        str(args.time_limit_sec),
        str(args.goal_set_size),
        str(case.outbound_prob),
        str(args.release_interval),
        "1" if args.debug else "0",
        str(schedule_yaml),
        "1" if args.anytime else "0",
        str(case.capacity),
        "0",
    ]
    start = time.time()
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=args.timeout_sec,
    )
    elapsed = time.time() - start
    if not case_csv.exists():
        raise RuntimeError(
            f"{case_name} did not produce {case_csv}\n"
            f"returncode={result.returncode}\nstdout={result.stdout}\n"
            f"stderr={result.stderr}"
        )
    row = read_single_row(case_csv)
    row["distribution"] = case.distribution
    row["outbound_prob"] = str(case.outbound_prob)
    row["requested_capacity"] = str(case.capacity)
    row["case_name"] = case_name
    row["case_elapsed_sec"] = f"{elapsed:.6f}"
    row["returncode"] = str(result.returncode)
    row["stdout"] = result.stdout.replace("\n", "\\n")
    row["stderr"] = result.stderr.replace("\n", "\\n")
    if result.returncode != 0:
        raise RuntimeError(
            f"{case_name} failed with {result.returncode}\n"
            f"stdout={result.stdout}\nstderr={result.stderr}"
        )
    return row


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="./build/lifelong_benchmark")
    parser.add_argument("--map", default="./tests/assets/symbotic_star.map")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("./build/results/goal3_distribution_k_sweep"),
    )
    parser.add_argument("--agent-counts", default="10,20,50,100")
    parser.add_argument("--capacities", default="1,2,3")
    parser.add_argument("--horizon", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--time-limit-sec", type=float, default=2.0)
    parser.add_argument("--goal-set-size", type=int, default=3)
    parser.add_argument("--release-interval", type=int, default=10)
    parser.add_argument("--debug", action="store_true", default=False)
    parser.add_argument("--anytime", action="store_true", default=False)
    parser.add_argument("--timeout-sec", type=int, default=900)
    parser.add_argument(
        "--workers",
        type=int,
        default=max(1, (os.cpu_count() or 1) - 10),
        help="parallel workers; default is cpu_count - 10",
    )
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.cache = args.out_dir / "symbotic_star.distcache"
    summary_path = args.out_dir / "throughput_sweep_results.csv"
    if summary_path.exists():
        summary_path.unlink()

    cases = [
        Case(distribution, outbound_prob, capacity, agents)
        for distribution, outbound_prob in CASES
        for capacity in parse_int_list(args.capacities)
        for agents in parse_int_list(args.agent_counts)
    ]
    workers = max(1, min(args.workers, len(cases)))
    print(
        f"running {len(cases)} cases with {workers} workers "
        f"(cpu_count={os.cpu_count()})",
        flush=True,
    )

    rows: list[dict[str, str]] = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case, args, case): case for case in cases}
        for future in as_completed(futures):
            case = futures[future]
            row = future.result()
            rows.append(row)
            rows.sort(
                key=lambda r: (
                    r["distribution"],
                    int(r["requested_capacity"]),
                    int(r["num_agents"]),
                )
            )
            write_summary(summary_path, rows)
            print(
                "finished "
                f"{case.distribution} K={case.capacity} A={case.agents} "
                f"throughput={row.get('throughput')} "
                f"valid={row.get('valid')} "
                f"elapsed={row.get('case_elapsed_sec')}s",
                flush=True,
            )

    write_summary(summary_path, rows)
    print(f"wrote {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
