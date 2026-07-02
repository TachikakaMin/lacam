#!/usr/bin/env python3
"""Run the requested symbotic lifelong TAPF parameter grid."""

from __future__ import annotations

import argparse
import csv
import itertools
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path


MAPS = {
    "symbotic": "tests/assets/symbotic.map",
    "symbotic_star": "tests/assets/symbotic_star.map",
}
DISTS = {
    "50_50": 0.5,
    "80_20": 0.8,
    "20_80": 0.2,
}


@dataclass(frozen=True)
class Case:
    map_name: str
    k: int
    slot: int
    agents: int
    dist_label: str
    duration: int
    cost_mode: int = 0
    seed: int = 0

    @property
    def outbound_prob(self) -> float:
        return DISTS[self.dist_label]

    @property
    def label(self) -> str:
        return (
            f"map-{self.map_name}__k-{self.k}__slot-{self.slot}"
            f"__agents-{self.agents}__dist-{self.dist_label}"
            f"__dur-{self.duration}__cost-{self.cost_mode}"
            f"__seed-{self.seed}"
        )


def parse_ints(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item]


def parse_seed_values(args: argparse.Namespace) -> list[int]:
    seeds = getattr(args, "seeds", None)
    if seeds:
        return parse_ints(seeds)
    return [int(args.seed)]


def build_cases(args: argparse.Namespace) -> list[Case]:
    maps = [item for item in args.maps.split(",") if item]
    dists = [item for item in args.dists.split(",") if item]
    unknown_maps = sorted(set(maps) - set(MAPS))
    unknown_dists = sorted(set(dists) - set(DISTS))
    if unknown_maps:
        raise ValueError(f"unknown map labels: {unknown_maps}")
    if unknown_dists:
        raise ValueError(f"unknown distribution labels: {unknown_dists}")

    return [
        Case(map_name, k, slot, agents, dist_label, duration, cost_mode, seed)
        for map_name, k, slot, agents, dist_label, duration, cost_mode, seed in itertools.product(
            maps,
            parse_ints(args.ks),
            parse_ints(args.slots),
            parse_ints(args.agent_counts),
            dists,
            parse_ints(args.durations),
            parse_ints(args.cost_modes),
            parse_seed_values(args),
        )
    ]


def read_single_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise RuntimeError(f"expected one row in {path}, got {len(rows)}")
    return rows[0]


def reusable_result(path: Path) -> bool:
    if not path.exists():
        return False
    try:
        row = read_single_row(path)
    except Exception:
        return False
    return row.get("valid") in {"0", "1"}


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    if not rows:
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    tmp.replace(path)


def run_case(args: argparse.Namespace, case: Case) -> dict[str, str]:
    run_dir = args.out_dir / "runs" / case.label
    run_dir.mkdir(parents=True, exist_ok=True)

    result_csv = run_dir / "result.csv"
    trace_csv = Path(str(result_csv) + ".trace.csv")
    schedule_yaml = run_dir / "schedule.yaml"
    stdout_path = run_dir / "stdout.txt"
    stderr_path = run_dir / "stderr.txt"
    cache_path = args.out_dir / "cache" / f"{case.map_name}.distcache"
    cache_path.parent.mkdir(parents=True, exist_ok=True)

    if reusable_result(result_csv) and not args.force:
        row = read_single_row(result_csv)
        elapsed = 0.0
        returncode = 0
        skipped_existing = "1"
    else:
        for path in (
            result_csv,
            trace_csv,
            schedule_yaml,
            Path(str(schedule_yaml) + ".bin"),
            stdout_path,
            stderr_path,
        ):
            if path.exists():
                path.unlink()

        cmd = [
            str(args.binary),
            MAPS[case.map_name],
            str(case.agents),
            str(args.horizon),
            str(case.seed),
            str(result_csv),
            str(cache_path),
            str(args.time_limit_sec),
            str(args.goal_set_size),
            str(case.outbound_prob),
            str(args.release_interval),
            "0",
            str(schedule_yaml),
            "1",
            str(case.k),
            "0",
            str(args.service_commit_agents),
            str(case.slot),
            str(case.duration),
            str(case.duration),
            str(case.cost_mode),
        ]
        start = time.time()
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=args.timeout_sec,
        )
        elapsed = time.time() - start
        returncode = proc.returncode
        skipped_existing = "0"
        stdout_path.write_text(proc.stdout, encoding="utf-8")
        stderr_path.write_text(proc.stderr, encoding="utf-8")
        if proc.returncode != 0 or not result_csv.exists():
            raise RuntimeError(
                f"{case.label} failed with returncode={proc.returncode}\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )
        row = read_single_row(result_csv)

    row.update(
        {
            "map": case.map_name,
            "k": str(case.k),
            "slot": str(case.slot),
            "agents": str(case.agents),
            "dist_label": case.dist_label,
            "outbound_prob": str(case.outbound_prob),
            "duration": str(case.duration),
            "cost_mode": str(case.cost_mode),
            "time_limit_sec": str(args.time_limit_sec),
            "horizon_requested": str(args.horizon),
            "seed_requested": str(case.seed),
            "goal_set_size_requested": str(args.goal_set_size),
            "release_interval_requested": str(args.release_interval),
            "anytime_requested": "1",
            "service_commit_agents_requested": str(args.service_commit_agents),
            "case_elapsed_sec": f"{elapsed:.6f}",
            "returncode": str(returncode),
            "skipped_existing": skipped_existing,
            "run_status": "done",
        }
    )
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("./build/lifelong_benchmark"))
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("./tmp_runs/requested_symbotic_grid_current"),
    )
    parser.add_argument("--maps", default="symbotic,symbotic_star")
    parser.add_argument("--ks", default="1,2,4")
    parser.add_argument("--slots", default="1,2,3")
    parser.add_argument("--agent-counts", default="25,50,100,150,200")
    parser.add_argument("--dists", default="50_50,80_20")
    parser.add_argument("--durations", default="0,1,2,4,8")
    parser.add_argument("--cost-modes", default="0")
    parser.add_argument("--horizon", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--seeds",
        default="",
        help="Comma-separated seeds. When set, overrides --seed.",
    )
    parser.add_argument("--time-limit-sec", type=float, default=1.0)
    parser.add_argument("--goal-set-size", type=int, default=3)
    parser.add_argument("--release-interval", type=int, default=10)
    parser.add_argument("--service-commit-agents", type=int, default=0)
    parser.add_argument("--timeout-sec", type=int, default=120)
    parser.add_argument("--workers", type=int, default=min(32, os.cpu_count() or 1))
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    if not args.binary.exists():
        raise FileNotFoundError(f"benchmark binary not found: {args.binary}")

    cases = build_cases(args)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.out_dir / "all_results.csv"
    workers = max(1, min(args.workers, len(cases)))
    print(f"running {len(cases)} cases with {workers} workers", flush=True)

    rows: list[dict[str, str]] = []
    completed = 0
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case, args, case): case for case in cases}
        for future in as_completed(futures):
            case = futures[future]
            row = future.result()
            rows.append(row)
            completed += 1
            rows.sort(
                key=lambda r: (
                    r["map"],
                    int(r["k"]),
                    int(r["slot"]),
                    int(r["agents"]),
                    r["dist_label"],
                    int(r["duration"]),
                    int(r["cost_mode"]),
                    int(r["seed_requested"]),
                )
            )
            write_rows(summary_path, rows)
            print(
                f"[{completed}/{len(cases)}] {case.label} "
                f"throughput={row.get('throughput')} "
                f"valid={row.get('valid')} skipped={row['skipped_existing']}",
                flush=True,
            )

    write_rows(summary_path, rows)
    print(f"wrote {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
