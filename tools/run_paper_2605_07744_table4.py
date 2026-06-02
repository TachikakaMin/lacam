#!/usr/bin/env python3
"""Run arXiv 2605.07744 Table 4 style ITA-ECBS comparison.

The paper compares ITA-ECBS against DBS-Hungarian on small TAPF instances.
This runner uses the ITA-CBS2 YAML fixtures, runs ITA-ECBS directly on those
fixtures, converts the same YAML files to ir-tapf matrices for DBS-Hungarian,
and also records LaCAM-TAPF rows for this repository.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path
from typing import Any

import yaml

from run_ir_lacam_cross_experiment import parse_ir_stdout, parse_kv, yaml_to_matrix


TABLE4_DIRS = {
    "random": "paper_random_32_32",
    "hotspot": "paper_random_32_32_for_foundation",
    "warehouse_random": "paper_warehouse_161_63",
    "warehouse_hotspot": "paper_warehouse_161_63_2",
}


def safe_float(value: Any) -> float:
    try:
        if value == "" or value is None:
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return sum(finite) / len(finite) if finite else math.nan


def fixture_sort_key(path: Path) -> tuple[int, int, str]:
    match = re.search(r"_agents_(\d+)_test_(\d+)\.yaml$", path.name)
    if not match:
        return (0, 0, path.name)
    return (int(match.group(1)), int(match.group(2)), path.name)


def fixture_meta(path: Path) -> tuple[int, int]:
    match = re.search(r"_agents_(\d+)_test_(\d+)\.yaml$", path.name)
    if not match:
        raise ValueError(f"cannot parse fixture name: {path}")
    return int(match.group(1)), int(match.group(2))


def discover_cases(args: argparse.Namespace) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    selected_modes = list(TABLE4_DIRS) if args.modes == ["all"] else args.modes
    for mode in selected_modes:
        fixture_dir = args.ita_fixture_root / TABLE4_DIRS[mode]
        fixtures = sorted(fixture_dir.glob("*.yaml"), key=fixture_sort_key)
        for fixture in fixtures:
            agents, test_id = fixture_meta(fixture)
            if args.agent_counts and agents not in args.agent_counts:
                continue
            if args.max_tests_per_agents > 0 and test_id >= args.max_tests_per_agents:
                continue
            with fixture.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            case_id = f"{mode}/{fixture.stem}"
            matrix = args.out_dir / "yaml_to_matrix" / mode / f"{fixture.stem}.matrix"
            meta = yaml_to_matrix(fixture, matrix, None, seed=test_id + 1)
            cases.append(
                {
                    "suite": "table4",
                    "scenario": f"table4_{mode}",
                    "target_mode": mode,
                    "case_id": case_id,
                    "fixture_file": str(fixture),
                    "matrix_file": str(matrix),
                    "map_file": str((fixture.parent / data["map"]).resolve()),
                    "agents": agents,
                    "seed": test_id,
                    "num_unique_tasks": meta["num_unique_tasks"],
                }
            )
    if args.max_cases > 0:
        cases = cases[: args.max_cases]
    return cases


def run_ita_ecbs(case: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    output = args.out_dir / "ita_outputs" / f"{case['case_id'].replace('/', '__')}.yaml"
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(args.ita_ecbs_bin),
        "-i",
        case["fixture_file"],
        "-o",
        str(output.resolve()),
        "-w",
        str(args.ecbs_weight),
    ]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd,
            cwd=args.ita_repo,
            text=True,
            capture_output=True,
            timeout=args.timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            **case,
            "method": "ita_ecbs",
            "solver": "ita_ecbs",
            "output_file": str(output),
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "stderr": str(exc),
        }

    row: dict[str, Any] = {}
    if output.exists() and output.stat().st_size > 0:
        try:
            with output.open("r", encoding="utf-8") as f:
                stats = (yaml.safe_load(f) or {}).get("statistics", {})
            cost = int(stats.get("cost", -1))
            row.update(
                {
                    "soc": cost if cost >= 0 else "",
                    "runtime_ms": safe_float(stats.get("runtime")) * 1000.0,
                    "ta_runtime_ms": safe_float(stats.get("TA_runtime")) * 1000.0,
                    "lowlevel_search_time_ms": safe_float(stats.get("lowlevel_search_time")) * 1000.0,
                    "numTaskAssignments": stats.get("numTaskAssignments", ""),
                    "total_lowlevel_node": stats.get("total_lowlevel_node", ""),
                    "lowLevelExpanded": stats.get("lowLevelExpanded", ""),
                }
            )
            row["solved"] = int(cost >= 0)
            row["valid_solution"] = int(cost >= 0)
        except Exception as exc:
            row["first_error"] = repr(exc)

    row.update(
        {
            **case,
            "method": "ita_ecbs",
            "solver": "ita_ecbs",
            "output_file": str(output),
            "exit_code": cp.returncode,
            "wall_time_s": time.time() - start,
            "external_timed_out": 0,
            "timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    if cp.returncode != 0:
        row["solved"] = 0
        row["valid_solution"] = 0
    return row


def run_ir(case: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    cmd = [
        str(args.ir_bin),
        "solve",
        "--matrix",
        str(Path(case["matrix_file"]).resolve()),
        "--solver",
        "dbs_hungarian",
        "--max-iterations",
        str(args.max_iterations),
        "--time-limit-sec",
        str(args.time_limit),
    ]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd,
            cwd=args.ir_repo,
            text=True,
            capture_output=True,
            timeout=args.timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            **case,
            "method": "dbs_hungarian",
            "solver": "ir_tapf:dbs_hungarian",
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "stderr": str(exc),
        }
    row = parse_ir_stdout(cp.stdout)
    row.update(
        {
            **case,
            "method": "dbs_hungarian",
            "solver": "ir_tapf:dbs_hungarian",
            "exit_code": cp.returncode,
            "wall_time_s": time.time() - start,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    if cp.returncode != 0:
        row["solved"] = 0
        row["valid_solution"] = 0
    return row


def run_lacam(case: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    cmd = [
        str(args.lacam_bin),
        case["fixture_file"],
        "",
        str(args.time_limit),
        "",
        "1",
        "0",
        "-1",
        "focal",
        str(args.focal_weight),
        "h",
    ]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=args.timeout)
    except subprocess.TimeoutExpired as exc:
        return {
            **case,
            "method": "lacam_focal_h",
            "solver": "lacam_tapf",
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "stderr": str(exc),
        }
    row = parse_kv(cp.stdout)
    row.update(
        {
            **case,
            "method": "lacam_focal_h",
            "solver": "lacam_tapf",
            "exit_code": cp.returncode,
            "wall_time_s": time.time() - start,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
            "search_mode_arg": "focal",
            "focal_weight_arg": args.focal_weight,
            "focal_tie_break_arg": "h",
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    row.setdefault("timed_out", 0)
    return row


def run_task(task: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    method = task["method"]
    case = task["case"]
    if method == "ita_ecbs":
        return run_ita_ecbs(case, args)
    if method == "dbs_hungarian":
        return run_ir(case, args)
    if method == "lacam_focal_h":
        return run_lacam(case, args)
    raise ValueError(method)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = (row["scenario"], row["target_mode"], row["method"], row["agents"])
        groups.setdefault(key, []).append(row)
    out = []
    for key, group in sorted(groups.items()):
        solved = [r for r in group if int(r.get("solved") or 0)]
        out.append(
            {
                "scenario": key[0],
                "target_mode": key[1],
                "method": key[2],
                "agents": key[3],
                "cases": len(group),
                "solved": len(solved),
                "solve_rate": len(solved) / len(group) if group else math.nan,
                "mean_soc": mean([safe_float(r.get("soc")) for r in solved]),
                "mean_wall_time_s": mean([safe_float(r.get("wall_time_s")) for r in group]),
                "external_timeouts": sum(int(r.get("external_timed_out") or 0) for r in group),
            }
        )
    write_csv(path, out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/paper_2605_07744_table4"))
    parser.add_argument("--ita-repo", type=Path, default=Path("/home/yimin/research/ITA-CBS2"))
    parser.add_argument("--ita-ecbs-bin", type=Path, default=Path("/home/yimin/research/ITA-CBS2/build/ITA_ECBS_v2"))
    parser.add_argument("--ita-fixture-root", type=Path, default=Path("/home/yimin/research/ITA-CBS2/map_file_ecbs"))
    parser.add_argument("--ir-repo", type=Path, default=Path("/home/yimin/research/ir-tapf"))
    parser.add_argument("--ir-bin", type=Path, default=Path("/home/yimin/research/ir-tapf/target/release/ir_tapf"))
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--modes", nargs="+", choices=["all", *TABLE4_DIRS.keys()], default=["all"])
    parser.add_argument("--methods", nargs="+", choices=["ita_ecbs", "dbs_hungarian", "lacam_focal_h"], default=["ita_ecbs", "dbs_hungarian", "lacam_focal_h"])
    parser.add_argument("--agent-counts", type=int, nargs="*", default=[])
    parser.add_argument("--max-tests-per-agents", type=int, default=0)
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument("--time-limit", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--max-iterations", type=int, default=100000)
    parser.add_argument("--ecbs-weight", type=float, default=1.1)
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    cases = discover_cases(args)
    tasks = [{"case": case, "method": method} for case in cases for method in args.methods]

    rows: list[dict[str, Any]] = []
    completed: set[tuple[str, str]] = set()
    if args.resume and rows_jsonl.exists():
        with rows_jsonl.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                row = json.loads(line)
                rows.append(row)
                completed.add((str(row["case_id"]), str(row["method"])))
    elif rows_jsonl.exists():
        rows_jsonl.unlink()

    tasks = [t for t in tasks if (t["case"]["case_id"], t["method"]) not in completed]
    print(f"cases={len(cases)} tasks={len(tasks)} completed={len(rows)} jobs={args.jobs}", flush=True)

    if rows:
        write_csv(rows_csv, rows)
        write_summary(summary_csv, rows)

    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = {executor.submit(run_task, task, args): task for task in tasks}
        for done, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            task = futures[future]
            try:
                row = future.result()
            except Exception as exc:
                case = task["case"]
                row = {
                    **case,
                    "method": task["method"],
                    "solver": task["method"],
                    "solved": 0,
                    "valid_solution": 0,
                    "external_timed_out": 0,
                    "exit_code": -1,
                    "wall_time_s": 0,
                    "stderr": repr(exc),
                    "first_error": "runner exception",
                }
            rows.append(row)
            with rows_jsonl.open("a", encoding="utf-8") as f:
                f.write(json.dumps(row, sort_keys=True) + "\n")
            print(
                f"[{done}/{len(tasks)}] {row['case_id']} {row['method']} "
                f"solved={row.get('solved', 0)} soc={row.get('soc', '')} "
                f"wall={safe_float(row.get('wall_time_s')):.2f}s",
                flush=True,
            )

    write_csv(rows_csv, rows)
    write_summary(summary_csv, rows)
    print(f"wrote {rows_csv}", flush=True)
    print(f"wrote {summary_csv}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
