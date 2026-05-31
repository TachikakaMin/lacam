#!/usr/bin/env python3
"""Run DFS-LaCAM, FOCAL-LaCAM, and IR on full exp1/exp2/IR suites."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml

from run_ir_lacam_cross_experiment import (
    matrix_to_yaml,
    parse_ir_stdout,
    yaml_to_matrix,
)


def fixture_sort_key(path: Path) -> tuple[str, int, int, str]:
    m = re.search(r"agents_(\d+)_test_(\d+)\.yaml$", path.name)
    if not m:
        return (path.parent.name, 0, 0, path.name)
    return (path.parent.name, int(m.group(1)), int(m.group(2)), path.name)


def parse_kv(text: str) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
        if value in {"0", "1"}:
            row[key] = int(value)
            continue
        try:
            row[key] = int(value)
            continue
        except ValueError:
            pass
        try:
            row[key] = float(value)
            continue
        except ValueError:
            pass
        row[key] = value
    return row


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


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def yaml_case_meta(path: Path, suite: str) -> dict[str, Any]:
    m = re.search(r"agents_(\d+)_test_(\d+)\.yaml$", path.name)
    return {
        "suite": suite,
        "case_id": f"{path.parent.name}/{path.stem}",
        "fixture_file": str(path),
        "matrix_file": "",
        "map_file": "",
        "num_agents": int(m.group(1)) if m else "",
        "num_unique_tasks": "",
    }


def matrix_case_meta(path: Path) -> dict[str, Any]:
    return {
        "suite": "ir",
        "case_id": str(path.relative_to(path.parents[5]))
        if len(path.parents) > 5
        else str(path),
        "fixture_file": "",
        "matrix_file": str(path),
        "map_file": "",
        "num_agents": "",
        "num_unique_tasks": "",
    }


def discover_cases(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.all_itacbs_data:
        data_root = args.data_root
        args.exp1_dir.extend(
            sorted(
                p
                for p in data_root.iterdir()
                if p.is_dir() and p.name.startswith("Paper_") and p.name.endswith("_gp_5")
            )
        )
        args.exp2_dir.extend(
            sorted(p for p in data_root.iterdir() if p.is_dir() and "_ratio_" in p.name)
        )

    cases: list[dict[str, Any]] = []
    for suite, dirs in (("exp1", args.exp1_dir), ("exp2", args.exp2_dir)):
        for fixture_dir in dirs:
            paths = sorted(fixture_dir.glob("*.yaml"), key=fixture_sort_key)
            if args.max_cases_per_dir > 0:
                paths = paths[: args.max_cases_per_dir]
            for path in paths:
                cases.append(yaml_case_meta(path, suite))

    matrices = sorted(args.ir_matrix_root.glob("**/*.matrix"))
    if args.max_ir_cases > 0:
        matrices = matrices[: args.max_ir_cases]
    for matrix in matrices:
        cases.append(matrix_case_meta(matrix))
    return cases


def converted_matrix_path(out_dir: Path, fixture: Path) -> Path:
    return (
        out_dir
        / "converted"
        / "yaml_to_matrix"
        / fixture.parent.name
        / f"{fixture.stem}.matrix"
    )


def converted_yaml_path(out_dir: Path, matrix: Path) -> Path:
    return (
        out_dir
        / "converted"
        / "matrix_to_yaml"
        / matrix.parent.parent.name
        / matrix.parent.name
        / f"{matrix.stem}.yaml"
    )


def ensure_matrix(case: dict[str, Any], out_dir: Path) -> Path:
    matrix_value = str(case.get("matrix_file") or "")
    if matrix_value:
        matrix = Path(matrix_value)
        return matrix
    fixture = Path(case["fixture_file"])
    matrix = converted_matrix_path(out_dir, fixture)
    if not matrix.exists():
        yaml_to_matrix(fixture, matrix, None, seed=1)
    return matrix


def ensure_yaml(case: dict[str, Any], out_dir: Path) -> Path:
    fixture_value = str(case.get("fixture_file") or "")
    if fixture_value:
        fixture = Path(fixture_value)
        return fixture
    matrix = Path(case["matrix_file"])
    yaml_path = converted_yaml_path(out_dir, matrix)
    if not yaml_path.exists():
        matrix_to_yaml(matrix, yaml_path)
    return yaml_path


def run_lacam(
    case: dict[str, Any],
    method: str,
    args: argparse.Namespace,
) -> dict[str, Any]:
    fixture = ensure_yaml(case, args.out_dir)
    mode = "focal" if method == "lacam_focal_h" else "dfs"
    cmd = [
        str(args.lacam_bin),
        str(fixture),
        "",
        str(args.time_limit),
        "",
        "1",
        "0",
        "-1",
        mode,
        str(args.focal_weight),
        "h",
    ]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=args.timeout)
    except subprocess.TimeoutExpired as exc:
        return {
            **case,
            "method": method,
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
            "fixture_file": str(fixture),
            "method": method,
            "solver": "lacam_tapf",
            "search_mode_arg": mode,
            "exit_code": cp.returncode,
            "wall_time_s": time.time() - start,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    row.setdefault("timed_out", 0)
    return row


def run_ir(case: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    matrix = ensure_matrix(case, args.out_dir)
    cmd = [
        str(args.ir_bin),
        "solve",
        "--matrix",
        str(matrix.resolve()),
        "--solver",
        args.ir_solver,
        "--max-iterations",
        str(args.ir_max_iterations),
        "--time-limit-sec",
        str(args.time_limit),
    ]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd, cwd=args.ir_repo, text=True, capture_output=True, timeout=args.timeout
        )
    except subprocess.TimeoutExpired as exc:
        return {
            **case,
            "matrix_file": str(matrix),
            "method": "ir",
            "solver": f"ir_tapf:{args.ir_solver}",
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
            "matrix_file": str(matrix),
            "method": "ir",
            "solver": f"ir_tapf:{args.ir_solver}",
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


def run_task(task: tuple[dict[str, Any], str], args: argparse.Namespace) -> dict[str, Any]:
    case, method = task
    if method == "ir":
        return run_ir(case, args)
    return run_lacam(case, method, args)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((str(row["suite"]), str(row["method"])), []).append(row)
    summary = []
    for (suite, method), group in sorted(groups.items()):
        solved = [r for r in group if int(r.get("solved") or 0)]
        summary.append(
            {
                "suite": suite,
                "method": method,
                "cases": len(group),
                "solved": len(solved),
                "solve_rate": len(solved) / len(group) if group else math.nan,
                "mean_soc": mean([safe_float(r.get("soc")) for r in solved]),
                "mean_sum_of_loss": mean(
                    [safe_float(r.get("sum_of_loss")) for r in solved]
                ),
                "mean_runtime_ms": mean(
                    [safe_float(r.get("runtime_ms")) for r in solved]
                ),
                "mean_wall_time_s": mean(
                    [safe_float(r.get("wall_time_s")) for r in group]
                ),
                "external_timeouts": sum(
                    int(r.get("external_timed_out") or 0) for r in group
                ),
            }
        )
    write_csv(path, summary)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exp1-dir", type=Path, action="append", default=[])
    parser.add_argument("--exp2-dir", type=Path, action="append", default=[])
    parser.add_argument(
        "--data-root",
        type=Path,
        default=Path("/media/project0/yimin/lacam_tapf_itacbs_data"),
    )
    parser.add_argument("--all-itacbs-data", action="store_true")
    parser.add_argument(
        "--ir-matrix-root",
        type=Path,
        default=Path("/home/yimin/research/ir-tapf/matrix"),
    )
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--ir-repo", type=Path, default=Path("/home/yimin/research/ir-tapf"))
    parser.add_argument(
        "--ir-bin",
        type=Path,
        default=Path("/home/yimin/research/ir-tapf/target/release/ir_tapf"),
    )
    parser.add_argument("--ir-solver", default="dbs_hungarian")
    parser.add_argument("--ir-max-iterations", type=int, default=100000)
    parser.add_argument("--time-limit", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.add_argument("--max-cases-per-dir", type=int, default=0)
    parser.add_argument("--max-ir-cases", type=int, default=0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/full_three_method"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    cases = discover_cases(args)
    methods = ["lacam_dfs", "lacam_focal_h", "ir"]
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

    tasks = [
        (case, method)
        for case in cases
        for method in methods
        if (case["case_id"], method) not in completed
    ]
    print(
        f"cases={len(cases)} tasks={len(tasks)} completed={len(rows)} "
        f"jobs={args.jobs} time_limit={args.time_limit}s",
        flush=True,
    )

    if rows:
        write_csv(rows_csv, rows)
        write_summary(summary_csv, rows)

    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = {executor.submit(run_task, task, args): task for task in tasks}
        for done, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            case, method = futures[future]
            try:
                row = future.result()
            except Exception as exc:
                row = {
                    **case,
                    "method": method,
                    "solver": method,
                    "solved": 0,
                    "valid_solution": 0,
                    "timed_out": 0,
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
                f"[{done}/{len(tasks)}] {row['suite']} {row['case_id']} "
                f"{method} solved={row.get('solved', 0)} soc={row.get('soc', '')} "
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
