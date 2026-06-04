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
    infer_seed,
    load_map,
    matrix_to_yaml,
    parse_matrix,
    parse_ir_stdout,
    yaml_to_matrix,
)


OPT_IR_SOLVERS = {
    "opt_dbs_hungarian": "dbs_hungarian",
    "opt_sbs_hungarian": "sbs_hungarian",
    "opt_random_hungarian": "random_hungarian",
    "opt_dbs_pibt": "dbs_pibt",
    "opt_sbs_pibt": "sbs_pibt",
    "opt_random_pibt": "random_pibt",
}


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


def parse_final_goals(stdout: str) -> list[int]:
    match = re.search(r"(?m)^final_goals:\s*(.*)$", stdout)
    if not match:
        return []
    goals: list[int] = []
    for token in match.group(1).split():
        if token == "None":
            return []
        goals.append(int(token))
    return goals


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
    seed = int(m.group(2)) if m else infer_seed(path)
    return {
        "suite": suite,
        "case_id": f"{path.parent.name}/{path.stem}",
        "fixture_file": str(path),
        "matrix_file": "",
        "map_file": "",
        "num_agents": int(m.group(1)) if m else "",
        "num_unique_tasks": "",
        "seed": seed,
    }


def matrix_case_meta(path: Path) -> dict[str, Any]:
    seed = infer_seed(path)
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
        "seed": seed,
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

    if not args.skip_ir_suite:
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


def fixed_goal_yaml_path(out_dir: Path, method: str, matrix: Path) -> Path:
    return (
        out_dir
        / "converted"
        / "fixed_goal_yaml"
        / matrix.parent.name
        / method
        / f"{matrix.stem}.yaml"
    )


def output_needs_refresh(output: Path, source: Path) -> bool:
    return not output.exists() or output.stat().st_mtime < source.stat().st_mtime


def write_fixed_goal_yaml(matrix: Path, goals: list[int], yaml_path: Path) -> None:
    parsed = parse_matrix(matrix)
    info = load_map(Path(parsed["metadata"]["map"]))
    starts = parsed["starts"]
    if len(starts) != len(goals):
        raise ValueError(f"goal count mismatch: starts={len(starts)} goals={len(goals)}")
    agents = []
    for i, (start_id, goal_id) in enumerate(zip(starts, goals)):
        start = info.id_to_coord[start_id]
        goal = info.id_to_coord[goal_id]
        agents.append(
            {
                "name": f"agent{i}",
                "start": [start[0], start[1]],
                "potentialGoals": [[goal[0], goal[1]]],
            }
        )
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        yaml.safe_dump({"agents": agents, "map": str(info.path)}, sort_keys=False),
        encoding="utf-8",
    )


def ensure_matrix(case: dict[str, Any], out_dir: Path) -> Path:
    matrix_value = str(case.get("matrix_file") or "")
    if matrix_value:
        matrix = Path(matrix_value)
        return matrix
    fixture = Path(case["fixture_file"])
    matrix = converted_matrix_path(out_dir, fixture)
    if not matrix.exists():
        yaml_to_matrix(fixture, matrix, None, seed=int(case.get("seed", 0)))
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
        str(int(case.get("seed", 0))),
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
            "time_limit": args.time_limit,
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
            "time_limit": args.time_limit,
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


def run_ir(
    case: dict[str, Any],
    args: argparse.Namespace,
    *,
    method: str = "ir",
    ir_solver: str | None = None,
    solver_prefix: str = "ir_tapf",
    time_limit: float | None = None,
) -> dict[str, Any]:
    matrix = ensure_matrix(case, args.out_dir)
    solver = ir_solver or args.ir_solver
    solve_limit = args.time_limit if time_limit is None else time_limit
    cmd = [
        str(args.ir_bin),
        "solve",
        "--matrix",
        str(matrix.resolve()),
        "--solver",
        solver,
        "--max-iterations",
        str(args.ir_max_iterations),
        "--time-limit-sec",
        str(solve_limit),
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
            "method": method,
            "solver": f"{solver_prefix}:{solver}",
            "ir_solver_arg": solver,
            "time_limit": solve_limit,
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "stderr": str(exc),
        }
    row = parse_ir_stdout(cp.stdout)
    final_goals = parse_final_goals(cp.stdout)
    row.update(
        {
            **case,
            "matrix_file": str(matrix),
            "method": method,
            "solver": f"{solver_prefix}:{solver}",
            "ir_solver_arg": solver,
            "final_goals": " ".join(str(g) for g in final_goals),
            "time_limit": solve_limit,
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


def fallback_opt_row(
    case: dict[str, Any],
    method: str,
    base_solver: str,
    ir_row: dict[str, Any],
    *,
    total_time_limit_s: float | None = None,
    first_error: str = "",
    final_opt_skipped_budget: int = 1,
) -> dict[str, Any]:
    row = {**case, **ir_row}
    row.update(
        {
            "method": method,
            "solver": f"ir_opt:{base_solver}",
            "ir_solver_arg": base_solver,
            "time_limit": total_time_limit_s if total_time_limit_s is not None else ir_row.get("time_limit", ""),
            "target_refinement_cost": ir_row.get("soc", ""),
            "target_refinement_wall_time_s": ir_row.get("wall_time_s", ""),
            "first_stage_time_limit_s": ir_row.get("time_limit", ""),
            "final_opt_time_limit_s": 0.0,
            "final_opt_skipped_budget": final_opt_skipped_budget,
        }
    )
    if first_error:
        row["first_error"] = first_error
    return row


def run_ir_final_opt(
    case: dict[str, Any],
    method: str,
    args: argparse.Namespace,
) -> dict[str, Any]:
    total_start = time.time()
    matrix = ensure_matrix(case, args.out_dir)
    base_solver = OPT_IR_SOLVERS[method]
    first_stage_limit_s = (
        args.ir_refinement_time_limit
        if args.ir_refinement_time_limit > 0
        else args.time_limit
    )
    ir_row = run_ir(
        case,
        args,
        method="ir_refinement",
        ir_solver=base_solver,
        solver_prefix="ir_tapf",
        time_limit=first_stage_limit_s,
    )
    if not int(ir_row.get("solved") or 0):
        return fallback_opt_row(
            case,
            method,
            base_solver,
            ir_row,
            total_time_limit_s=args.time_limit,
            first_error="target refinement failed",
        )

    final_goals = [int(token) for token in str(ir_row.get("final_goals", "")).split() if token]
    if not final_goals:
        return fallback_opt_row(
            case,
            method,
            base_solver,
            ir_row,
            total_time_limit_s=args.time_limit,
            first_error="missing final_goals from ir-tapf",
        )

    elapsed_s = time.time() - total_start
    final_opt_limit_s = max(0.0, args.time_limit - elapsed_s)
    if final_opt_limit_s <= 0.05:
        return fallback_opt_row(case, method, base_solver, ir_row, total_time_limit_s=args.time_limit)

    yaml_path = fixed_goal_yaml_path(args.out_dir, method, matrix)
    if output_needs_refresh(yaml_path, matrix):
        write_fixed_goal_yaml(matrix, final_goals, yaml_path)
    cmd = [
        str(args.lacam_bin),
        str(yaml_path),
        "",
        f"{final_opt_limit_s:.6f}",
        "",
        "1",
        "0",
        str(int(case.get("seed", 0))),
        "focal",
        str(args.focal_weight),
        "h",
    ]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            timeout=min(args.timeout, max(0.1, final_opt_limit_s + 2.0)),
        )
    except subprocess.TimeoutExpired as exc:
        total_wall_time_s = time.time() - total_start
        row = fallback_opt_row(
            case,
            method,
            base_solver,
            ir_row,
            total_time_limit_s=args.time_limit,
            first_error="final path optimization timed out",
            final_opt_skipped_budget=0,
        )
        row.update(
            {
                "fixture_file": str(yaml_path),
                "timed_out": 1,
                "external_timed_out": 1,
                "exit_code": 124,
                "wall_time_s": total_wall_time_s,
                "runtime_ms": total_wall_time_s * 1000.0,
                "stderr": str(exc),
                "final_opt_time_limit_s": final_opt_limit_s,
            }
        )
        return row

    lacam_row = parse_kv(cp.stdout)
    if cp.returncode != 0 or not int(lacam_row.get("solved") or 0):
        total_wall_time_s = time.time() - total_start
        row = fallback_opt_row(
            case,
            method,
            base_solver,
            ir_row,
            total_time_limit_s=args.time_limit,
            first_error="final path optimization failed",
            final_opt_skipped_budget=0,
        )
        row.update(
            {
                "fixture_file": str(yaml_path),
                "exit_code": cp.returncode,
                "wall_time_s": total_wall_time_s,
                "runtime_ms": total_wall_time_s * 1000.0,
                "stderr": cp.stderr.strip(),
                "final_opt_time_limit_s": final_opt_limit_s,
            }
        )
        return row

    total_wall_time_s = time.time() - total_start
    final_opt_runtime_ms = lacam_row.get("runtime_ms", "")
    lacam_row.update(
        {
            **case,
            "matrix_file": str(matrix),
            "fixture_file": str(yaml_path),
            "method": method,
            "solver": f"ir_opt:{base_solver}",
            "ir_solver_arg": base_solver,
            "search_mode_arg": "focal",
            "time_limit": args.time_limit,
            "exit_code": cp.returncode,
            "wall_time_s": total_wall_time_s,
            "runtime_ms": total_wall_time_s * 1000.0,
            "final_opt_runtime_ms": final_opt_runtime_ms,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
            "target_refinement_cost": ir_row.get("soc", ""),
            "target_refinement_wall_time_s": ir_row.get("wall_time_s", ""),
            "initial_solution_cost": ir_row.get("initial_solution_cost", ir_row.get("soc", "")),
            "initial_solution_time_ms": ir_row.get("initial_solution_time_ms", ""),
            "sum_shortest_distances": ir_row.get("sum_shortest_distances", ""),
            "final_goals": ir_row.get("final_goals", ""),
            "first_stage_time_limit_s": first_stage_limit_s,
            "final_opt_time_limit_s": final_opt_limit_s,
            "final_opt_skipped_budget": 0,
        }
    )
    lacam_row.setdefault("valid_solution", 0)
    lacam_row.setdefault("timed_out", 0)
    return lacam_row


def run_task(task: tuple[dict[str, Any], str], args: argparse.Namespace) -> dict[str, Any]:
    case, method = task
    if method == "ir":
        return run_ir(case, args)
    if method in OPT_IR_SOLVERS:
        return run_ir_final_opt(case, method, args)
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
    parser.add_argument(
        "--ir-refinement-time-limit",
        type=float,
        default=0.0,
        help=(
            "First-stage IR time for opt_* methods. Defaults to --time-limit, "
            "so final path optimization uses only leftover wall-clock budget."
        ),
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.add_argument("--max-cases-per-dir", type=int, default=0)
    parser.add_argument("--max-ir-cases", type=int, default=0)
    parser.add_argument("--skip-ir-suite", action="store_true")
    parser.add_argument(
        "--methods",
        nargs="+",
        default=["lacam_dfs", "lacam_focal_h", "ir"],
        choices=["lacam_dfs", "lacam_focal_h", "ir", *sorted(OPT_IR_SOLVERS)],
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/full_three_method"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    cases = discover_cases(args)
    methods = args.methods
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
                    "time_limit": args.time_limit,
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
