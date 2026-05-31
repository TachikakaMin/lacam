#!/usr/bin/env python3
"""Run small cross experiments between LaCAM-TAPF and ir-tapf.

The script runs two suites:

1. Existing LaCAM/ITA-CBS YAML TAPF fixtures converted to ir-tapf matrix files.
2. ir-tapf generated matrix fixtures converted to LaCAM/ITA-CBS YAML files.

Rows are appended to JSONL as each solver finishes, then consolidated to CSV
and summary CSV files.
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
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple

import yaml


Coord = Tuple[int, int]


@dataclass(frozen=True)
class MapInfo:
    path: Path
    width: int
    height: int
    coord_to_id: dict[Coord, int]
    id_to_coord: dict[int, Coord]


_MAP_CACHE: dict[Path, MapInfo] = {}


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_map(path: Path) -> MapInfo:
    path = path.resolve()
    cached = _MAP_CACHE.get(path)
    if cached is not None:
        return cached

    lines = path.read_text(encoding="utf-8").splitlines()
    width = 0
    height = 0
    map_start = None
    for i, line in enumerate(lines):
        if line.startswith("height"):
            height = int(line.split()[1])
        elif line.startswith("width"):
            width = int(line.split()[1])
        elif line == "map":
            map_start = i + 1
            break
    if map_start is None or width <= 0 or height <= 0:
        raise ValueError(f"invalid MovingAI map: {path}")

    coord_to_id: dict[Coord, int] = {}
    id_to_coord: dict[int, Coord] = {}
    vertex_id = 0
    for r in range(height):
        row = lines[map_start + r].rstrip("\r")
        for c in range(width):
            if row[c] in {"@", "T"}:
                continue
            coord = (r, c)
            coord_to_id[coord] = vertex_id
            id_to_coord[vertex_id] = coord
            vertex_id += 1

    info = MapInfo(
        path=path.resolve(),
        width=width,
        height=height,
        coord_to_id=coord_to_id,
        id_to_coord=id_to_coord,
    )
    _MAP_CACHE[path] = info
    return info


def resolve_yaml_map(yaml_path: Path, map_dir: Path | None) -> Path:
    data = load_yaml(yaml_path)
    map_value = Path(str(data["map"]))
    if map_value.is_absolute():
        return map_value
    if map_dir is not None:
        return (map_dir / map_value).resolve()
    return (yaml_path.parent / map_value).resolve()


def fixture_sort_key(path: Path) -> tuple[str, int, int, str]:
    m = re.search(r"agents_(\d+)_test_(\d+)\.yaml$", path.name)
    if not m:
        return (path.parent.name, 0, 0, path.name)
    return (path.parent.name, int(m.group(1)), int(m.group(2)), path.name)


def discover_yaml_fixtures(fixture_dirs: list[Path], max_cases: int) -> list[Path]:
    fixtures: list[Path] = []
    for fixture_dir in fixture_dirs:
        fixtures.extend(sorted(fixture_dir.glob("*.yaml"), key=fixture_sort_key))
    fixtures = sorted(fixtures, key=fixture_sort_key)
    return fixtures[:max_cases] if max_cases > 0 else fixtures


def yaml_goals(agent: dict[str, Any]) -> list[Coord]:
    goals = agent.get("potentialGoals")
    if goals is None:
        goals = agent.get("goal")
    if goals is None:
        return []
    if (
        isinstance(goals, list)
        and len(goals) == 2
        and all(isinstance(v, (int, float)) for v in goals)
    ):
        return [(int(goals[0]), int(goals[1]))]
    return [(int(g[0]), int(g[1])) for g in goals]


def yaml_to_matrix(
    yaml_path: Path,
    matrix_path: Path,
    map_dir: Path | None,
    seed: int,
) -> dict[str, Any]:
    data = load_yaml(yaml_path)
    map_path = resolve_yaml_map(yaml_path, map_dir)
    info = load_map(map_path)
    agents = data.get("agents") or []

    starts: list[int] = []
    target_rows: list[list[int]] = []
    for i, agent in enumerate(agents):
        start = (int(agent["start"][0]), int(agent["start"][1]))
        if start not in info.coord_to_id:
            raise ValueError(f"{yaml_path}: agent {i} start on obstacle: {start}")
        starts.append(info.coord_to_id[start])

        targets = []
        for goal in yaml_goals(agent):
            if goal not in info.coord_to_id:
                raise ValueError(f"{yaml_path}: agent {i} goal on obstacle: {goal}")
            targets.append(info.coord_to_id[goal])
        target_rows.append(sorted(set(targets)))

    matrix_path.parent.mkdir(parents=True, exist_ok=True)
    with matrix_path.open("w", encoding="utf-8") as f:
        f.write(f"# seed: {seed}\n")
        f.write(f"# map: {info.path}\n")
        f.write(f"# n: {len(agents)}\n")
        f.write("Start positions: " + " ".join(str(v) for v in starts) + "\n")
        for targets in target_rows:
            f.write(" ".join(str(v) for v in targets) + "\n")

    unique_targets = {g for row in target_rows for g in row}
    return {
        "num_agents": len(agents),
        "num_unique_tasks": len(unique_targets),
        "map_file": str(info.path),
    }


def parse_matrix(matrix_path: Path) -> dict[str, Any]:
    lines = matrix_path.read_text(encoding="utf-8").splitlines()
    metadata: dict[str, str] = {}
    data_start = None
    for i, line in enumerate(lines):
        if line.startswith("#") and ":" in line:
            key, value = line[1:].split(":", 1)
            metadata[key.strip()] = value.strip()
        elif line.startswith("Start positions:"):
            data_start = i
            break
    if data_start is None:
        raise ValueError(f"invalid matrix file: {matrix_path}")
    starts = [
        int(tok)
        for tok in lines[data_start][len("Start positions:") :].split()
    ]
    target_rows: list[list[int]] = []
    for line in lines[data_start + 1 : data_start + 1 + len(starts)]:
        target_rows.append([int(tok) for tok in line.split()])
    return {"metadata": metadata, "starts": starts, "targets": target_rows}


def matrix_to_yaml(matrix_path: Path, yaml_path: Path) -> dict[str, Any]:
    matrix = parse_matrix(matrix_path)
    map_path = Path(matrix["metadata"]["map"]).resolve()
    info = load_map(map_path)
    starts: list[int] = matrix["starts"]
    target_rows: list[list[int]] = matrix["targets"]

    agents = []
    for i, start_id in enumerate(starts):
        start = info.id_to_coord[start_id]
        goals = [info.id_to_coord[g] for g in sorted(set(target_rows[i]))]
        agents.append(
            {
                "name": f"agent{i}",
                "start": [start[0], start[1]],
                "potentialGoals": [[r, c] for r, c in goals],
            }
        )

    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        yaml.safe_dump({"agents": agents, "map": str(info.path)}, sort_keys=False),
        encoding="utf-8",
    )
    unique_targets = {g for row in target_rows for g in row}
    return {
        "num_agents": len(starts),
        "num_unique_tasks": len(unique_targets),
        "map_file": str(info.path),
    }


def parse_kv(stdout: str) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        try:
            row[key] = float(value) if "." in value else int(value)
        except ValueError:
            row[key] = value
    return row


def parse_ir_stdout(stdout: str) -> dict[str, Any]:
    def find_float(pattern: str) -> float:
        m = re.search(pattern, stdout)
        return float(m.group(1)) if m else math.nan

    def find_int(pattern: str) -> int | None:
        m = re.search(pattern, stdout)
        return int(m.group(1)) if m else None

    solved = "Solution found!" in stdout
    row: dict[str, Any] = {
        "solved": int(solved),
        "valid_solution": int(solved),
        "collision_free": "",
        "soc": find_int(r"(?m)^soc: (\d+)") if solved else "",
        "makespan": "",
        "runtime_ms": find_float(r"(?m)^Time Taken: ([\d.]+)") * 1000.0
        if solved
        else "",
        "iterations_used": find_int(r"(?m)^Iterations used: (\d+)") if solved else "",
        "initial_solution_time_ms": find_float(
            r"(?m)^initial_solution_time_ms: ([\d.]+)"
        )
        if solved
        else "",
        "initial_solution_cost": find_int(r"(?m)^initial_solution_cost: (\d+)")
        if solved
        else "",
        "iterations_time_ms": find_float(r"(?m)^iterations_time_ms: ([\d.]+)")
        if solved
        else "",
        "refinement_time_ms": find_float(r"(?m)^refinement_time_ms: ([\d.]+)")
        if solved
        else "",
        "refinement_input_cost": find_int(r"(?m)^refinement_input_cost: (\d+)")
        if solved
        else "",
        "sum_shortest_distances": find_int(r"(?m)^sum_shortest_distances: (\d+)"),
    }
    return row


def run_lacam(
    lacam_bin: Path,
    fixture: Path,
    time_limit: float,
    timeout: float,
    anytime: bool,
    full_ta: bool,
    search_mode: str,
    focal_weight: float,
    focal_tie_break: str,
) -> dict[str, Any]:
    cmd = [
        str(lacam_bin),
        str(fixture),
        "",
        str(time_limit),
        "",
        "1" if anytime else "0",
        "1" if full_ta else "0",
        "-1",
        search_mode,
        str(focal_weight),
        focal_tie_break,
    ]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        return {
            "solver": "lacam_tapf",
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "wall_time_s": time.time() - start,
            "exit_code": 124,
            "stderr": str(exc),
            "search_mode": search_mode,
            "focal_weight": focal_weight,
            "focal_tie_break": focal_tie_break,
        }
    row = parse_kv(cp.stdout)
    row.update(
        {
            "solver": "lacam_tapf",
            "wall_time_s": time.time() - start,
            "exit_code": cp.returncode,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
            "search_mode_arg": search_mode,
            "focal_weight_arg": focal_weight,
            "focal_tie_break_arg": focal_tie_break,
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    row.setdefault("timed_out", 0)
    return row


def run_ir(
    ir_bin: Path,
    ir_repo: Path,
    matrix: Path,
    solver: str,
    max_iterations: int,
    time_limit: float,
    timeout: float,
) -> dict[str, Any]:
    cmd = [
        str(ir_bin),
        "solve",
        "--matrix",
        str(matrix),
        "--solver",
        solver,
        "--max-iterations",
        str(max_iterations),
        "--time-limit-sec",
        str(time_limit),
    ]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd, cwd=ir_repo, text=True, capture_output=True, timeout=timeout
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "solver": f"ir_tapf:{solver}",
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "wall_time_s": time.time() - start,
            "exit_code": 124,
            "stderr": str(exc),
        }
    row = parse_ir_stdout(cp.stdout)
    row.update(
        {
            "solver": f"ir_tapf:{solver}",
            "wall_time_s": time.time() - start,
            "exit_code": cp.returncode,
            "timed_out": int(cp.returncode == 124),
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    if cp.returncode != 0:
        row["solved"] = 0
        row["valid_solution"] = 0
    return row


def fixture_base(case: dict[str, Any]) -> dict[str, Any]:
    return {
        "suite": case["suite"],
        "case_id": case["case_id"],
        "fixture_file": str(case["yaml"]),
        "matrix_file": str(case["matrix"]),
        "map_file": case["map_file"],
        "num_agents": case["num_agents"],
        "num_unique_tasks": case["num_unique_tasks"],
    }


def run_task(task: tuple[dict[str, Any], str], args: argparse.Namespace) -> dict[str, Any]:
    case, solver_kind = task
    if solver_kind == "lacam":
        result = run_lacam(
            args.lacam_bin.resolve(),
            case["yaml"].resolve(),
            args.time_limit,
            args.timeout,
            args.lacam_anytime,
            args.lacam_full_ta,
            args.lacam_search_mode,
            args.lacam_focal_weight,
            args.lacam_focal_tie_break,
        )
    elif solver_kind == "ir":
        result = run_ir(
            args.ir_bin.resolve(),
            args.ir_repo.resolve(),
            case["matrix"].resolve(),
            args.ir_solver,
            args.ir_max_iterations,
            args.time_limit,
            args.timeout,
        )
    else:
        raise ValueError(f"unknown solver kind: {solver_kind}")
    return {**fixture_base(case), **result}


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({k for row in rows for k in row})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def safe_float(value: Any) -> float:
    try:
        if value == "" or value is None:
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((str(row["suite"]), str(row["solver"])), []).append(row)

    summary_rows = []
    for (suite, solver), group in sorted(groups.items()):
        solved = [r for r in group if int(r.get("solved") or 0)]
        soc_values = [safe_float(r.get("soc")) for r in solved]
        wall_values = [safe_float(r.get("wall_time_s")) for r in group]
        runtime_values = [safe_float(r.get("runtime_ms")) for r in solved]
        summary_rows.append(
            {
                "suite": suite,
                "solver": solver,
                "cases": len(group),
                "solved": len(solved),
                "solve_rate": len(solved) / len(group) if group else math.nan,
                "mean_soc_solved": mean_finite(soc_values),
                "mean_runtime_ms_solved": mean_finite(runtime_values),
                "mean_wall_time_s": mean_finite(wall_values),
                "external_timeouts": sum(int(r.get("external_timed_out") or 0) for r in group),
            }
        )
    write_csv(path, summary_rows)


def write_paired_comparison(out_dir: Path, rows: list[dict[str, Any]], ir_solver: str) -> None:
    ir_name = f"ir_tapf:{ir_solver}"
    by_case_solver: dict[tuple[str, str, str], dict[str, Any]] = {}
    for row in rows:
        by_case_solver[(str(row["suite"]), str(row["case_id"]), str(row["solver"]))] = row

    paired_rows = []
    summary: dict[str, dict[str, Any]] = {}
    case_keys = sorted({(str(r["suite"]), str(r["case_id"])) for r in rows})
    for suite, case_id in case_keys:
        lacam = by_case_solver.get((suite, case_id, "lacam_tapf"))
        ir = by_case_solver.get((suite, case_id, ir_name))
        if lacam is None or ir is None:
            continue
        lacam_solved = int(lacam.get("solved") or 0)
        ir_solved = int(ir.get("solved") or 0)
        lacam_soc = safe_float(lacam.get("soc"))
        ir_soc = safe_float(ir.get("soc"))
        both_solved = lacam_solved and ir_solved and math.isfinite(lacam_soc) and math.isfinite(ir_soc)
        diff = lacam_soc - ir_soc if both_solved else math.nan
        paired_rows.append(
            {
                "suite": suite,
                "case_id": case_id,
                "num_agents": lacam.get("num_agents", ir.get("num_agents")),
                "lacam_solved": lacam_solved,
                "ir_solved": ir_solved,
                "both_solved": int(bool(both_solved)),
                "lacam_soc": lacam_soc if math.isfinite(lacam_soc) else "",
                "ir_soc": ir_soc if math.isfinite(ir_soc) else "",
                "soc_lacam_minus_ir": diff if math.isfinite(diff) else "",
                "winner": (
                    "ir"
                    if both_solved and diff > 0
                    else "lacam"
                    if both_solved and diff < 0
                    else "tie"
                    if both_solved
                    else ""
                ),
                "lacam_wall_time_s": lacam.get("wall_time_s", ""),
                "ir_wall_time_s": ir.get("wall_time_s", ""),
            }
        )

        s = summary.setdefault(
            suite,
            {
                "suite": suite,
                "paired_cases": 0,
                "both_solved": 0,
                "lacam_better": 0,
                "ir_better": 0,
                "ties": 0,
                "diffs": [],
                "lacam_soc": [],
                "ir_soc": [],
            },
        )
        s["paired_cases"] += 1
        if both_solved:
            s["both_solved"] += 1
            s["diffs"].append(diff)
            s["lacam_soc"].append(lacam_soc)
            s["ir_soc"].append(ir_soc)
            if diff > 0:
                s["ir_better"] += 1
            elif diff < 0:
                s["lacam_better"] += 1
            else:
                s["ties"] += 1

    summary_rows = []
    for suite, s in sorted(summary.items()):
        summary_rows.append(
            {
                "suite": suite,
                "paired_cases": s["paired_cases"],
                "both_solved": s["both_solved"],
                "lacam_better": s["lacam_better"],
                "ir_better": s["ir_better"],
                "ties": s["ties"],
                "mean_lacam_soc": mean_finite(s["lacam_soc"]),
                "mean_ir_soc": mean_finite(s["ir_soc"]),
                "mean_soc_lacam_minus_ir": mean_finite(s["diffs"]),
            }
        )

    write_csv(out_dir / "paired_cases.csv", paired_rows)
    write_csv(out_dir / "paired_summary.csv", summary_rows)


def mean_finite(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return sum(finite) / len(finite) if finite else math.nan


def prepare_lacam_cases(args: argparse.Namespace) -> list[dict[str, Any]]:
    fixtures = discover_yaml_fixtures(args.lacam_fixture_dir, args.lacam_cases)
    cases = []
    matrix_dir = args.out_dir / "converted" / "lacam_yaml_to_ir_matrix"
    for i, fixture in enumerate(fixtures):
        matrix = matrix_dir / f"{fixture.parent.name}_{fixture.stem}.matrix"
        meta = yaml_to_matrix(fixture, matrix, args.lacam_map_dir, seed=i + 1)
        cases.append(
            {
                "suite": "lacam_fixtures",
                "case_id": f"lacam_{i:04d}",
                "yaml": fixture,
                "matrix": matrix,
                **meta,
            }
        )
    return cases


def ir_matrix_path(
    ir_repo: Path,
    map_file: Path,
    agents: int,
    seed: int,
    target_probability: float,
    avoid_distance: int,
    max_targets_per_agent: int,
) -> Path:
    map_name = map_file.stem
    if max_targets_per_agent > 0:
        subdir = f"prob{target_probability}_dist{avoid_distance}_max{max_targets_per_agent}"
    else:
        subdir = f"prob{target_probability}_dist{avoid_distance}"
    return ir_repo / "matrix" / map_name / f"agents{agents}" / subdir / f"seed{seed}.matrix"


def ensure_ir_matrices(args: argparse.Namespace) -> list[Path]:
    matrix_files: list[Path] = []
    pairs: list[tuple[int, int]] = []
    for agents in args.ir_agent_counts:
        for seed in args.ir_seeds:
            pairs.append((agents, seed))
            if len(pairs) >= args.ir_cases:
                break
        if len(pairs) >= args.ir_cases:
            break

    for agents, seed in pairs:
        matrix = ir_matrix_path(
            args.ir_repo,
            args.ir_map.resolve(),
            agents,
            seed,
            args.ir_target_probability,
            args.ir_avoid_distance,
            args.ir_max_targets_per_agent,
        )
        if not matrix.exists():
            cmd = [
                str(args.ir_bin.resolve()),
                "setup",
                "--map",
                str(args.ir_map.resolve()),
                "--num-agents",
                str(agents),
                "--seeds",
                str(seed),
                "random",
                "--target-probability",
                str(args.ir_target_probability),
                "--avoid-distance",
                str(args.ir_avoid_distance),
                "--max-targets-per-agent",
                str(args.ir_max_targets_per_agent),
            ]
            subprocess.run(cmd, cwd=args.ir_repo, check=True, text=True, capture_output=True)
        matrix_files.append(matrix)
    return matrix_files


def prepare_ir_cases(args: argparse.Namespace) -> list[dict[str, Any]]:
    matrices = ensure_ir_matrices(args)
    cases = []
    yaml_dir = args.out_dir / "converted" / "ir_matrix_to_lacam_yaml"
    for i, matrix in enumerate(matrices):
        yaml_path = yaml_dir / f"{matrix.parent.parent.name}_{matrix.stem}.yaml"
        meta = matrix_to_yaml(matrix, yaml_path)
        cases.append(
            {
                "suite": "ir_fixtures",
                "case_id": f"ir_{i:04d}",
                "yaml": yaml_path,
                "matrix": matrix,
                **meta,
            }
        )
    return cases


def parse_int_list(value: str) -> list[int]:
    out = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = [int(v) for v in part.split("-", 1)]
            out.extend(range(lo, hi + 1))
        else:
            out.append(int(part))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ir-repo", type=Path, default=Path("/home/yimin/research/ir-tapf"))
    parser.add_argument(
        "--ir-bin",
        type=Path,
        default=Path("/home/yimin/research/ir-tapf/target/release/ir_tapf"),
    )
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--lacam-fixture-dir", type=Path, action="append", required=True)
    parser.add_argument("--lacam-map-dir", type=Path, default=None)
    parser.add_argument("--lacam-cases", type=int, default=100)
    parser.add_argument("--ir-cases", type=int, default=100)
    parser.add_argument("--ir-map", type=Path, default=Path("/home/yimin/research/ir-tapf/map/random-32-32-10.map"))
    parser.add_argument("--ir-agent-counts", type=parse_int_list, default=parse_int_list("10,20,30,40,50"))
    parser.add_argument("--ir-seeds", type=parse_int_list, default=parse_int_list("1-20"))
    parser.add_argument("--ir-target-probability", type=float, default=0.4)
    parser.add_argument("--ir-avoid-distance", type=int, default=2)
    parser.add_argument("--ir-max-targets-per-agent", type=int, default=0)
    parser.add_argument("--ir-solver", default="dbs_hungarian")
    parser.add_argument("--ir-max-iterations", type=int, default=100000)
    parser.add_argument("--time-limit", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.set_defaults(lacam_anytime=True)
    parser.add_argument("--lacam-anytime", dest="lacam_anytime", action="store_true")
    parser.add_argument("--no-lacam-anytime", dest="lacam_anytime", action="store_false")
    parser.add_argument("--lacam-full-ta", action="store_true")
    parser.add_argument("--lacam-search-mode", choices=["dfs", "focal"], default="dfs")
    parser.add_argument("--lacam-focal-weight", type=float, default=1.5)
    parser.add_argument(
        "--lacam-focal-tie-break",
        choices=["h", "anti_wait", "anti_zigzag", "anti_push", "anti_all"],
        default="h",
    )
    parser.add_argument("--suite", choices=["both", "lacam", "ir"], default="both")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/ir_lacam_cross_100"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_path = args.out_dir / "rows.jsonl"
    csv_path = args.out_dir / "rows.csv"
    summary_path = args.out_dir / "summary.csv"

    cases: list[dict[str, Any]] = []
    if args.suite in {"both", "lacam"}:
        cases.extend(prepare_lacam_cases(args))
    if args.suite in {"both", "ir"}:
        cases.extend(prepare_ir_cases(args))

    rows: list[dict[str, Any]] = []
    completed: set[tuple[str, str]] = set()
    if args.resume and rows_path.exists():
        for line in rows_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            rows.append(row)
            completed.add((row["case_id"], row["solver"]))
    elif rows_path.exists():
        rows_path.unlink()

    tasks = []
    for case in cases:
        for solver_kind, solver_name in (
            ("lacam", "lacam_tapf"),
            ("ir", f"ir_tapf:{args.ir_solver}"),
        ):
            if (case["case_id"], solver_name) not in completed:
                tasks.append((case, solver_kind))

    print(
        f"Prepared {len(cases)} cases, {len(tasks)} solver tasks, "
        f"jobs={args.jobs}, time_limit={args.time_limit}s, timeout={args.timeout}s"
    )
    if rows:
        write_csv(csv_path, rows)
        write_summary(summary_path, rows)
        write_paired_comparison(args.out_dir, rows, args.ir_solver)

    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as executor:
        future_to_task = {
            executor.submit(run_task, task, args): task
            for task in tasks
        }
        for done, future in enumerate(concurrent.futures.as_completed(future_to_task), start=1):
            case, solver_kind = future_to_task[future]
            try:
                row = future.result()
            except Exception:
                print(f"ERROR {case['case_id']} {solver_kind}", file=sys.stderr)
                write_csv(csv_path, rows)
                write_summary(summary_path, rows)
                raise
            rows.append(row)
            append_jsonl(rows_path, row)
            print(
                f"[{done}/{len(tasks)}] {row['suite']} {row['case_id']} "
                f"{row['solver']} solved={row.get('solved', 0)} "
                f"soc={row.get('soc', '')} wall={safe_float(row.get('wall_time_s')):.2f}s"
            )

    write_csv(csv_path, rows)
    write_summary(summary_path, rows)
    write_paired_comparison(args.out_dir, rows, args.ir_solver)
    print(f"wrote {csv_path}")
    print(f"wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
