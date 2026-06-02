#!/usr/bin/env python3
"""Run arXiv 2605.07744 TAPF experiments with LaCAM-TAPF baselines.

The runner generates ir-tapf matrix instances using the ir-tapf setup command,
runs the paper's ir-tapf solvers, converts the same matrices to LaCAM-TAPF
YAML, and records comparable rows for plotting.
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
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from run_ir_lacam_cross_experiment import load_map, matrix_to_yaml, parse_ir_stdout, parse_matrix


MAP_ROOT = Path("scripts/map")


@dataclass(frozen=True)
class Scenario:
    name: str
    map_name: str
    target_mode: str
    agents: tuple[int, ...]
    seeds: tuple[int, ...]
    time_limits: tuple[float, ...]
    solvers: tuple[str, ...]
    max_iterations: int
    target_probability: float = 0.4
    avoid_distance: int = 2
    max_targets_per_agent: int = 10
    num_hotspots: int = 1
    hotspot_radius: int | None = None
    targets_per_hotspot: int = 10
    hotspot_placement: str = "random"
    lacam_methods: tuple[str, ...] = ("lacam_dfs", "lacam_focal_h")


COMPONENT_SOLVERS = (
    "dbs_hungarian",
    "sbs_hungarian",
    "random_hungarian",
    "dbs_pibt",
    "sbs_pibt",
    "random_pibt",
)

RANDOM_INITIAL_SOLVERS = (
    "dbs_hungarian_random_initial",
    "sbs_hungarian_random_initial",
    "random_hungarian_random_initial",
    "dbs_pibt_random_initial",
    "sbs_pibt_random_initial",
    "random_pibt_random_initial",
)


def one_to(n: int) -> tuple[int, ...]:
    return tuple(range(1, n + 1))


def paper_scenarios(kind: str, smoke: bool) -> list[Scenario]:
    seeds = one_to(2 if smoke else 30)
    if kind == "smoke":
        seeds = one_to(1)

    maps = (
        "random-64-64-20",
        "warehouse-10-20-10-2-2",
        "Boston_0_256",
        "den520d",
        "lak303d",
        "ost003d",
    )
    agents_fig3 = (200,) if smoke else (200, 400, 600, 800)
    solvers_component = ("dbs_hungarian", "dbs_pibt") if smoke else COMPONENT_SOLVERS

    scenarios: list[Scenario] = []
    if kind in {"all", "fig3", "smoke"}:
        for map_name in maps if not smoke else ("random-64-64-20",):
            for mode in ("random", "hotspot"):
                scenarios.append(
                    Scenario(
                        name=f"fig3_{map_name}_{mode}",
                        map_name=map_name,
                        target_mode=mode,
                        agents=agents_fig3,
                        seeds=seeds,
                        time_limits=(10.0,),
                        solvers=solvers_component,
                        max_iterations=100000,
                        target_probability=0.4,
                        avoid_distance=2 if mode == "random" else 0,
                        max_targets_per_agent=10,
                        num_hotspots=1,
                        hotspot_radius=25,
                        targets_per_hotspot=10,
                        hotspot_placement="random",
                    )
                )

    if kind in {"all", "table1", "smoke"}:
        table_solvers = solvers_component if smoke else COMPONENT_SOLVERS
        scenarios.append(
            Scenario(
                name="table1_random64_hotspot_200_time",
                map_name="random-64-64-20",
                target_mode="hotspot",
                agents=(200,),
                seeds=seeds,
                time_limits=(10.0,),
                solvers=table_solvers,
                max_iterations=100000,
                avoid_distance=0,
                hotspot_radius=25,
            )
        )
        scenarios.append(
            Scenario(
                name="table2_random64_hotspot_200_iter100",
                map_name="random-64-64-20",
                target_mode="hotspot",
                agents=(200,),
                seeds=seeds,
                time_limits=(1000.0,),
                solvers=tuple(f"iter_{s}" for s in table_solvers),
                max_iterations=100,
                avoid_distance=0,
                hotspot_radius=25,
                lacam_methods=(),
            )
        )

    if kind in {"all", "table3"} and not smoke:
        scenarios.append(
            Scenario(
                name="table3_random_initial_random64_hotspot_200",
                map_name="random-64-64-20",
                target_mode="hotspot",
                agents=(200,),
                seeds=seeds,
                time_limits=(10.0,),
                solvers=RANDOM_INITIAL_SOLVERS,
                max_iterations=100000,
                avoid_distance=0,
                hotspot_radius=25,
            )
        )

    if kind in {"all", "fig4"} and not smoke:
        scenarios.append(
            Scenario(
                name="fig4_lak303d_hotspot_final_opt",
                map_name="lak303d",
                target_mode="hotspot",
                agents=(200,),
                seeds=seeds,
                time_limits=(30.0,),
                solvers=(
                    "opt_dbs_hungarian",
                    "opt_sbs_hungarian",
                    "opt_random_hungarian",
                    "opt_dbs_pibt",
                    "opt_sbs_pibt",
                    "opt_random_pibt",
                    "opt_initial",
                ),
                max_iterations=100000,
                avoid_distance=0,
                hotspot_radius=25,
            )
        )

    if kind in {"all", "fig5"} and not smoke:
        scenarios.append(
            Scenario(
                name="fig5_scalability_warehouse20",
                map_name="warehouse-20-40-10-2-2",
                target_mode="hotspot",
                agents=(1000, 2000, 5000, 10000),
                seeds=one_to(10),
                time_limits=(600.0,),
                solvers=("dbs_hungarian",),
                max_iterations=100000,
                avoid_distance=0,
                hotspot_radius=80,
                lacam_methods=("lacam_dfs",),
            )
        )

    if kind in {"all", "fig6"} and not smoke:
        # ir-tapf currently exposes k internally, so this records the default
        # k=3 setting. The plotter leaves room for adding patched k variants.
        scenarios.append(
            Scenario(
                name="fig6_ost003d_multibottleneck_default",
                map_name="ost003d",
                target_mode="hotspot",
                agents=(100, 200, 400, 600, 800),
                seeds=seeds,
                time_limits=(10.0,),
                solvers=("dbs_hungarian", "dbs_pibt"),
                max_iterations=100000,
                avoid_distance=0,
                hotspot_radius=25,
            )
        )

    return scenarios


def parse_kv(text: str) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
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


def bfs_distances(info: Any, start: int) -> dict[int, int]:
    start_coord = info.id_to_coord[start]
    q = [start_coord]
    dist = {start_coord: 0}
    head = 0
    while head < len(q):
        r, c = q[head]
        head += 1
        for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if (nr, nc) in dist:
                continue
            if (nr, nc) not in info.coord_to_id:
                continue
            dist[(nr, nc)] = dist[(r, c)] + 1
            q.append((nr, nc))
    return {info.coord_to_id[coord]: value for coord, value in dist.items()}


_MATRIX_LB_CACHE: dict[Path, int] = {}


def matrix_sum_shortest_distances(matrix: Path) -> int:
    matrix = matrix.resolve()
    cached = _MATRIX_LB_CACHE.get(matrix)
    if cached is not None:
        return cached
    parsed = parse_matrix(matrix)
    info = load_map(Path(parsed["metadata"]["map"]))
    total = 0
    for start, targets in zip(parsed["starts"], parsed["targets"]):
        dist = bfs_distances(info, start)
        reachable = [dist[t] for t in targets if t in dist]
        if not reachable:
            continue
        total += min(reachable)
    _MATRIX_LB_CACHE[matrix] = total
    return total


def matrix_path(ir_repo: Path, scenario: Scenario, agents: int, seed: int) -> Path:
    if scenario.target_mode == "random":
        suffix = f"prob{scenario.target_probability}_dist{scenario.avoid_distance}"
        if scenario.max_targets_per_agent > 0:
            suffix += f"_max{scenario.max_targets_per_agent}"
    else:
        radius = scenario.hotspot_radius if scenario.hotspot_radius is not None else 25
        suffix = (
            f"hotspot_{scenario.hotspot_placement}_c{scenario.num_hotspots}"
            f"_r{radius}_tpa{scenario.targets_per_hotspot}_dist{scenario.avoid_distance}"
        )
    return (
        ir_repo
        / "matrix"
        / scenario.map_name
        / f"agents{agents}"
        / suffix
        / f"seed{seed}.matrix"
    )


def is_valid_matrix(matrix: Path) -> bool:
    try:
        parsed = parse_matrix(matrix)
        return bool(parsed["starts"]) and len(parsed["targets"]) == len(parsed["starts"])
    except Exception:
        return False


def ensure_matrix(
    ir_bin: Path,
    ir_repo: Path,
    map_root: Path,
    scenario: Scenario,
    agents: int,
    seed: int,
) -> Path:
    matrix = matrix_path(ir_repo, scenario, agents, seed)
    if matrix.exists() and is_valid_matrix(matrix):
        return matrix
    lock = matrix.with_suffix(matrix.suffix + ".lock")
    lock.parent.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.close(fd)
            break
        except FileExistsError:
            time.sleep(0.1)
            if matrix.exists() and is_valid_matrix(matrix):
                return matrix
    map_file = (map_root / f"{scenario.map_name}.map").resolve()
    try:
        if matrix.exists() and not is_valid_matrix(matrix):
            matrix.unlink()
        cmd = [
            str(ir_bin),
            "setup",
            "--map",
            str(map_file),
            "--num-agents",
            str(agents),
            "--seeds",
            str(seed),
        ]
        if scenario.target_mode == "random":
            cmd += [
                "random",
                "--target-probability",
                str(scenario.target_probability),
                "--avoid-distance",
                str(scenario.avoid_distance),
                "--max-targets-per-agent",
                str(scenario.max_targets_per_agent),
            ]
        else:
            radius = scenario.hotspot_radius if scenario.hotspot_radius is not None else 25
            cmd += [
                "hotspot",
                "--num-hotspots",
                str(scenario.num_hotspots),
                "--hotspot-radius",
                str(radius),
                "--targets-per-hotspot",
                str(scenario.targets_per_hotspot),
                "--avoid-distance",
                str(scenario.avoid_distance),
                "--hotspot-placement",
                scenario.hotspot_placement,
            ]
        subprocess.run(cmd, cwd=ir_repo, check=True, text=True, capture_output=True)
    finally:
        lock.unlink(missing_ok=True)
    if not is_valid_matrix(matrix):
        raise ValueError(f"invalid generated matrix: {matrix}")
    return matrix


def converted_yaml_path(out_dir: Path, scenario_name: str, matrix: Path) -> Path:
    return out_dir / "converted_yaml" / scenario_name / matrix.parent.name / f"{matrix.stem}.yaml"


def run_ir(
    task: dict[str, Any],
    args: argparse.Namespace,
    scenario: Scenario,
) -> dict[str, Any]:
    matrix = ensure_matrix(
        args.ir_bin, args.ir_repo, args.map_root, scenario, task["agents"], task["seed"]
    )
    cmd = [
        str(args.ir_bin),
        "solve",
        "--matrix",
        str(matrix.resolve()),
        "--solver",
        task["method"],
        "--max-iterations",
        str(scenario.max_iterations),
        "--time-limit-sec",
        str(task["time_limit"]),
    ]
    start = time.time()
    sum_shortest_distances = matrix_sum_shortest_distances(matrix)
    try:
        cp = subprocess.run(
            cmd, cwd=args.ir_repo, text=True, capture_output=True, timeout=args.timeout
        )
    except subprocess.TimeoutExpired as exc:
        return {**task, "matrix_file": str(matrix), "solver": f"ir_tapf:{task['method']}",
                "solved": 0, "valid_solution": 0, "timed_out": 1,
                "external_timed_out": 1, "exit_code": 124,
                "wall_time_s": time.time() - start, "stderr": str(exc)}
    row = parse_ir_stdout(cp.stdout)
    row["sum_shortest_distances"] = sum_shortest_distances
    row.update(
        {
            **task,
            "matrix_file": str(matrix),
            "solver": f"ir_tapf:{task['method']}",
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


def run_lacam(
    task: dict[str, Any],
    args: argparse.Namespace,
    scenario: Scenario,
) -> dict[str, Any]:
    matrix = ensure_matrix(
        args.ir_bin, args.ir_repo, args.map_root, scenario, task["agents"], task["seed"]
    )
    yaml_path = converted_yaml_path(args.out_dir, scenario.name, matrix)
    if not yaml_path.exists():
        matrix_to_yaml(matrix, yaml_path)
    sum_shortest_distances = matrix_sum_shortest_distances(matrix)
    mode = "focal" if task["method"] == "lacam_focal_h" else "dfs"
    cmd = [
        str(args.lacam_bin),
        str(yaml_path),
        "",
        str(task["time_limit"]),
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
        return {**task, "matrix_file": str(matrix), "fixture_file": str(yaml_path),
                "solver": "lacam_tapf", "search_mode_arg": mode,
                "solved": 0, "valid_solution": 0, "timed_out": 1,
                "external_timed_out": 1, "exit_code": 124,
                "sum_shortest_distances": sum_shortest_distances,
                "wall_time_s": time.time() - start, "stderr": str(exc)}
    row = parse_kv(cp.stdout)
    row["sum_shortest_distances"] = sum_shortest_distances
    row.update(
        {
            **task,
            "matrix_file": str(matrix),
            "fixture_file": str(yaml_path),
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


def run_task(task: dict[str, Any], scenarios: dict[str, Scenario], args: argparse.Namespace) -> dict[str, Any]:
    scenario = scenarios[task["scenario"]]
    if task["method"].startswith("lacam_"):
        return run_lacam(task, args, scenario)
    return run_ir(task, args, scenario)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = (row["scenario"], row["map_name"], row["target_mode"], row["method"], row["agents"], row["time_limit"])
        groups.setdefault(key, []).append(row)
    out = []
    for key, group in sorted(groups.items()):
        solved = [r for r in group if int(r.get("solved") or 0)]
        out.append(
            {
                "scenario": key[0],
                "map_name": key[1],
                "target_mode": key[2],
                "method": key[3],
                "agents": key[4],
                "time_limit": key[5],
                "cases": len(group),
                "solved": len(solved),
                "solve_rate": len(solved) / len(group) if group else math.nan,
                "mean_soc": mean([safe_float(r.get("soc")) for r in solved]),
                "mean_initial_solution_cost": mean(
                    [safe_float(r.get("initial_solution_cost")) for r in solved]
                ),
                "mean_sum_shortest_distances": mean(
                    [safe_float(r.get("sum_shortest_distances")) for r in solved]
                ),
                "mean_normalized_cost": mean(
                    [
                        safe_float(r.get("soc")) / safe_float(r.get("sum_shortest_distances"))
                        for r in solved
                        if safe_float(r.get("sum_shortest_distances")) > 0
                    ]
                ),
                "mean_improvement_pct": mean(
                    [
                        100.0
                        * (safe_float(r.get("initial_solution_cost")) - safe_float(r.get("soc")))
                        / safe_float(r.get("initial_solution_cost"))
                        for r in solved
                        if safe_float(r.get("initial_solution_cost")) > 0
                    ]
                ),
                "mean_iterations_used": mean(
                    [safe_float(r.get("iterations_used")) for r in solved]
                ),
                "mean_wall_time_s": mean([safe_float(r.get("wall_time_s")) for r in group]),
                "external_timeouts": sum(int(r.get("external_timed_out") or 0) for r in group),
            }
        )
    write_csv(path, out)


def build_tasks(scenarios: list[Scenario]) -> list[dict[str, Any]]:
    tasks = []
    for scenario in scenarios:
        for agents in scenario.agents:
            for seed in scenario.seeds:
                for time_limit in scenario.time_limits:
                    methods = list(scenario.solvers) + list(scenario.lacam_methods)
                    for method in methods:
                        tasks.append(
                            {
                                "scenario": scenario.name,
                                "map_name": scenario.map_name,
                                "target_mode": scenario.target_mode,
                                "agents": agents,
                                "seed": seed,
                                "time_limit": time_limit,
                                "method": method,
                            }
                        )
    return tasks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paper-suite", choices=["all", "fig3", "fig4", "fig5", "fig6", "table1", "table3", "smoke"], default="smoke")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/paper_2605_07744"))
    parser.add_argument("--ir-repo", type=Path, default=Path("/home/yimin/research/ir-tapf"))
    parser.add_argument("--ir-bin", type=Path, default=Path("/home/yimin/research/ir-tapf/target/release/ir_tapf"))
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--map-root", type=Path, default=MAP_ROOT)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-tasks", type=int, default=0)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    smoke = args.paper_suite == "smoke"
    scenarios = paper_scenarios(args.paper_suite, smoke)
    scenario_by_name = {s.name: s for s in scenarios}
    tasks = build_tasks(scenarios)
    if args.max_tasks > 0:
        tasks = tasks[: args.max_tasks]

    rows: list[dict[str, Any]] = []
    completed: set[tuple[Any, ...]] = set()
    if args.resume and rows_jsonl.exists():
        with rows_jsonl.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                row = json.loads(line)
                rows.append(row)
                completed.add((row["scenario"], row["agents"], row["seed"], row["time_limit"], row["method"]))
    elif rows_jsonl.exists():
        rows_jsonl.unlink()

    tasks = [
        t
        for t in tasks
        if (t["scenario"], t["agents"], t["seed"], t["time_limit"], t["method"])
        not in completed
    ]
    print(f"scenarios={len(scenarios)} tasks={len(tasks)} completed={len(rows)} jobs={args.jobs}", flush=True)

    if rows:
        write_csv(rows_csv, rows)
        write_summary(summary_csv, rows)

    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(run_task, task, scenario_by_name, args): task for task in tasks
        }
        for done, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            task = futures[future]
            try:
                row = future.result()
            except Exception as exc:
                row = {**task, "solver": task["method"], "solved": 0, "valid_solution": 0,
                       "external_timed_out": 0, "exit_code": -1, "wall_time_s": 0,
                       "stderr": repr(exc), "first_error": "runner exception"}
            rows.append(row)
            with rows_jsonl.open("a", encoding="utf-8") as f:
                f.write(json.dumps(row, sort_keys=True) + "\n")
            print(
                f"[{done}/{len(tasks)}] {row['scenario']} n={row['agents']} "
                f"seed={row['seed']} {row['method']} solved={row.get('solved', 0)} "
                f"soc={row.get('soc', '')} wall={safe_float(row.get('wall_time_s')):.2f}s",
                flush=True,
            )

    write_csv(rows_csv, rows)
    write_summary(summary_csv, rows)
    print(f"wrote {rows_csv}", flush=True)
    print(f"wrote {summary_csv}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
