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
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

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

FIG4_BASE_SOLVERS = {
    "opt_dbs_hungarian": "dbs_hungarian",
    "opt_sbs_hungarian": "sbs_hungarian",
    "opt_random_hungarian": "random_hungarian",
    "opt_dbs_pibt": "dbs_pibt",
    "opt_sbs_pibt": "sbs_pibt",
    "opt_random_pibt": "random_pibt",
    "opt_initial": "lazy_greedy_with_refinement",
}


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

    if kind in {"all", "table1", "table2", "smoke"}:
        table_solvers = solvers_component if smoke else COMPONENT_SOLVERS
        if kind in {"all", "table1", "smoke"}:
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
        if kind in {"all", "table2", "smoke"}:
            scenarios.append(
                Scenario(
                    name="table2_random64_hotspot_200_iter100",
                    map_name="random-64-64-20",
                    target_mode="hotspot",
                    agents=(200,),
                    seeds=seeds,
                    time_limits=(10.0,),
                    solvers=tuple(f"iter_{s}" for s in table_solvers),
                    max_iterations=100,
                    avoid_distance=0,
                    hotspot_radius=25,
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
            )
        )

    if kind in {"all", "fig6"} and not smoke:
        scenarios.append(
            Scenario(
                name="fig6_ost003d_multibottleneck_default",
                map_name="ost003d",
                target_mode="hotspot",
                agents=(100, 200, 400, 600, 800),
                seeds=seeds,
                time_limits=(10.0,),
                solvers=(
                    "dbs_hungarian_k1",
                    "dbs_hungarian_k3",
                    "dbs_hungarian_k10",
                    "dbs_pibt_k1",
                    "dbs_pibt_k3",
                    "dbs_pibt_k10",
                ),
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
_MAP_COMPONENT_CACHE: dict[Path, dict[int, int]] = {}


def map_components(info: Any) -> dict[int, int]:
    path = Path(info.path).resolve()
    cached = _MAP_COMPONENT_CACHE.get(path)
    if cached is not None:
        return cached
    components: dict[int, int] = {}
    component_id = 0
    for coord, vertex in info.coord_to_id.items():
        if vertex in components:
            continue
        q = [coord]
        components[vertex] = component_id
        head = 0
        while head < len(q):
            r, c = q[head]
            head += 1
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                neighbor = info.coord_to_id.get((nr, nc))
                if neighbor is None or neighbor in components:
                    continue
                components[neighbor] = component_id
                q.append((nr, nc))
        component_id += 1
    _MAP_COMPONENT_CACHE[path] = components
    return components


def matrix_lb_cache_path(matrix: Path) -> Path:
    return matrix.with_suffix(matrix.suffix + ".sum_shortest.json")


def read_matrix_lb_cache(matrix: Path) -> int | None:
    cache = matrix_lb_cache_path(matrix)
    try:
        stat = matrix.stat()
        data = json.loads(cache.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    if (
        data.get("matrix_size") == stat.st_size
        and data.get("matrix_mtime_ns") == stat.st_mtime_ns
        and isinstance(data.get("sum_shortest_distances"), int)
    ):
        return int(data["sum_shortest_distances"])
    return None


def write_matrix_lb_cache(matrix: Path, total: int) -> None:
    cache = matrix_lb_cache_path(matrix)
    stat = matrix.stat()
    payload = {
        "matrix_size": stat.st_size,
        "matrix_mtime_ns": stat.st_mtime_ns,
        "sum_shortest_distances": total,
    }
    tmp = cache.with_suffix(cache.suffix + f".{os.getpid()}.tmp")
    tmp.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    os.replace(tmp, cache)


def compute_matrix_sum_shortest_distances(matrix: Path) -> int:
    parsed = parse_matrix(matrix)
    info = load_map(Path(parsed["metadata"]["map"]))
    starts = parsed["starts"]
    targets = parsed["targets"]
    unique_starts = set(starts)
    unique_targets = {target for row in targets for target in row}

    total = 0
    if len(unique_targets) <= len(unique_starts):
        distances_by_target = {target: bfs_distances(info, target) for target in unique_targets}
        for start, row in zip(starts, targets):
            reachable = [distances_by_target[target][start] for target in row if start in distances_by_target[target]]
            if reachable:
                total += min(reachable)
        return total

    distances_by_start: dict[int, dict[int, int]] = {}
    for start, row in zip(starts, targets):
        dist = distances_by_start.get(start)
        if dist is None:
            dist = bfs_distances(info, start)
            distances_by_start[start] = dist
        reachable = [dist[target] for target in row if target in dist]
        if reachable:
            total += min(reachable)
    return total


def matrix_sum_shortest_distances(matrix: Path) -> int:
    matrix = matrix.resolve()
    cached = _MATRIX_LB_CACHE.get(matrix)
    if cached is not None:
        return cached
    cached = read_matrix_lb_cache(matrix)
    if cached is not None:
        _MATRIX_LB_CACHE[matrix] = cached
        return cached

    lock = matrix_lb_cache_path(matrix).with_suffix(".lock")
    while True:
        try:
            fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.close(fd)
            break
        except FileExistsError:
            time.sleep(0.2)
            cached = read_matrix_lb_cache(matrix)
            if cached is not None:
                _MATRIX_LB_CACHE[matrix] = cached
                return cached
            try:
                if time.time() - lock.stat().st_mtime > 3600:
                    lock.unlink()
            except FileNotFoundError:
                pass

    try:
        cached = read_matrix_lb_cache(matrix)
        if cached is not None:
            total = cached
        else:
            total = compute_matrix_sum_shortest_distances(matrix)
            write_matrix_lb_cache(matrix, total)
    finally:
        lock.unlink(missing_ok=True)
    _MATRIX_LB_CACHE[matrix] = total
    return total


def task_sum_shortest_distances(matrix: Path, row: dict[str, Any], args: argparse.Namespace) -> int | str:
    if args.skip_sum_shortest or not int(row.get("solved") or 0):
        return ""
    return matrix_sum_shortest_distances(matrix)


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
        starts = parsed["starts"]
        targets = parsed["targets"]
        if not starts or len(targets) != len(starts):
            return False
        if any(not row for row in targets):
            return False
        info = load_map(Path(parsed["metadata"]["map"]))
        components = map_components(info)
        reachable_targets = []
        for start, row in zip(starts, targets):
            start_component = components.get(start)
            reachable_row = [
                target
                for target in row
                if start_component is not None and components.get(target) == start_component
            ]
            if not reachable_row:
                return False
            reachable_targets.append(reachable_row)
        return matrix_has_perfect_matching(reachable_targets, len(starts))
    except Exception:
        return False


def matrix_has_perfect_matching(targets: list[list[int]], num_agents: int) -> bool:
    sys.setrecursionlimit(max(sys.getrecursionlimit(), num_agents * 2 + 100))
    match_to_agent: dict[int, int] = {}

    def augment(agent: int, seen: set[int]) -> bool:
        for target in targets[agent]:
            if target in seen:
                continue
            seen.add(target)
            assigned = match_to_agent.get(target)
            if assigned is None or augment(assigned, seen):
                match_to_agent[target] = agent
                return True
        return False

    for agent in range(num_agents):
        if not augment(agent, set()):
            return False
    return True


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
    agents_dir = matrix.parent.parent.name
    return out_dir / "converted_yaml" / scenario_name / agents_dir / matrix.parent.name / f"{matrix.stem}.yaml"


def fixed_goal_yaml_path(out_dir: Path, scenario_name: str, method: str, matrix: Path) -> Path:
    agents_dir = matrix.parent.parent.name
    return out_dir / "fixed_goal_yaml" / scenario_name / method / agents_dir / matrix.parent.name / f"{matrix.stem}.yaml"


def output_needs_refresh(output: Path, source: Path) -> bool:
    return not output.exists() or output.stat().st_mtime < source.stat().st_mtime


def ensure_converted_yaml(matrix: Path, yaml_path: Path) -> None:
    if not output_needs_refresh(yaml_path, matrix):
        return
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    lock = yaml_path.with_suffix(yaml_path.suffix + ".lock")
    while True:
        try:
            fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.close(fd)
            break
        except FileExistsError:
            time.sleep(0.1)
            if not output_needs_refresh(yaml_path, matrix):
                return
    try:
        if output_needs_refresh(yaml_path, matrix):
            matrix_to_yaml(matrix, yaml_path)
    finally:
        lock.unlink(missing_ok=True)


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


def run_ir(
    task: dict[str, Any],
    args: argparse.Namespace,
    scenario: Scenario,
) -> dict[str, Any]:
    matrix = ensure_matrix(
        args.ir_bin, args.ir_repo, args.map_root, scenario, task["agents"], task["seed"]
    )
    solver = task["method"]
    pickup_agents = task.get("num_pickup_agents")
    match = re.match(r"^(.*)_k(\d+)$", solver)
    if match:
        solver = match.group(1)
        pickup_agents = int(match.group(2))

    cmd = [
        str(args.ir_bin),
        "solve",
        "--matrix",
        str(matrix.resolve()),
        "--solver",
        solver,
        "--max-iterations",
        str(scenario.max_iterations),
        "--time-limit-sec",
        str(task["time_limit"]),
    ]
    if pickup_agents is not None:
        cmd += ["--num-pickup-agents", str(pickup_agents)]
    start = time.time()
    try:
        cp = subprocess.run(
            cmd, cwd=args.ir_repo, text=True, capture_output=True, timeout=args.timeout
        )
    except subprocess.TimeoutExpired as exc:
        return {**task, "matrix_file": str(matrix), "solver": f"ir_tapf:{solver}",
                "ir_solver_arg": solver,
                "num_pickup_agents": pickup_agents if pickup_agents is not None else "",
                "solved": 0, "valid_solution": 0, "timed_out": 1,
                "external_timed_out": 1, "exit_code": 124,
                "sum_shortest_distances": "",
                "wall_time_s": time.time() - start, "stderr": str(exc)}
    wall_time_s = time.time() - start
    row = parse_ir_stdout(cp.stdout)
    final_goals = parse_final_goals(cp.stdout)
    row.update(
        {
            **task,
            "matrix_file": str(matrix),
            "solver": f"ir_tapf:{solver}",
            "ir_solver_arg": solver,
            "num_pickup_agents": pickup_agents if pickup_agents is not None else "",
            "final_goals": " ".join(str(g) for g in final_goals),
            "exit_code": cp.returncode,
            "wall_time_s": wall_time_s,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    if cp.returncode != 0:
        row["solved"] = 0
        row["valid_solution"] = 0
    row["sum_shortest_distances"] = task_sum_shortest_distances(matrix, row, args)
    return row


def run_fig4_final_opt(
    task: dict[str, Any],
    args: argparse.Namespace,
    scenario: Scenario,
) -> dict[str, Any]:
    matrix = ensure_matrix(
        args.ir_bin, args.ir_repo, args.map_root, scenario, task["agents"], task["seed"]
    )
    base_solver = FIG4_BASE_SOLVERS[task["method"]]
    ir_task = {**task, "method": base_solver, "time_limit": 20.0}
    ir_row = run_ir(ir_task, args, scenario)
    if not int(ir_row.get("solved") or 0):
        return {
            **task,
            "matrix_file": str(matrix),
            "solver": f"fig4_two_stage:{base_solver}",
            "ir_solver_arg": base_solver,
            "solved": 0,
            "valid_solution": 0,
            "timed_out": ir_row.get("timed_out", 0),
            "external_timed_out": ir_row.get("external_timed_out", 0),
            "exit_code": ir_row.get("exit_code", -1),
            "wall_time_s": ir_row.get("wall_time_s", 0),
            "stderr": ir_row.get("stderr", ""),
            "first_error": "target refinement failed",
        }

    final_goals = [int(token) for token in str(ir_row.get("final_goals", "")).split() if token]
    if not final_goals:
        return {
            **task,
            "matrix_file": str(matrix),
            "solver": f"fig4_two_stage:{base_solver}",
            "ir_solver_arg": base_solver,
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 0,
            "external_timed_out": 0,
            "exit_code": -1,
            "wall_time_s": ir_row.get("wall_time_s", 0),
            "stderr": "",
            "first_error": "missing final_goals from ir-tapf",
        }

    yaml_path = fixed_goal_yaml_path(args.out_dir, scenario.name, task["method"], matrix)
    if output_needs_refresh(yaml_path, matrix):
        write_fixed_goal_yaml(matrix, final_goals, yaml_path)
    cmd = [
        str(args.lacam_bin),
        str(yaml_path),
        "",
        "10",
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
            **task,
            "matrix_file": str(matrix),
            "fixture_file": str(yaml_path),
            "solver": f"fig4_two_stage:{base_solver}",
            "ir_solver_arg": base_solver,
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "initial_solution_cost": ir_row.get("soc", ""),
            "initial_solution_time_ms": ir_row.get("wall_time_s", 0) * 1000.0,
            "sum_shortest_distances": ir_row.get("sum_shortest_distances", ""),
            "wall_time_s": safe_float(ir_row.get("wall_time_s")) + time.time() - start,
            "stderr": str(exc),
        }

    lacam_row = parse_kv(cp.stdout)
    lacam_row.update(
        {
            **task,
            "matrix_file": str(matrix),
            "fixture_file": str(yaml_path),
            "solver": f"fig4_two_stage:{base_solver}",
            "ir_solver_arg": base_solver,
            "search_mode_arg": "focal",
            "exit_code": cp.returncode,
            "wall_time_s": safe_float(ir_row.get("wall_time_s")) + time.time() - start,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
            "target_refinement_cost": ir_row.get("soc", ""),
            "target_refinement_wall_time_s": ir_row.get("wall_time_s", ""),
            "initial_solution_cost": ir_row.get("initial_solution_cost", ir_row.get("soc", "")),
            "initial_solution_time_ms": ir_row.get("initial_solution_time_ms", ""),
            "sum_shortest_distances": ir_row.get("sum_shortest_distances", ""),
            "final_goals": ir_row.get("final_goals", ""),
        }
    )
    lacam_row.setdefault("solved", 0)
    lacam_row.setdefault("valid_solution", 0)
    lacam_row.setdefault("timed_out", 0)
    return lacam_row


def run_lacam(
    task: dict[str, Any],
    args: argparse.Namespace,
    scenario: Scenario,
) -> dict[str, Any]:
    matrix = ensure_matrix(
        args.ir_bin, args.ir_repo, args.map_root, scenario, task["agents"], task["seed"]
    )
    yaml_path = converted_yaml_path(args.out_dir, scenario.name, matrix)
    ensure_converted_yaml(matrix, yaml_path)
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
                "sum_shortest_distances": "",
                "wall_time_s": time.time() - start, "stderr": str(exc)}
    wall_time_s = time.time() - start
    row = parse_kv(cp.stdout)
    row.update(
        {
            **task,
            "matrix_file": str(matrix),
            "fixture_file": str(yaml_path),
            "solver": "lacam_tapf",
            "search_mode_arg": mode,
            "exit_code": cp.returncode,
            "wall_time_s": wall_time_s,
            "external_timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    row.setdefault("timed_out", 0)
    row["sum_shortest_distances"] = task_sum_shortest_distances(matrix, row, args)
    return row


def run_task(task: dict[str, Any], scenarios: dict[str, Scenario], args: argparse.Namespace) -> dict[str, Any]:
    scenario = scenarios[task["scenario"]]
    if task["method"].startswith("lacam_"):
        return run_lacam(task, args, scenario)
    if task["method"] in FIG4_BASE_SOLVERS:
        return run_fig4_final_opt(task, args, scenario)
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


def is_completed_row(row: dict[str, Any]) -> bool:
    return (
        (
            row.get("exit_code") in (0, 124, None)
            or int(row.get("timed_out") or 0)
            or int(row.get("external_timed_out") or 0)
        )
        and not row.get("first_error")
    )


def build_tasks(scenarios: list[Scenario]) -> list[dict[str, Any]]:
    tasks = []
    for scenario in scenarios:
        for agents in scenario.agents:
            for seed in scenario.seeds:
                for time_limit in scenario.time_limits:
                    for method in scenario.solvers:
                        task: dict[str, Any] = {
                            "scenario": scenario.name,
                            "map_name": scenario.map_name,
                            "target_mode": scenario.target_mode,
                            "agents": agents,
                            "seed": seed,
                            "time_limit": time_limit,
                            "method": method,
                        }
                        match = re.match(r"^(.*)_k(\d+)$", method)
                        if match:
                            task["num_pickup_agents"] = int(match.group(2))
                        tasks.append(
                            task
                        )
                for time_limit in scenario.time_limits:
                    for method in scenario.lacam_methods:
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


def parse_int_filter(values: list[int]) -> set[int] | None:
    return set(values) if values else None


def parse_str_filter(values: list[str]) -> set[str] | None:
    return set(values) if values else None


def task_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (row["scenario"], row["agents"], row["seed"], row["time_limit"], row["method"])


def completed_keys_from_rows(paths: list[Path]) -> set[tuple[Any, ...]]:
    completed: set[tuple[Any, ...]] = set()
    for path in paths:
        if not path.exists():
            continue
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                row = json.loads(line)
                if is_completed_row(row):
                    completed.add(task_key(row))
    return completed


def filter_tasks(tasks: list[dict[str, Any]], args: argparse.Namespace) -> list[dict[str, Any]]:
    scenarios = parse_str_filter(args.scenarios)
    methods = parse_str_filter(args.methods)
    agents = parse_int_filter(args.agent_counts)
    seeds = parse_int_filter(args.seeds)
    target_modes = parse_str_filter(args.target_modes)
    return [
        task
        for task in tasks
        if (scenarios is None or task["scenario"] in scenarios)
        and (methods is None or task["method"] in methods)
        and (agents is None or int(task["agents"]) in agents)
        and (seeds is None or int(task["seed"]) in seeds)
        and (target_modes is None or task["target_mode"] in target_modes)
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paper-suite", choices=["all", "fig3", "fig4", "fig5", "fig6", "table1", "table2", "table3", "smoke"], default="smoke")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/paper_2605_07744"))
    parser.add_argument("--ir-repo", type=Path, default=Path("/home/yimin/research/ir-tapf"))
    parser.add_argument("--ir-bin", type=Path, default=Path("/home/yimin/research/ir-tapf/target/release/ir_tapf"))
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--map-root", type=Path, default=MAP_ROOT)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 4))
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--keep-failed-rows", action="store_true")
    parser.add_argument("--max-tasks", type=int, default=0)
    parser.add_argument("--skip-rows-jsonl", type=Path, action="append", default=[])
    parser.add_argument("--scenarios", nargs="*", default=[])
    parser.add_argument("--target-modes", nargs="*", choices=["random", "hotspot"], default=[])
    parser.add_argument("--agent-counts", type=int, nargs="*", default=[])
    parser.add_argument("--seeds", type=int, nargs="*", default=[])
    parser.add_argument("--methods", nargs="*", default=[])
    parser.add_argument("--skip-sum-shortest", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    smoke = args.paper_suite == "smoke"
    scenarios = paper_scenarios(args.paper_suite, smoke)
    scenario_by_name = {s.name: s for s in scenarios}
    tasks = build_tasks(scenarios)
    tasks = filter_tasks(tasks, args)
    if args.max_tasks > 0:
        tasks = tasks[: args.max_tasks]

    rows: list[dict[str, Any]] = []
    completed: set[tuple[Any, ...]] = completed_keys_from_rows(args.skip_rows_jsonl)
    if args.resume and rows_jsonl.exists():
        with rows_jsonl.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                row = json.loads(line)
                if args.keep_failed_rows or is_completed_row(row):
                    rows.append(row)
                    if is_completed_row(row):
                        completed.add(task_key(row))
    elif rows_jsonl.exists() and not args.dry_run:
        rows_jsonl.unlink()

    if args.resume and rows_jsonl.exists() and not args.keep_failed_rows and not args.dry_run:
        with rows_jsonl.open("w", encoding="utf-8") as f:
            for row in rows:
                f.write(json.dumps(row, sort_keys=True) + "\n")

    tasks = [
        t
        for t in tasks
        if task_key(t) not in completed
    ]
    print(f"scenarios={len(scenarios)} tasks={len(tasks)} completed={len(rows)} jobs={args.jobs}", flush=True)
    if args.dry_run:
        return 0

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
