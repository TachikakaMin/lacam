#!/usr/bin/env python3
"""Run LaCAM-TAPF and ITA-CBS on ITA-CBS-format TAPF fixtures."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml

from validate_tapf_solution import validate


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def fixture_info(path: Path) -> Dict[str, Any]:
    data = load_yaml(path)
    agents = data.get("agents") or []
    goals_per_agent = []
    all_goals = []
    for agent in agents:
      goals = agent.get("potentialGoals")
      if goals is None:
          goals = [agent["goal"]]
      goals_per_agent.append(len(goals))
      all_goals.extend(tuple(g) for g in goals)

    m = re.search(r"agents_(\d+)_test_(\d+)\.yaml$", path.name)
    return {
        "instance_file": str(path),
        "map_file": data.get("map", ""),
        "num_agents": len(agents),
        "case_agents": int(m.group(1)) if m else len(agents),
        "case_test": int(m.group(2)) if m else -1,
        "num_unique_tasks": len(set(all_goals)),
        "potential_goals_min": min(goals_per_agent) if goals_per_agent else 0,
        "potential_goals_max": max(goals_per_agent) if goals_per_agent else 0,
        "potential_goals_avg": (
            sum(goals_per_agent) / len(goals_per_agent) if goals_per_agent else 0
        ),
    }


def fixture_sort_key(path: Path) -> Tuple[str, int, int, str]:
    m = re.search(r"agents_(\d+)_test_(\d+)\.yaml$", path.name)
    if not m:
        return (path.parent.name, 0, 0, path.name)
    return (path.parent.name, int(m.group(1)), int(m.group(2)), path.name)


def parse_kv(stdout: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if value in ("0", "1"):
            result[key] = int(value)
            continue
        try:
            if "." in value:
                result[key] = float(value)
            else:
                result[key] = int(value)
        except ValueError:
            result[key] = value
    return result


def run_lacam(
    binary: Path, fixture: Path, map_dir: Path, time_limit: float, timeout: float
) -> Dict[str, Any]:
    cmd = [str(binary), str(fixture), str(map_dir), str(time_limit)]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        return {
            "solver": "lacam_tapf",
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "solved": 0,
            "valid_solution": 0,
            "collision_free": 0,
            "timed_out": 1,
            "stderr": str(exc),
        }
    wall = time.time() - start
    row = parse_kv(cp.stdout)
    row.update(
        {
            "solver": "lacam_tapf",
            "exit_code": cp.returncode,
            "wall_time_s": wall,
            "timed_out": 0,
            "stderr": cp.stderr.strip(),
        }
    )
    if cp.returncode not in (0, 1):
        raise RuntimeError(f"LaCAM command failed rc={cp.returncode}: {cp.stderr}")
    if row.get("solved") and not row.get("collision_free"):
        raise RuntimeError(f"LaCAM reported solved but invalid for {fixture}")
    return row


def state_at(path: List[Dict[str, Any]], t: int) -> Tuple[int, int]:
    if t < len(path):
        return int(path[t]["x"]), int(path[t]["y"])
    return int(path[-1]["x"]), int(path[-1]["y"])


def schedule_makespan(output: Path) -> int:
    data = load_yaml(output)
    schedule = (data or {}).get("schedule") or {}
    if not schedule:
        return 0
    return max(len(path) - 1 for path in schedule.values())


def run_itacbs(
    binary: Path,
    fixture: Path,
    output: Path,
    time_limit: float,
    timeout: float,
) -> Dict[str, Any]:
    cmd = [str(binary), "-i", str(fixture), "-o", str(output)]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        return {
            "solver": "itacbs",
            "exit_code": 124,
            "wall_time_s": time.time() - start,
            "solved": 0,
            "valid_solution": 0,
            "collision_free": 0,
            "timed_out": 1,
            "stderr": str(exc),
        }

    wall = time.time() - start
    row: Dict[str, Any] = {
        "solver": "itacbs",
        "exit_code": cp.returncode,
        "wall_time_s": wall,
        "timed_out": 0,
        "stderr": cp.stderr.strip(),
    }
    if cp.returncode != 0:
        row.update(
            {
                "solved": 0,
                "valid_solution": 0,
                "collision_free": 0,
                "first_error": f"nonzero exit code {cp.returncode}",
            }
        )
        return row
    if not output.exists():
        row.update({"solved": 0, "valid_solution": 0, "collision_free": 0})
        return row

    out_data = load_yaml(output) or {}
    stats = out_data.get("statistics") or {}
    schedule = out_data.get("schedule") or {}
    if not schedule:
        row.update(
            {
                "solved": 0,
                "valid_solution": 0,
                "collision_free": 0,
                "first_error": "empty schedule",
                "makespan": 0,
                "soc": stats.get("cost", 0),
                "runtime_ms": float(stats.get("runtime", 0)) * 1000,
                "itacbs_TA_runtime_ms": float(stats.get("TA_runtime", 0)) * 1000,
                "itacbs_newnode_runtime_ms": float(stats.get("newnode_runtime", 0))
                * 1000,
                "itacbs_firstconflict_runtime_ms": float(
                    stats.get("firstconflict_runtime", 0)
                )
                * 1000,
                "itacbs_lowlevel_search_time_ms": float(
                    stats.get("lowlevel_search_time", 0)
                )
                * 1000,
                "itacbs_total_lowlevel_node": stats.get("total_lowlevel_node", 0),
                "itacbs_lowLevelExpanded": stats.get("lowLevelExpanded", 0),
                "itacbs_numTaskAssignments": stats.get("numTaskAssignments", 0),
                "itacbs_numTaskAssignmentChanged": stats.get(
                    "numTaskAssignmentChanged", 0
                ),
            }
        )
        return row

    errors = validate(fixture, output, require_unique_goals=True)
    row.update(
        {
            "solved": int(not errors),
            "valid_solution": int(not errors),
            "collision_free": int(not errors),
            "first_error": errors[0] if errors else "",
            "makespan": schedule_makespan(output),
            "soc": stats.get("cost", 0),
            "runtime_ms": float(stats.get("runtime", 0)) * 1000,
            "itacbs_TA_runtime_ms": float(stats.get("TA_runtime", 0)) * 1000,
            "itacbs_newnode_runtime_ms": float(stats.get("newnode_runtime", 0)) * 1000,
            "itacbs_firstconflict_runtime_ms": float(
                stats.get("firstconflict_runtime", 0)
            )
            * 1000,
            "itacbs_lowlevel_search_time_ms": float(
                stats.get("lowlevel_search_time", 0)
            )
            * 1000,
            "itacbs_total_lowlevel_node": stats.get("total_lowlevel_node", 0),
            "itacbs_lowLevelExpanded": stats.get("lowLevelExpanded", 0),
            "itacbs_numTaskAssignments": stats.get("numTaskAssignments", 0),
            "itacbs_numTaskAssignmentChanged": stats.get(
                "numTaskAssignmentChanged", 0
            ),
        }
    )
    if errors:
        raise RuntimeError(f"ITA-CBS invalid solution for {fixture}: {errors[0]}")
    return row


def discover_fixtures(fixture_dirs: List[Path], max_cases: int) -> List[Path]:
    fixtures: List[Path] = []
    for fixture_dir in fixture_dirs:
        dir_fixtures = sorted(fixture_dir.glob("*.yaml"), key=fixture_sort_key)
        fixtures.extend(dir_fixtures[:max_cases] if max_cases > 0 else dir_fixtures)
    return fixtures


def run_fixture(
    fixture: Path,
    args: argparse.Namespace,
    itacbs_out_dir: Path,
) -> List[Dict[str, Any]]:
    base = fixture_info(fixture)
    rows: List[Dict[str, Any]] = []

    lacam = run_lacam(
        args.lacam_bin,
        fixture,
        args.map_dir,
        args.time_limit,
        args.lacam_timeout,
    )
    rows.append({**base, **lacam})

    if not args.skip_itacbs:
        relative_name = "_".join(fixture.parts[-2:])
        output = itacbs_out_dir / f"{Path(relative_name).stem}.out.yaml"
        itacbs = run_itacbs(
            args.itacbs_bin,
            fixture,
            output,
            args.time_limit,
            args.itacbs_timeout,
        )
        rows.append({**base, **itacbs})

    return rows


def itacbs_output_path(fixture: Path, itacbs_out_dir: Path) -> Path:
    relative_name = "_".join(fixture.parts[-2:])
    return itacbs_out_dir / f"{Path(relative_name).stem}.out.yaml"


def run_solver_task(
    fixture: Path,
    solver: str,
    args: argparse.Namespace,
    itacbs_out_dir: Path,
) -> Dict[str, Any]:
    base = fixture_info(fixture)
    if solver == "lacam_tapf":
        result = run_lacam(
            args.lacam_bin,
            fixture,
            args.map_dir,
            args.time_limit,
            args.lacam_timeout,
        )
    elif solver == "itacbs":
        result = run_itacbs(
            args.itacbs_bin,
            fixture,
            itacbs_output_path(fixture, itacbs_out_dir),
            args.time_limit,
            args.itacbs_timeout,
        )
    else:
        raise ValueError(f"unknown solver: {solver}")
    return {**base, **result}


def write_rows(csv_path: Path, jsonl_path: Path, rows: List[Dict[str, Any]]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({key for row in rows for key in row.keys()})
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    with jsonl_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture-dir", type=Path, action="append", required=True)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--lacam-bin", type=Path, default=Path("./build/tapf_benchmark"))
    parser.add_argument(
        "--itacbs-bin",
        type=Path,
        default=Path("./third_party/ITA-CBS2/build/ITACBS_remake"),
    )
    parser.add_argument("--time-limit", type=float, default=1.0)
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Default external subprocess timeout in seconds.",
    )
    parser.add_argument(
        "--lacam-timeout",
        type=float,
        default=0.0,
        help="External timeout for LaCAM-TAPF. Defaults to --timeout.",
    )
    parser.add_argument(
        "--itacbs-timeout",
        type=float,
        default=0.0,
        help="External timeout for ITA-CBS. Defaults to --timeout.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=max(1, (os.cpu_count() or 4) // 4),
        help="Number of fixtures to run concurrently.",
    )
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument("--skip-itacbs", action="store_true")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Load existing rows and skip completed solver/fixture pairs.",
    )
    parser.add_argument(
        "--parallel-solvers",
        action="store_true",
        help="Submit each solver/fixture pair as its own parallel task.",
    )
    parser.add_argument("--out-prefix", type=Path, default=Path("build/results/tapf_full_1s"))
    args = parser.parse_args()
    if args.lacam_timeout <= 0:
        args.lacam_timeout = args.timeout
    if args.itacbs_timeout <= 0:
        args.itacbs_timeout = args.timeout

    fixtures = discover_fixtures(args.fixture_dir, args.max_cases)

    rows: List[Dict[str, Any]] = []
    itacbs_out_dir = args.out_prefix.parent / f"{args.out_prefix.name}_itacbs_outputs"
    itacbs_out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.out_prefix.with_suffix(".csv")
    jsonl_path = args.out_prefix.with_suffix(".jsonl")
    completed_keys = set()
    if args.resume and jsonl_path.exists():
        with jsonl_path.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                row = json.loads(line)
                rows.append(row)
                completed_keys.add((row.get("instance_file"), row.get("solver")))

    if args.parallel_solvers:
        solvers = ["lacam_tapf"] if args.skip_itacbs else ["lacam_tapf", "itacbs"]
        tasks = [
            (fixture, solver)
            for fixture in fixtures
            for solver in solvers
            if (str(fixture), solver) not in completed_keys
        ]
        total = len(tasks)
        print(
            f"Running {total} solver tasks for {len(fixtures)} fixtures with "
            f"jobs={args.jobs}, time_limit={args.time_limit}s, "
            f"lacam_timeout={args.lacam_timeout}s, "
            f"itacbs_timeout={args.itacbs_timeout}s, "
            f"resumed_rows={len(rows)}"
        )
        executor = concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs)
        try:
            future_to_task = {
                executor.submit(run_solver_task, fixture, solver, args, itacbs_out_dir): (
                    fixture,
                    solver,
                )
                for fixture, solver in tasks
            }
            completed = 0
            for future in concurrent.futures.as_completed(future_to_task):
                fixture, solver = future_to_task[future]
                completed += 1
                try:
                    row = future.result()
                except Exception:
                    write_rows(csv_path, jsonl_path, rows)
                    print(
                        f"[{completed}/{total}] ERROR {solver} {fixture}",
                        file=sys.stderr,
                    )
                    for pending in future_to_task:
                        pending.cancel()
                    try:
                        executor.shutdown(wait=False, cancel_futures=True)
                    except TypeError:
                        executor.shutdown(wait=False)
                    raise
                rows.append(row)
                write_rows(csv_path, jsonl_path, rows)
                print(
                    f"[{completed}/{total}] {solver} {fixture} "
                    f"solved={row.get('solved', 0)}"
                )
        finally:
            try:
                executor.shutdown(wait=True, cancel_futures=False)
            except TypeError:
                executor.shutdown(wait=True)

        return 0

    print(
        f"Running {len(fixtures)} fixtures with jobs={args.jobs}, "
        f"time_limit={args.time_limit}s, lacam_timeout={args.lacam_timeout}s, "
        f"itacbs_timeout={args.itacbs_timeout}s"
    )
    executor = concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs)
    try:
        future_to_fixture = {
            executor.submit(run_fixture, fixture, args, itacbs_out_dir): fixture
            for fixture in fixtures
        }
        completed = 0
        for future in concurrent.futures.as_completed(future_to_fixture):
            fixture = future_to_fixture[future]
            completed += 1
            try:
                new_rows = future.result()
            except Exception:
                write_rows(csv_path, jsonl_path, rows)
                print(f"[{completed}/{len(fixtures)}] ERROR {fixture}", file=sys.stderr)
                for pending in future_to_fixture:
                    pending.cancel()
                try:
                    executor.shutdown(wait=False, cancel_futures=True)
                except TypeError:
                    executor.shutdown(wait=False)
                raise
            rows.extend(new_rows)
            write_rows(csv_path, jsonl_path, rows)
            solved = ", ".join(
                f"{row['solver']}:solved={row.get('solved', 0)}"
                for row in new_rows
            )
            print(f"[{completed}/{len(fixtures)}] {fixture} {solved}")
    finally:
        try:
            executor.shutdown(wait=True, cancel_futures=False)
        except TypeError:
            executor.shutdown(wait=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
