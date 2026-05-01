#!/usr/bin/env python3
"""Run LaCAM-TAPF and ITA-CBS on ITA-CBS-format TAPF fixtures."""

from __future__ import annotations

import argparse
import csv
import json
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
    cp = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
    wall = time.time() - start
    row = parse_kv(cp.stdout)
    row.update(
        {
            "solver": "lacam_tapf",
            "exit_code": cp.returncode,
            "wall_time_s": wall,
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
        raise RuntimeError(f"ITA-CBS command failed rc={cp.returncode}: {cp.stderr}")
    if not output.exists():
        row.update({"solved": 0, "valid_solution": 0, "collision_free": 0})
        return row

    out_data = load_yaml(output) or {}
    stats = out_data.get("statistics") or {}
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
    parser.add_argument("--fixture-dir", type=Path, required=True)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--lacam-bin", type=Path, default=Path("./build/tapf_benchmark"))
    parser.add_argument(
        "--itacbs-bin",
        type=Path,
        default=Path("./third_party/ITA-CBS2/build/ITACBS_remake"),
    )
    parser.add_argument("--time-limit", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument("--skip-itacbs", action="store_true")
    parser.add_argument("--out-prefix", type=Path, default=Path("build/results/tapf_full_1s"))
    args = parser.parse_args()

    fixtures = sorted(args.fixture_dir.glob("*.yaml"))
    fixtures = [
        path for path in fixtures
        if (load_yaml(path).get("agents") or [])
        and "potentialGoals" in load_yaml(path)["agents"][0]
    ]
    if args.max_cases > 0:
        fixtures = fixtures[: args.max_cases]

    rows: List[Dict[str, Any]] = []
    itacbs_out_dir = args.out_prefix.parent / "itacbs_outputs"
    itacbs_out_dir.mkdir(parents=True, exist_ok=True)

    for idx, fixture in enumerate(fixtures, 1):
        base = fixture_info(fixture)
        print(f"[{idx}/{len(fixtures)}] {fixture}")
        lacam = run_lacam(
            args.lacam_bin, fixture, args.map_dir, args.time_limit, args.timeout
        )
        rows.append({**base, **lacam})

        if not args.skip_itacbs:
            output = itacbs_out_dir / f"{fixture.stem}.out.yaml"
            itacbs = run_itacbs(
                args.itacbs_bin, fixture, output, args.time_limit, args.timeout
            )
            rows.append({**base, **itacbs})

        write_rows(args.out_prefix.with_suffix(".csv"), args.out_prefix.with_suffix(".jsonl"), rows)

    return 0


if __name__ == "__main__":
    sys.exit(main())
