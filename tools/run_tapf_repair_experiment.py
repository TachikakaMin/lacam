#!/usr/bin/env python3
"""Run a two-pass TAPF repair experiment.

Pass 1 obtains the first LaCAM-TAPF solution with anytime disabled.
The highest-cost agents are left unbound. All other agents are fixed to
their pass-1 final target, then pass 2 runs LaCAM-TAPF on the repaired TAPF.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def write_yaml(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def parse_kv(text: str) -> dict[str, str]:
    row: dict[str, str] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        row[key.strip()] = value.strip()
    return row


def agent_index(name: str) -> int:
    if name.startswith("agent") and name[len("agent") :].isdigit():
        return int(name[len("agent") :])
    raise ValueError(f"cannot parse agent name: {name}")


def coord_from_state(state: dict[str, Any]) -> tuple[int, int]:
    return int(state["x"]), int(state["y"])


def expand_sparse_path(path: list[dict[str, Any]], makespan: int) -> list[tuple[int, int]]:
    expanded: list[tuple[int, int]] = []
    idx = 0
    for t in range(makespan + 1):
        while idx + 1 < len(path) and int(path[idx + 1]["t"]) <= t:
            idx += 1
        expanded.append(coord_from_state(path[idx]))
    return expanded


def per_agent_costs(schedule_yaml: Path) -> tuple[list[dict[str, Any]], dict[int, tuple[int, int]]]:
    data = load_yaml(schedule_yaml)
    schedule = data["schedule"]
    assignments = data.get("assignments") or {}
    makespan = int((data.get("statistics") or {}).get("makespan", 0))
    if makespan <= 0:
        for path in schedule.values():
            makespan = max(makespan, max(int(step["t"]) for step in path))

    rows: list[dict[str, Any]] = []
    final_targets: dict[int, tuple[int, int]] = {}
    for name, sparse_path in sorted(schedule.items(), key=lambda kv: agent_index(kv[0])):
        idx = agent_index(name)
        path = expand_sparse_path(sparse_path, makespan)
        final = path[-1]
        if name in assignments:
            final = coord_from_state(assignments[name])
        final_targets[idx] = final

        cost = makespan
        while cost > 0 and path[cost - 1] == final:
            cost -= 1
        moves = sum(1 for t in range(1, len(path)) if path[t] != path[t - 1])
        aba = sum(
            1
            for t in range(2, len(path))
            if path[t] == path[t - 2] and path[t] != path[t - 1]
        )
        rows.append(
            {
                "agent": idx,
                "cost": cost,
                "moves": moves,
                "aba": aba,
                "final": final,
            }
        )
    return rows, final_targets


def run_tapf(
    tapf_bin: Path,
    yaml_path: Path,
    time_limit: float,
    schedule_out: Path,
    anytime: bool,
    mode: str,
    focal_weight: float,
    tie_break: str,
) -> dict[str, Any]:
    cmd = [
        str(tapf_bin),
        str(yaml_path),
        "",
        str(time_limit),
        str(schedule_out),
        "1" if anytime else "0",
        "0",
        "-1",
        mode,
        str(focal_weight),
        tie_break,
    ]
    cp = subprocess.run(cmd, text=True, capture_output=True)
    row: dict[str, Any] = parse_kv(cp.stdout)
    row["exit_code"] = cp.returncode
    row["stderr"] = cp.stderr.strip()
    return row


def make_repair_yaml(
    original_yaml: Path,
    output_yaml: Path,
    repaired_agents: set[int],
    fixed_targets: dict[int, tuple[int, int]],
) -> None:
    data = load_yaml(original_yaml)
    if "map" in data:
      map_path = Path(str(data["map"]))
      if not map_path.is_absolute():
          map_path = (original_yaml.parent / map_path).resolve()
      data["map"] = str(map_path)
    agents = data.get("agents") or []
    for idx, agent in enumerate(agents):
        if idx in repaired_agents:
            continue
        target = fixed_targets[idx]
        agent["potentialGoals"] = [[target[0], target[1]]]
        agent.pop("goal", None)
    write_yaml(output_yaml, data)


def run_case(args: argparse.Namespace, case_yaml: Path) -> dict[str, Any]:
    stem = case_yaml.stem
    case_dir = args.out_dir / stem
    case_dir.mkdir(parents=True, exist_ok=True)
    first_schedule = case_dir / "first.yaml"
    repair_yaml = case_dir / "repair.yaml"
    repair_schedule = case_dir / "repair_solution.yaml"

    first = run_tapf(
        args.tapf_bin,
        case_yaml,
        args.first_time_limit,
        first_schedule,
        False,
        args.first_mode,
        args.focal_weight,
        args.tie_break,
    )
    if first.get("valid_solution") != "1":
        return {
            "case": stem,
            "input": str(case_yaml),
            "first_valid": first.get("valid_solution", "0"),
            "repair_valid": 0,
            "error": first.get("stderr", ""),
        }

    costs, final_targets = per_agent_costs(first_schedule)
    n = len(costs)
    repair_count = max(1, math.ceil(n * args.repair_fraction))
    repair_agents = {
        row["agent"]
        for row in sorted(costs, key=lambda row: (row["cost"], row["aba"], row["moves"]), reverse=True)[
            :repair_count
        ]
    }
    top = sorted((row for row in costs if row["agent"] in repair_agents), key=lambda row: row["cost"], reverse=True)

    make_repair_yaml(case_yaml, repair_yaml, repair_agents, final_targets)
    repaired = run_tapf(
        args.tapf_bin,
        repair_yaml,
        args.repair_time_limit,
        repair_schedule,
        args.repair_anytime,
        args.repair_mode,
        args.focal_weight,
        args.tie_break,
    )

    first_soc = int(first.get("soc", 0) or 0)
    repair_soc = int(repaired.get("soc", 0) or 0)
    first_sol = int(first.get("sum_of_loss", 0) or 0)
    repair_sol = int(repaired.get("sum_of_loss", 0) or 0)
    return {
        "case": stem,
        "input": str(case_yaml),
        "repair_yaml": str(repair_yaml),
        "first_schedule": str(first_schedule),
        "repair_schedule": str(repair_schedule),
        "first_valid": first.get("valid_solution", "0"),
        "repair_valid": repaired.get("valid_solution", "0"),
        "num_agents": n,
        "repair_count": repair_count,
        "repair_agents": " ".join(str(i) for i in sorted(repair_agents)),
        "top_costs": " ".join(f"a{r['agent']}:{r['cost']}" for r in top),
        "first_soc": first_soc,
        "repair_soc": repair_soc,
        "delta_soc": repair_soc - first_soc if repair_soc else "",
        "first_sum_of_loss": first_sol,
        "repair_sum_of_loss": repair_sol,
        "delta_sum_of_loss": repair_sol - first_sol if repair_sol else "",
        "first_makespan": first.get("makespan", ""),
        "repair_makespan": repaired.get("makespan", ""),
        "first_runtime_ms": first.get("runtime_ms", ""),
        "repair_runtime_ms": repaired.get("runtime_ms", ""),
        "repair_exit_code": repaired.get("exit_code", ""),
        "repair_stderr": repaired.get("stderr", ""),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cases", type=Path, nargs="+")
    parser.add_argument("--tapf-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/tapf_repair_top10"))
    parser.add_argument("--repair-fraction", type=float, default=0.10)
    parser.add_argument("--first-time-limit", type=float, default=10.0)
    parser.add_argument("--repair-time-limit", type=float, default=10.0)
    parser.add_argument("--first-mode", choices=["dfs", "focal"], default="dfs")
    parser.add_argument("--repair-mode", choices=["dfs", "focal"], default="dfs")
    parser.add_argument("--repair-anytime", action="store_true")
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--tie-break", default="h")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = [run_case(args, case) for case in args.cases]

    csv_path = args.out_dir / "rows.csv"
    fields = sorted({key for row in rows for key in row})
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print("case first_soc repair_soc delta_soc first_sol repair_sol delta_sol agents top_costs")
    for row in rows:
        print(
            row.get("case"),
            row.get("first_soc"),
            row.get("repair_soc"),
            row.get("delta_soc"),
            row.get("first_sum_of_loss"),
            row.get("repair_sum_of_loss"),
            row.get("delta_sum_of_loss"),
            row.get("repair_agents"),
            row.get("top_costs"),
        )
    print(csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
