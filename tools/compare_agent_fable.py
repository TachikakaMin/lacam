#!/usr/bin/env python3
"""Differentially test position-only TAPF against pre-motion agent_fable."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re
import statistics
import subprocess


STABLE_METRICS = (
    "valid_instance", "solved", "valid_solution", "start_valid",
    "moves_valid", "collision_free", "goal_valid",
    "unique_goal_assignment", "makespan", "soc", "sum_of_loss",
    "hl_loop_iterations", "hl_nodes_created", "hl_nodes_explored",
    "hl_reinsertions", "hl_duplicate_configs", "open_max_size",
    "solution_depth", "constraints_popped", "constraints_generated",
    "constraint_failures", "pibt_calls", "pibt_failures",
    "pibt_recursions", "assignment_calls", "assignment_changes",
    "final_assignment_changes", "final_agent_assignment_changes",
    "solution_cost", "first_solution_cost", "incumbent_updates",
    "solution_parent_edge_cost", "anytime_cost_updates", "swap_checks",
    "swap_applied", "timed_out",
)


def parse_key_values(output: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in output.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def case_seed(path: Path) -> int:
    match = re.search(r"-s(-?\d+)$", path.stem)
    return int(match.group(1)) if match else -1


def run(command: list[str], timeout: float) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return subprocess.CompletedProcess(command, 124, output)


def schedules_equal(old_path: Path, new_path: Path) -> bool:
    old_exists, new_exists = old_path.exists(), new_path.exists()
    if old_exists != new_exists:
        return False
    return not old_exists or old_path.read_bytes() == new_path.read_bytes()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--old-binary", type=Path, required=True)
    parser.add_argument("--new-binary", type=Path,
                        default=Path("build/tapf_benchmark"))
    parser.add_argument("--cases", type=Path,
                        default=Path("experiments/mawpf_paper_comparison/cases"))
    parser.add_argument("--output", type=Path,
                        default=Path("/tmp/agent_fable_position_only_compare"))
    parser.add_argument("--time-limit", type=float, default=3.0)
    parser.add_argument("--process-timeout", type=float, default=15.0)
    parser.add_argument("--max-cases", type=int, default=0)
    args = parser.parse_args()

    old_binary = args.old_binary.resolve()
    new_binary = args.new_binary.resolve()
    cases = sorted(args.cases.resolve().glob("*.yaml"))
    if args.max_cases > 0:
        cases = cases[:args.max_cases]
    if not old_binary.is_file() or not new_binary.is_file():
        parser.error("both benchmark binaries must exist")
    if not cases:
        parser.error("no YAML cases found")

    output = args.output.resolve()
    schedules = output / "schedules"
    raw = output / "raw"
    schedules.mkdir(parents=True, exist_ok=True)
    raw.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []

    for index, case in enumerate(cases):
        seed = case_seed(case)
        old_schedule = schedules / f"{case.stem}.old.yaml"
        new_schedule = schedules / f"{case.stem}.new.yaml"
        for stale in (old_schedule, new_schedule,
                      Path(str(old_schedule) + ".bin"),
                      Path(str(new_schedule) + ".bin")):
            stale.unlink(missing_ok=True)
        common = [str(case), "", str(args.time_limit)]
        old_command = [
            str(old_binary), *common, str(old_schedule), "0", "0", str(seed),
            "dfs", "1.5", "h",
        ]
        # All motion behavior is screened.  Its seven action costs remain one
        # so the recorded compatibility configuration is unambiguous.
        new_command = [
            str(new_binary), *common, str(new_schedule), "0", "0", str(seed),
            "dfs", "1.5", "h", "0", "0", "0", "0", "disabled",
            "1,1,1,1,1,1,1", "0",
        ]
        # Alternate launch order to reduce systematic warm-cache bias in the
        # runtime comparison.  Functional comparison is deterministic.
        if index % 2:
            new_run = run(new_command, args.process_timeout)
            old_run = run(old_command, args.process_timeout)
        else:
            old_run = run(old_command, args.process_timeout)
            new_run = run(new_command, args.process_timeout)
        (raw / f"{case.stem}.old.txt").write_text(old_run.stdout)
        (raw / f"{case.stem}.new.txt").write_text(new_run.stdout)
        old_values = parse_key_values(old_run.stdout)
        new_values = parse_key_values(new_run.stdout)
        metric_differences = {
            key: [old_values.get(key), new_values.get(key)]
            for key in STABLE_METRICS
            if old_values.get(key) != new_values.get(key)
        }
        old_binary_schedule = Path(str(old_schedule) + ".bin")
        new_binary_schedule = Path(str(new_schedule) + ".bin")
        rows.append({
            "case": case.stem,
            "old_exit": old_run.returncode,
            "new_exit": new_run.returncode,
            "same_exit": old_run.returncode == new_run.returncode,
            "same_metrics": not metric_differences,
            "same_schedule": schedules_equal(old_binary_schedule,
                                               new_binary_schedule),
            "old_runtime_ms": float(old_values.get("runtime_ms", "nan")),
            "new_runtime_ms": float(new_values.get("runtime_ms", "nan")),
            "old_solved": int(old_values.get("solved", "0") == "1"),
            "new_solved": int(new_values.get("solved", "0") == "1"),
            "metric_differences": json.dumps(metric_differences,
                                               sort_keys=True),
        })

    comparable = [row for row in rows if row["same_schedule"] and
                  row["same_metrics"] and row["old_solved"] and
                  row["new_solved"]]
    ratios = [float(row["new_runtime_ms"]) / float(row["old_runtime_ms"])
              for row in comparable if float(row["old_runtime_ms"]) > 0]
    differences = [row for row in rows if not row["same_exit"] or
                   not row["same_metrics"] or not row["same_schedule"]]
    summary = {
        "configuration": {
            "search_mode": "dfs", "anytime": False,
            "force_full_assignment": False, "motion": False,
            "motion_actions": "disabled",
            "motion_action_costs": [1, 1, 1, 1, 1, 1, 1],
            "max_speed": 0, "rotation_steps": 0, "path_length": 0,
            "follower_collisions": False,
        },
        "old_binary": str(old_binary),
        "new_binary": str(new_binary),
        "time_limit_sec": args.time_limit,
        "cases": len(rows),
        "same_exit": sum(bool(row["same_exit"]) for row in rows),
        "same_metrics": sum(bool(row["same_metrics"]) for row in rows),
        "same_schedule": sum(bool(row["same_schedule"]) for row in rows),
        "old_solved": sum(int(row["old_solved"]) for row in rows),
        "new_solved": sum(int(row["new_solved"]) for row in rows),
        "runtime_median_ms": {
            "old": statistics.median(float(row["old_runtime_ms"])
                                     for row in comparable),
            "new": statistics.median(float(row["new_runtime_ms"])
                                     for row in comparable),
        } if comparable else None,
        "new_over_old_runtime_ratio_median": (
            statistics.median(ratios) if ratios else None),
        "differences": differences,
    }
    with (output / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 1 if differences else 0


if __name__ == "__main__":
    raise SystemExit(main())
