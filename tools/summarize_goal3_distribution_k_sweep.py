#!/usr/bin/env python3
"""Write a compact markdown summary for the goal3 throughput sweep."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("./build/results/goal3_distribution_k_sweep/throughput_sweep_results.csv"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("./build/results/goal3_distribution_k_sweep/summary.md"),
    )
    args = parser.parse_args()

    rows = list(csv.DictReader(args.input.open("r", encoding="utf-8")))
    rows.sort(
        key=lambda r: (
            r["distribution"],
            int(r["requested_capacity"]),
            int(r["num_agents"]),
        )
    )
    lines = [
        "# Goal3 Distribution/K Throughput Sweep",
        "",
        "Map: `tests/assets/symbotic_star.map`",
        "",
        "Horizon: 1000",
        "",
        "Seed: 0",
        "",
        "| distribution | K | agents | throughput | completed | invocations | partial | timeout | failure |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {distribution} | {k} | {agents} | {throughput} | {completed} | "
            "{invocations} | {partial} | {timeout} | {failure} |".format(
                distribution=row["distribution"],
                k=row["requested_capacity"],
                agents=row["num_agents"],
                throughput=row["throughput"],
                completed=row["completed_tasks"],
                invocations=row["planner_invocations"],
                partial=row["planner_partial_solution_count"],
                timeout=row["planner_timeout_count"],
                failure=row["planner_failure_count"],
            )
        )
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
