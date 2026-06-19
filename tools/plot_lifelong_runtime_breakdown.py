#!/usr/bin/env python3
"""Plot wall-clock runtime breakdowns for lifelong benchmark CSV files."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COMPONENTS = [
    ("assignment_sec", "Assignment/Hungarian", "#e41a1c"),
    ("planner_search_other_sec", "Planner search/PIBT/other", "#377eb8"),
    ("non_planner_sim_sec", "Non-planner simulation", "#ffb000"),
    ("runner_overhead_sec", "Runner overhead", "#8f9aa8"),
]


def safe_float(value: object, default: float = math.nan) -> float:
    try:
        if value is None or value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: object, default: int = 0) -> int:
    try:
        if value is None or value == "":
            return default
        return int(float(value))
    except (TypeError, ValueError):
        return default


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def normalize_distribution(row: dict[str, str]) -> str:
    if row.get("distribution"):
        return row["distribution"]
    outbound = safe_float(row.get("outbound_prob"))
    if math.isfinite(outbound):
        return f"{int(round((1.0 - outbound) * 100))}_{int(round(outbound * 100))}_inbound"
    return "unknown"


def build_breakdown(rows: Iterable[dict[str, str]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for row in rows:
        simulation_sec = safe_float(row.get("total_simulation_runtime"), 0.0) / 1000.0
        planner_sec = safe_float(row.get("total_planner_runtime"), 0.0) / 1000.0
        assignment_sec = safe_float(row.get("total_assignment_runtime"), 0.0) / 1000.0
        if "total_planner_search_runtime" in row:
            planner_search_sec = safe_float(row.get("total_planner_search_runtime"), 0.0) / 1000.0
        else:
            planner_search_sec = max(0.0, planner_sec - assignment_sec)

        wall_sec = safe_float(row.get("case_elapsed_sec"))
        if not math.isfinite(wall_sec) or wall_sec <= 0:
            wall_sec = simulation_sec

        non_planner_sim_sec = max(0.0, simulation_sec - planner_sec)
        runner_overhead_sec = max(0.0, wall_sec - simulation_sec)
        component_total = (
            assignment_sec
            + planner_search_sec
            + non_planner_sim_sec
            + runner_overhead_sec
        )
        residual_sec = max(0.0, wall_sec - component_total)
        if residual_sec > 1e-6:
            non_planner_sim_sec += residual_sec

        out.append(
            {
                "distribution": normalize_distribution(row),
                "multi_carry_capacity": safe_int(row.get("multi_carry_capacity"), 1),
                "num_agents": safe_int(row.get("num_agents"), 0),
                "throughput": safe_float(row.get("throughput")),
                "wall_sec": wall_sec,
                "simulation_sec": simulation_sec,
                "planner_sec": planner_sec,
                "assignment_sec": assignment_sec,
                "planner_search_other_sec": planner_search_sec,
                "non_planner_sim_sec": non_planner_sim_sec,
                "runner_overhead_sec": runner_overhead_sec,
                "assignment_pct_wall": 100.0 * assignment_sec / wall_sec if wall_sec else math.nan,
                "planner_search_other_pct_wall": 100.0 * planner_search_sec / wall_sec if wall_sec else math.nan,
                "non_planner_sim_pct_wall": 100.0 * non_planner_sim_sec / wall_sec if wall_sec else math.nan,
                "runner_overhead_pct_wall": 100.0 * runner_overhead_sec / wall_sec if wall_sec else math.nan,
                "assignment_pct_planner": 100.0 * assignment_sec / planner_sec if planner_sec else math.nan,
                "planner_search_other_pct_planner": 100.0 * planner_search_sec / planner_sec if planner_sec else math.nan,
                "planner_invocations": safe_int(row.get("planner_invocations")),
                "planner_timeout_count": safe_int(row.get("planner_timeout_count")),
                "assignment_row_cache_requests": safe_int(row.get("assignment_row_cache_requests")),
                "assignment_row_cache_hit_rate": safe_float(row.get("assignment_row_cache_hit_rate")),
                "valid": safe_int(row.get("valid"), 1),
                "returncode": safe_int(row.get("returncode")),
            }
        )
    return out


def write_breakdown_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def sort_key(row: dict[str, object]) -> tuple[int, int, str]:
    return (
        int(row["multi_carry_capacity"]),
        int(row["num_agents"]),
        str(row["distribution"]),
    )


def plot_breakdown(
    rows: list[dict[str, object]],
    out_path: Path,
    *,
    title: str,
    split_by_distribution: bool,
) -> None:
    if not rows:
        raise ValueError("no rows to plot")

    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    if split_by_distribution:
        for row in rows:
            grouped[str(row["distribution"])].append(row)
    else:
        grouped["all"] = rows

    group_items = sorted(grouped.items())
    fig_height = max(4.4, 3.8 * len(group_items))
    fig, axes = plt.subplots(
        len(group_items),
        1,
        figsize=(12.8, fig_height),
        sharey=len(group_items) > 1,
        squeeze=False,
    )

    for ax, (group_name, group_rows) in zip(axes[:, 0], group_items):
        ordered = sorted(group_rows, key=sort_key)
        labels = [
            f"{row['distribution']}\nK{row['multi_carry_capacity']} A{row['num_agents']}"
            if not split_by_distribution
            else f"K{row['multi_carry_capacity']}\nA{row['num_agents']}"
            for row in ordered
        ]
        xs = list(range(len(ordered)))
        bottom = [0.0] * len(ordered)
        for key, label, color in COMPONENTS:
            values = [float(row[key]) for row in ordered]
            ax.bar(xs, values, bottom=bottom, label=label, color=color)
            bottom = [b + v for b, v in zip(bottom, values)]

        for x, row, total in zip(xs, ordered, bottom):
            timeouts = int(row["planner_timeout_count"])
            throughput = float(row["throughput"])
            note = f"{total:.1f}s\nthr {throughput:.3f}"
            if timeouts:
                note += f"\nto {timeouts}"
            ax.text(x, total + max(bottom) * 0.015, note, ha="center", va="bottom", fontsize=7.5)

        ax.set_xticks(xs, labels)
        ax.set_ylabel("runtime (s)")
        ax.grid(True, axis="y", alpha=0.25)
        if split_by_distribution:
            readable = group_name.replace("50_50", "50/50").replace("80_20_inbound", "80/20 inbound-heavy")
            ax.set_title(readable)

    axes[0, 0].legend(loc="upper left", ncols=4, frameon=False)
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    fig.savefig(out_path.with_suffix(".pdf"))
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="benchmark result CSV")
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--prefix", default="runtime_breakdown")
    parser.add_argument("--focus-agents", type=int, default=0)
    args = parser.parse_args()

    out_dir = args.out_dir or args.csv.parent
    rows = build_breakdown(read_rows(args.csv))
    write_breakdown_csv(out_dir / f"{args.prefix}.csv", rows)
    plot_breakdown(
        rows,
        out_dir / f"{args.prefix}_stacked.png",
        title="Lifelong benchmark wall-clock runtime breakdown",
        split_by_distribution=True,
    )

    if args.focus_agents:
        focused = [row for row in rows if int(row["num_agents"]) == args.focus_agents]
        write_breakdown_csv(out_dir / f"{args.prefix}_a{args.focus_agents}.csv", focused)
        plot_breakdown(
            focused,
            out_dir / f"{args.prefix}_a{args.focus_agents}_stacked.png",
            title=f"A{args.focus_agents} wall-clock runtime breakdown",
            split_by_distribution=False,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
