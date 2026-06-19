#!/usr/bin/env python3
"""Plot per-replan planning-time distributions from lifelong trace CSV files."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


CASE_RE = re.compile(r"case_(?P<distribution>.+)_k(?P<capacity>\d+)_a(?P<agents>\d+)\.csv\.trace\.csv$")


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = pct / 100.0 * (len(ordered) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    weight = pos - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def read_case(path: Path) -> dict[str, object]:
    match = CASE_RE.match(path.name)
    if match is None:
        raise ValueError(f"cannot parse case name: {path.name}")
    rows = list(csv.DictReader(path.open("r", encoding="utf-8")))
    planning = [float(row["planning_runtime_ms"]) for row in rows]
    assignment = [float(row["assignment_time_ms"]) for row in rows]
    search = [float(row["planner_search_time_ms"]) for row in rows]
    timed_out = [row["timed_out"] == "1" for row in rows]
    return {
        "path": path,
        "distribution": match.group("distribution"),
        "capacity": int(match.group("capacity")),
        "agents": int(match.group("agents")),
        "planning": planning,
        "assignment": assignment,
        "search": search,
        "timeouts": sum(timed_out),
        "replans": len(rows),
    }


def summarize_case(case: dict[str, object]) -> dict[str, object]:
    planning = case["planning"]
    assignment = case["assignment"]
    search = case["search"]
    assert isinstance(planning, list)
    assert isinstance(assignment, list)
    assert isinstance(search, list)
    return {
        "distribution": case["distribution"],
        "multi_carry_capacity": case["capacity"],
        "num_agents": case["agents"],
        "replans": case["replans"],
        "timeouts": case["timeouts"],
        "planning_mean_ms": mean(planning) if planning else math.nan,
        "planning_p50_ms": percentile(planning, 50),
        "planning_p90_ms": percentile(planning, 90),
        "planning_p95_ms": percentile(planning, 95),
        "planning_p99_ms": percentile(planning, 99),
        "planning_max_ms": max(planning) if planning else math.nan,
        "assignment_mean_ms": mean(assignment) if assignment else math.nan,
        "search_mean_ms": mean(search) if search else math.nan,
    }


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def case_sort_key(case: dict[str, object]) -> tuple[str, int, int]:
    return (str(case["distribution"]), int(case["capacity"]), int(case["agents"]))


def readable_distribution(value: object) -> str:
    text = str(value)
    if text == "80_20_inbound":
        return "80/20 inbound"
    if text == "50_50":
        return "50/50"
    return text


def plot_cases(
    cases: list[dict[str, object]],
    out_path: Path,
    *,
    title: str,
    split_by_distribution: bool,
) -> None:
    if split_by_distribution:
        groups = []
        for distribution in sorted({case["distribution"] for case in cases}):
            groups.append((str(distribution), [c for c in cases if c["distribution"] == distribution]))
    else:
        groups = [("all", cases)]

    fig_height = max(4.8, 3.8 * len(groups))
    fig, axes = plt.subplots(len(groups), 1, figsize=(14.0, fig_height), squeeze=False)
    for ax, (group_name, group_cases) in zip(axes[:, 0], groups):
        ordered = sorted(group_cases, key=lambda c: (int(c["capacity"]), int(c["agents"]), str(c["distribution"])))
        data = [case["planning"] for case in ordered]
        labels = [
            f"{readable_distribution(case['distribution'])}\nK{case['capacity']} A{case['agents']}"
            if not split_by_distribution
            else f"K{case['capacity']}\nA{case['agents']}"
            for case in ordered
        ]
        parts = ax.violinplot(data, showmeans=False, showextrema=False, showmedians=False)
        for body in parts["bodies"]:
            body.set_facecolor("#7aa6c2")
            body.set_edgecolor("#3f6f8f")
            body.set_alpha(0.42)
        box = ax.boxplot(
            data,
            widths=0.18,
            patch_artist=True,
            showfliers=False,
            medianprops={"color": "#111111", "linewidth": 1.2},
            boxprops={"facecolor": "#ffffff", "edgecolor": "#333333", "linewidth": 0.9},
            whiskerprops={"color": "#333333", "linewidth": 0.8},
            capprops={"color": "#333333", "linewidth": 0.8},
        )
        del box
        for idx, case in enumerate(ordered, start=1):
            planning = case["planning"]
            assert isinstance(planning, list)
            p95 = percentile(planning, 95)
            max_v = max(planning) if planning else 0
            text = f"p95 {p95:.0f}\nmax {max_v:.0f}"
            if int(case["timeouts"]):
                text += f"\nto {case['timeouts']}"
            ax.text(idx, max_v * 1.08, text, ha="center", va="bottom", fontsize=7)
        ax.set_yscale("log")
        ax.set_ylabel("planning runtime per replan (ms, log)")
        ax.set_xticks(range(1, len(labels) + 1), labels)
        ax.grid(True, axis="y", which="both", alpha=0.24)
        if split_by_distribution:
            ax.set_title(readable_distribution(group_name))
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=180)
    fig.savefig(out_path.with_suffix(".pdf"))
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--prefix", default="planning_time_distribution")
    parser.add_argument("--focus-agents", type=int, default=50)
    args = parser.parse_args()

    trace_paths = sorted(args.result_dir.glob("case_*.csv.trace.csv"))
    cases = [read_case(path) for path in trace_paths]
    summaries = [summarize_case(case) for case in sorted(cases, key=case_sort_key)]
    write_summary(args.result_dir / f"{args.prefix}_summary.csv", summaries)
    plot_cases(
        cases,
        args.result_dir / f"{args.prefix}.png",
        title="Per-replan planning time distribution",
        split_by_distribution=True,
    )
    focused = [case for case in cases if int(case["agents"]) == args.focus_agents]
    if focused:
        write_summary(
            args.result_dir / f"{args.prefix}_a{args.focus_agents}_summary.csv",
            [summarize_case(case) for case in sorted(focused, key=case_sort_key)],
        )
        plot_cases(
            focused,
            args.result_dir / f"{args.prefix}_a{args.focus_agents}.png",
            title=f"A{args.focus_agents} per-replan planning time distribution",
            split_by_distribution=False,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
