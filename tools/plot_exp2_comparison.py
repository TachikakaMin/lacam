#!/usr/bin/env python3
"""Plot exp2 TAPF comparisons for ITA-CBS, LaCAM full TA, and LaCAM incremental TA."""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


SERIES = [
    ("itacbs", "ITA-CBS", "s"),
    ("lacam_full_ta", "LaCAM full TA", "o"),
    ("lacam_incremental_ta", "LaCAM incremental TA", "^"),
]
SERIES_LABEL = {key: label for key, label, _ in SERIES}
SERIES_MARKER = {key: marker for key, _, marker in SERIES}

RATIO_RE = re.compile(r"_ratio_(\d+)$")

LACAM_METRICS = [
    "soc",
    "makespan",
    "runtime_ms",
    "assignment_time_ms",
    "assignment_calls",
    "assignment_changes",
    "final_assignment_changes",
    "final_agent_assignment_changes",
]
ITACBS_METRIC_MAP = {
    "soc": "soc",
    "makespan": "makespan",
    "runtime_ms": "runtime_ms",
    "assignment_time_ms": "itacbs_TA_runtime_ms",
    "assignment_calls": "itacbs_numTaskAssignments",
    "assignment_changes": "itacbs_numTaskAssignmentChanged",
}

METRIC_LABELS = {
    "soc": "sum of costs",
    "makespan": "makespan",
    "runtime_ms": "runtime (ms)",
    "assignment_time_ms": "task-assignment time (ms)",
    "assignment_calls": "task-assignment calls",
    "assignment_changes": "task-assignment changes",
    "final_assignment_changes": "final assignment changes",
    "final_agent_assignment_changes": "final per-agent assignment changes",
}


def read_rows(path: Path, series: str, only_solver: str | None = None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if only_solver is not None and row.get("solver") != only_solver:
                continue
            if not is_exp2_instance(row.get("instance_file", "")):
                continue
            normalized = normalize_row(row, series)
            if normalized is not None:
                rows.append(normalized)
    return rows


def is_exp2_instance(instance_file: str) -> bool:
    return "/paper_" in instance_file and "_ratio_" in Path(instance_file).parent.name


def safe_int(value: Any) -> int | None:
    try:
        if value == "" or value is None:
            return None
        return int(float(value))
    except (TypeError, ValueError):
        return None


def safe_float(value: Any) -> float:
    try:
        if value == "" or value is None:
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def is_true(value: Any) -> bool:
    return str(value) == "1"


def normalize_row(row: dict[str, Any], series: str) -> dict[str, Any] | None:
    instance_file = row.get("instance_file", "")
    parent = Path(instance_file).parent.name
    ratio_match = RATIO_RE.search(parent)
    if ratio_match is None:
        return None

    solved = is_true(row.get("solved")) and row.get("valid_solution", "1") != "0"
    normalized: dict[str, Any] = {
        "series": series,
        "series_label": SERIES_LABEL[series],
        "case_key": instance_file,
        "instance_file": instance_file,
        "map_group": parent[: ratio_match.start()],
        "common_goal_ratio": int(ratio_match.group(1)),
        "case_agents": safe_int(row.get("case_agents")),
        "case_test": safe_int(row.get("case_test")),
        "solved": int(solved),
        "valid_solution": int(row.get("valid_solution", "1") != "0"),
    }

    if series == "itacbs":
        for metric, source in ITACBS_METRIC_MAP.items():
            normalized[metric] = safe_float(row.get(source)) if solved else math.nan
        for metric in set(LACAM_METRICS) - set(ITACBS_METRIC_MAP):
            normalized[metric] = math.nan
    else:
        for metric in LACAM_METRICS:
            normalized[metric] = safe_float(row.get(metric)) if solved else math.nan
    return normalized


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def finite_values(rows: Iterable[dict[str, Any]], metric: str) -> list[float]:
    vals = []
    for row in rows:
        value = safe_float(row.get(metric))
        if math.isfinite(value):
            vals.append(value)
    return vals


def summarize(rows: list[dict[str, Any]], group_keys: list[str], metrics: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        key = tuple(row.get(k) for k in group_keys)
        groups[key].append(row)

    out: list[dict[str, Any]] = []
    for key, group in sorted(groups.items(), key=lambda item: tuple(str(v) for v in item[0])):
        summary = {group_keys[i]: key[i] for i in range(len(group_keys))}
        summary["cases"] = len(group)
        summary["solved"] = sum(int(row["solved"]) for row in group)
        summary["solve_rate"] = summary["solved"] / summary["cases"] if summary["cases"] else math.nan
        for metric in metrics:
            vals = finite_values(group, metric)
            summary[f"mean_{metric}"] = mean(vals) if vals else math.nan
        out.append(summary)
    return out


def common_solved_keys(rows: list[dict[str, Any]], series_names: set[str]) -> set[str]:
    solved_by_series: dict[str, set[str]] = {series: set() for series in series_names}
    for row in rows:
        series = row["series"]
        if series in series_names and row["solved"]:
            solved_by_series[series].add(row["case_key"])
    if not solved_by_series:
        return set()
    return set.intersection(*solved_by_series.values())


def filter_common_solved(rows: list[dict[str, Any]], series_names: set[str]) -> list[dict[str, Any]]:
    keys = common_solved_keys(rows, series_names)
    return [row for row in rows if row["series"] in series_names and row["case_key"] in keys]


def plot_line(
    summaries: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    ylabel: str,
    title: str,
    out_path: Path,
) -> None:
    plt.figure(figsize=(8, 5.2))
    plotted = False
    for series, label, marker in SERIES:
        points = [row for row in summaries if row.get("series") == series]
        xs: list[float] = []
        ys: list[float] = []
        for row in sorted(points, key=lambda r: safe_float(r.get(x_key))):
            x = safe_float(row.get(x_key))
            y = safe_float(row.get(y_key))
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x)
                ys.append(y)
        if xs:
            plt.plot(xs, ys, marker=marker, linewidth=1.8, label=label)
            plotted = True
    plt.xlabel(x_key.replace("_", " "))
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True, alpha=0.25)
    if plotted:
        plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=180)
    plt.close()


def plot_full_incremental_scatter(rows: list[dict[str, Any]], metric: str, out_path: Path) -> None:
    by_series_key: dict[tuple[str, str], dict[str, Any]] = {}
    for row in rows:
        if row["series"] in {"lacam_full_ta", "lacam_incremental_ta"}:
            by_series_key[(row["series"], row["case_key"])] = row

    xs: list[float] = []
    ys: list[float] = []
    colors: list[int] = []
    for (_, case_key), full_row in sorted(by_series_key.items()):
        if full_row["series"] != "lacam_full_ta":
            continue
        incr_row = by_series_key.get(("lacam_incremental_ta", case_key))
        if incr_row is None or not full_row["solved"] or not incr_row["solved"]:
            continue
        x = safe_float(full_row.get(metric))
        y = safe_float(incr_row.get(metric))
        if math.isfinite(x) and math.isfinite(y):
            xs.append(x)
            ys.append(y)
            colors.append(int(full_row.get("case_agents") or 0))

    plt.figure(figsize=(6.5, 6))
    scatter = plt.scatter(xs, ys, c=colors, cmap="viridis", s=22, alpha=0.75) if xs else None
    if xs:
        lo = min(min(xs), min(ys))
        hi = max(max(xs), max(ys))
        plt.plot([lo, hi], [lo, hi], color="black", linestyle="--", linewidth=1, label="equal")
        plt.colorbar(scatter, label="agents")
        plt.legend()
    plt.xlabel(f"full TA {METRIC_LABELS[metric]}")
    plt.ylabel(f"incremental TA {METRIC_LABELS[metric]}")
    plt.title(f"Full TA vs Incremental TA: {METRIC_LABELS[metric]}")
    plt.grid(True, alpha=0.2)
    plt.tight_layout()
    plt.savefig(out_path, dpi=180)
    plt.close()


def build_pair_summaries(rows: list[dict[str, Any]], out_dir: Path) -> None:
    by_series_key = {(row["series"], row["case_key"]): row for row in rows}
    paired_rows: list[dict[str, Any]] = []
    mismatch_counts = {metric: 0 for metric in ["soc", "makespan", "assignment_changes", "final_assignment_changes", "final_agent_assignment_changes"]}
    common_solved = 0

    full_keys = {case_key for series, case_key in by_series_key if series == "lacam_full_ta"}
    incr_keys = {case_key for series, case_key in by_series_key if series == "lacam_incremental_ta"}
    for case_key in sorted(full_keys & incr_keys):
        full = by_series_key[("lacam_full_ta", case_key)]
        incr = by_series_key[("lacam_incremental_ta", case_key)]
        if not full["solved"] or not incr["solved"]:
            continue
        common_solved += 1
        paired: dict[str, Any] = {
            "case_key": case_key,
            "case_agents": full.get("case_agents"),
            "case_test": full.get("case_test"),
            "map_group": full.get("map_group"),
            "common_goal_ratio": full.get("common_goal_ratio"),
        }
        for metric in ["soc", "makespan", "solution_cost", "assignment_changes", "final_assignment_changes", "final_agent_assignment_changes"]:
            full_value = full.get(metric, full.get("soc") if metric == "solution_cost" else math.nan)
            incr_value = incr.get(metric, incr.get("soc") if metric == "solution_cost" else math.nan)
            paired[f"full_{metric}"] = full_value
            paired[f"incremental_{metric}"] = incr_value
            if metric in mismatch_counts and safe_float(full_value) != safe_float(incr_value):
                mismatch_counts[metric] += 1
        paired_rows.append(paired)

    paired_fields = [
        "case_key",
        "case_agents",
        "case_test",
        "map_group",
        "common_goal_ratio",
        "full_soc",
        "incremental_soc",
        "full_makespan",
        "incremental_makespan",
        "full_solution_cost",
        "incremental_solution_cost",
        "full_assignment_changes",
        "incremental_assignment_changes",
        "full_final_assignment_changes",
        "incremental_final_assignment_changes",
        "full_final_agent_assignment_changes",
        "incremental_final_agent_assignment_changes",
    ]
    write_csv(out_dir / "paired_common_solved.csv", paired_rows, paired_fields)
    write_csv(
        out_dir / "mismatch_summary.csv",
        [
            {"metric": metric, "common_solved": common_solved, "mismatches": count}
            for metric, count in mismatch_counts.items()
        ],
        ["metric", "common_solved", "mismatches"],
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--full-ta", type=Path, default=Path("build/results/exp2_10s_lacam_fullta.csv"))
    parser.add_argument("--incremental-ta", type=Path, default=Path("build/results/exp2_10s_lacam_incrta_noanytime.csv"))
    parser.add_argument("--itacbs", type=Path, default=Path("build/results/full_10s_solver_parallel.csv"))
    parser.add_argument("--out-dir", type=Path, default=Path("build/plots/exp2_corrected_full_vs_incr"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    rows.extend(read_rows(args.itacbs, "itacbs", only_solver="itacbs"))
    rows.extend(read_rows(args.full_ta, "lacam_full_ta", only_solver="lacam_tapf"))
    rows.extend(read_rows(args.incremental_ta, "lacam_incremental_ta", only_solver="lacam_tapf"))

    metric_fields = LACAM_METRICS
    lacam_common_rows = filter_common_solved(rows, {"lacam_full_ta", "lacam_incremental_ta"})
    all_solver_common_rows = filter_common_solved(
        rows, {"itacbs", "lacam_full_ta", "lacam_incremental_ta"}
    )

    overall = summarize(rows, ["series", "series_label"], metric_fields)
    by_agent = summarize(rows, ["series", "series_label", "case_agents"], metric_fields)
    by_ratio = summarize(rows, ["series", "series_label", "common_goal_ratio"], metric_fields)
    lacam_common_by_agent = summarize(
        lacam_common_rows, ["series", "series_label", "case_agents"], metric_fields
    )
    lacam_common_by_ratio = summarize(
        lacam_common_rows, ["series", "series_label", "common_goal_ratio"], metric_fields
    )
    all_common_by_agent = summarize(
        all_solver_common_rows, ["series", "series_label", "case_agents"], metric_fields
    )
    all_common_by_ratio = summarize(
        all_solver_common_rows, ["series", "series_label", "common_goal_ratio"], metric_fields
    )

    summary_fields = ["series", "series_label", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields]
    write_csv(args.out_dir / "summary_overall.csv", overall, summary_fields)
    write_csv(args.out_dir / "summary_by_agent.csv", by_agent, ["series", "series_label", "case_agents", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])
    write_csv(args.out_dir / "summary_by_ratio.csv", by_ratio, ["series", "series_label", "common_goal_ratio", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])
    write_csv(args.out_dir / "summary_lacam_common_solved_by_agent.csv", lacam_common_by_agent, ["series", "series_label", "case_agents", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])
    write_csv(args.out_dir / "summary_lacam_common_solved_by_ratio.csv", lacam_common_by_ratio, ["series", "series_label", "common_goal_ratio", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])
    write_csv(args.out_dir / "summary_all_common_solved_by_agent.csv", all_common_by_agent, ["series", "series_label", "case_agents", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])
    write_csv(args.out_dir / "summary_all_common_solved_by_ratio.csv", all_common_by_ratio, ["series", "series_label", "common_goal_ratio", "cases", "solved", "solve_rate"] + [f"mean_{m}" for m in metric_fields])

    plot_line(by_agent, "case_agents", "solve_rate", "solve rate", "Solve Rate vs Agent Count", args.out_dir / "solve_rate_vs_agent_count.png")
    plot_line(by_ratio, "common_goal_ratio", "solve_rate", "solve rate", "Solve Rate vs Common Goal Ratio", args.out_dir / "solve_rate_by_ratio.png")

    paired_lacam_metrics = {
        "soc",
        "makespan",
        "final_assignment_changes",
        "final_agent_assignment_changes",
    }
    all_common_metrics = {
        "soc",
        "makespan",
        "assignment_time_ms",
        "assignment_calls",
        "assignment_changes",
    }
    for metric in metric_fields:
        y_key = f"mean_{metric}"
        source_by_agent = by_agent
        source_by_ratio = by_ratio
        title_suffix = ""
        if metric in paired_lacam_metrics:
            source_by_agent = lacam_common_by_agent
            source_by_ratio = lacam_common_by_ratio
            title_suffix = " (LaCAM Common Solved)"
        if metric in all_common_metrics:
            source_by_agent = all_common_by_agent
            source_by_ratio = all_common_by_ratio
            title_suffix = " (All-Solver Common Solved)"
        plot_line(source_by_agent, "case_agents", y_key, METRIC_LABELS[metric], f"{METRIC_LABELS[metric].title()} vs Agent Count{title_suffix}", args.out_dir / f"{metric}_vs_agent_count.png")
        plot_line(source_by_ratio, "common_goal_ratio", y_key, METRIC_LABELS[metric], f"{METRIC_LABELS[metric].title()} vs Common Goal Ratio{title_suffix}", args.out_dir / f"{metric}_by_ratio.png")

    for metric in ["soc", "makespan", "runtime_ms", "assignment_time_ms", "assignment_calls"]:
        plot_full_incremental_scatter(rows, metric, args.out_dir / f"{metric}_full_vs_incremental_scatter.png")

    build_pair_summaries(rows, args.out_dir)

    print(f"wrote {args.out_dir}")
    for row in overall:
        print(
            f"{row['series_label']}: solved {row['solved']}/{row['cases']} "
            f"({row['solve_rate']:.3f}), mean runtime {safe_float(row.get('mean_runtime_ms')):.2f} ms"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
