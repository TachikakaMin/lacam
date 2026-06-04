#!/usr/bin/env python3
"""Plot arXiv 2307.00663-style comparison figures from local result files."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PAPER_DPI = 220
PAPER_FORMATS = ("png", "pdf")

MAP_ORDER = [
    "random-32-32-10",
    "empty-32-32",
    "den312d",
    "maze-32-32-2",
    "room-64-64-8",
    "warehouse-10-20-10-2-1",
    "orz900d",
    "Boston_0_256",
]

MAP_TITLES = {
    "random-32-32-10": "random-32-32-10",
    "empty-32-32": "empty-32-32",
    "den312d": "den312d",
    "maze-32-32-2": "maze-32-32-2",
    "room-64-64-8": "room",
    "warehouse-10-20-10-2-1": "warehouse",
    "orz900d": "orz900d",
    "Boston_0_256": "Boston",
}

DIR_MAPS = {
    "Paper_random_32_32_gp_5": ("random-32-32-10", "G"),
    "Paper_empty_32_32_gp_5": ("empty-32-32", "G"),
    "Paper_den312d_65_81_gp_5": ("den312d", "G"),
    "Paper_maze_32_32_2_gp_5": ("maze-32-32-2", "G"),
    "Paper_room_64_64_8_gp_5": ("room-64-64-8", "G"),
    "Paper_warehouse_161_63_gp_5": ("warehouse-10-20-10-2-1", "G"),
    "Paper_orz900d_gp_5": ("orz900d", "G"),
    "Paper_Boston_gp_5": ("Boston_0_256", "G"),
}

RATIO_PREFIXES = {
    "paper_random_32_32_ratio": "random-32-32-10",
    "paper_empty_32_32_ratio": "empty-32-32",
    "paper_den312d_65_81_ratio": "den312d",
    "paper_maze_32_32_2_ratio": "maze-32-32-2",
    "paper_room_64_64_8_ratio": "room-64-64-8",
    "paper_warehouse_161_63_ratio": "warehouse-10-20-10-2-1",
    "paper_orz900d_ratio": "orz900d",
    "paper_Boston_ratio": "Boston_0_256",
}

METHOD_LABELS = {
    "itacbs": "ITA-CBS",
    "ita_ecbs_v2": "ITA-ECBS",
    "lacam_focal_h": "LaCAM-TAPF",
    "lacam_dfs": "LaCAM-TAPF-DFS",
    "ir": "IR-TAPF",
    "opt_dbs_hungarian": "IR-TAPF+Opt",
    "opt_sbs_hungarian": "IR-SBS+Opt",
    "opt_random_hungarian": "IR-Random+Opt",
    "opt_dbs_pibt": "IR-DBS-PIBT+Opt",
    "opt_sbs_pibt": "IR-SBS-PIBT+Opt",
    "opt_random_pibt": "IR-Random-PIBT+Opt",
}

OPT_METHODS = [
    "opt_dbs_hungarian",
    "opt_sbs_hungarian",
    "opt_random_hungarian",
    "opt_dbs_pibt",
    "opt_sbs_pibt",
    "opt_random_pibt",
]

METHOD_ORDER = ["itacbs", "ita_ecbs_v2", "lacam_focal_h", "ir", "opt_dbs_hungarian"]
FIGURE4_METHOD_ORDER = ["itacbs", "lacam_focal_h", "lacam_dfs"]
METHOD_COLORS = {
    "itacbs": "#1f77b4",
    "ita_ecbs_v2": "#17becf",
    "lacam_focal_h": "#d62728",
    "lacam_dfs": "#ff9896",
    "ir": "#2ca02c",
    "opt_dbs_hungarian": "#9467bd",
    "opt_sbs_hungarian": "#c5b0d5",
    "opt_random_hungarian": "#8c564b",
    "opt_dbs_pibt": "#e377c2",
    "opt_sbs_pibt": "#bcbd22",
    "opt_random_pibt": "#7f7f7f",
}
METHOD_MARKERS = {
    "itacbs": "o",
    "ita_ecbs_v2": "s",
    "lacam_focal_h": "^",
    "lacam_dfs": "v",
    "ir": "D",
    "opt_dbs_hungarian": "P",
    "opt_sbs_hungarian": "X",
    "opt_random_hungarian": "*",
    "opt_dbs_pibt": "<",
    "opt_sbs_pibt": ">",
    "opt_random_pibt": "h",
}
SCENARIO_ORDER = ["G", "000", "030", "060", "100"]
SCENARIO_STYLES = {
    "G": "-",
    "000": "--",
    "030": "-.",
    "060": ":",
    "100": (0, (3, 1, 1, 1)),
}


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 8,
            "axes.labelsize": 9,
            "axes.titlesize": 9,
            "legend.fontsize": 7,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.45,
            "lines.linewidth": 1.25,
            "lines.markersize": 3.6,
        }
    )


def safe_float(value: Any) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def safe_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def solved_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value) > 0.5
    text = str(value).strip().lower()
    if text in {"true", "yes", "solved"}:
        return True
    try:
        return float(text) > 0.5
    except ValueError:
        return False


def normalize_map(name: str) -> str:
    name = name.replace(".map", "")
    if name in {"Boston", "Boston-0-256", "Boston_0_256"}:
        return "Boston_0_256"
    if name == "warehouse_161_63":
        return "warehouse-10-20-10-2-1"
    return name


def scenario_from_dir(dirname: str) -> tuple[str | None, str | None]:
    if dirname in DIR_MAPS:
        return DIR_MAPS[dirname]
    for prefix, map_name in RATIO_PREFIXES.items():
        if dirname.startswith(prefix + "_"):
            return map_name, dirname.rsplit("_", 1)[-1]
    return None, None


def parse_instance_path(path_value: Any) -> tuple[str | None, str | None, int | None, int | None]:
    if path_value is None or str(path_value) == "":
        return None, None, None, None
    path = Path(str(path_value))
    map_name, scenario = scenario_from_dir(path.parent.name)
    match = re.search(r"(.+)_agents_(\d+)_test_(\d+)(?:\.yaml)?$", path.stem)
    agents = None
    test = None
    file_map = None
    if match:
        file_map = normalize_map(match.group(1))
        agents = safe_int(match.group(2))
        test = safe_int(match.group(3))
    if map_name is None:
        map_name = file_map
    return map_name, scenario, agents, test


def parse_row_instance(row: pd.Series) -> tuple[str | None, str | None, int | None, int | None]:
    for key in ("fixture_file", "matrix_file", "case_id", "instance_file"):
        parsed = parse_instance_path(row.get(key))
        if parsed[0] and parsed[1] and parsed[2] is not None and parsed[3] is not None:
            return parsed
    return None, None, None, None


def runtime_seconds(row: pd.Series, *, timeout_s: float) -> float:
    runtime_ms = safe_float(row.get("runtime_ms"))
    if math.isfinite(runtime_ms):
        return min(timeout_s, runtime_ms / 1000.0)
    wall = safe_float(row.get("wall_time_s"))
    if math.isfinite(wall):
        return min(timeout_s, wall)
    return timeout_s


def first_solution_seconds(
    row: pd.Series,
    *,
    method: str,
    solved: bool,
    timeout_s: float,
) -> tuple[float, bool]:
    if method == "ir" or method in OPT_METHODS:
        raw_ms = safe_float(row.get("initial_solution_time_ms"))
    else:
        raw_ms = safe_float(row.get("first_solution_time_ms"))
    if not solved or not math.isfinite(raw_ms) or raw_ms < 0:
        return timeout_s, False
    return min(timeout_s, raw_ms / 1000.0), True


def read_row_table(path: Path) -> pd.DataFrame:
    if path.suffix == ".jsonl":
        records = []
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        return pd.DataFrame.from_records(records)
    return pd.read_csv(path)


def read_lacam_ir(path: Path, lacam_method: str, timeout_s: float) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame()
    raw = read_row_table(path)
    if raw.empty:
        return pd.DataFrame()
    raw = raw[raw["suite"].isin(["exp1", "exp2"])].copy()
    raw = raw[raw["method"].isin([lacam_method, "ir", *OPT_METHODS])].copy()
    records: list[dict[str, Any]] = []
    for _, row in raw.iterrows():
        map_name, scenario, agents, test = parse_row_instance(row)
        if not map_name or not scenario or agents is None or test is None:
            continue
        method = str(row["method"])
        cost = safe_float(row.get("soc"))
        solved = solved_bool(row.get("solved"))
        first_solution_s, first_solution_found = first_solution_seconds(
            row,
            method=method,
            solved=solved,
            timeout_s=timeout_s,
        )
        records.append(
            {
                "method": method,
                "method_label": METHOD_LABELS.get(method, method),
                "map": map_name,
                "scenario": scenario,
                "agents": agents,
                "test": test,
                "case_key": f"{map_name}|{scenario}|{agents}|{test}",
                "solved": solved,
                "runtime_s": runtime_seconds(row, timeout_s=timeout_s),
                "first_solution_s": first_solution_s,
                "first_solution_found": first_solution_found,
                "cost": cost if solved and math.isfinite(cost) else math.nan,
                "assignment_time_s": safe_float(row.get("assignment_time_ms")) / 1000.0,
                "assignment_calls": safe_float(row.get("assignment_calls")),
                "lowlevel_search_s": math.nan,
                "conflict_detection_s": math.nan,
                "node_count": safe_float(row.get("hl_nodes_explored")),
                "time_limit_s": timeout_s,
                "source": str(path),
            }
        )
    return pd.DataFrame.from_records(records)


def merge_lacam_ir_sources(
    paths: list[Path], lacam_method: str, timeout_s: float
) -> pd.DataFrame:
    """Load LaCAM/IR row files, letting later files override earlier duplicates."""
    frames: list[pd.DataFrame] = []
    for source_order, path in enumerate(paths):
        frame = read_lacam_ir(path, lacam_method, timeout_s)
        if frame.empty:
            continue
        frame["_source_order"] = source_order
        frames.append(frame)
    if not frames:
        return pd.DataFrame()

    merged = pd.concat(frames, ignore_index=True, sort=False)
    merged = merged.sort_values(["case_key", "method", "_source_order"])
    merged = merged.drop_duplicates(["case_key", "method"], keep="last")
    return merged.drop(columns=["_source_order"]).reset_index(drop=True)


def read_itacbs(path: Path, timeout_s: float) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame()
    raw = pd.read_csv(path)
    raw = raw[raw["solver"] == "itacbs"].copy()
    records: list[dict[str, Any]] = []
    for _, row in raw.iterrows():
        map_name, scenario, agents, test = parse_instance_path(row.get("instance_file"))
        if not map_name or not scenario or agents is None or test is None:
            continue
        solved = solved_bool(row.get("solved"))
        cost = safe_float(row.get("soc"))
        records.append(
            {
                "method": "itacbs",
                "method_label": METHOD_LABELS["itacbs"],
                "map": map_name,
                "scenario": scenario,
                "agents": agents,
                "test": test,
                "case_key": f"{map_name}|{scenario}|{agents}|{test}",
                "solved": solved,
                "runtime_s": runtime_seconds(row, timeout_s=timeout_s),
                "first_solution_s": math.nan,
                "first_solution_found": math.nan,
                "cost": cost if solved and math.isfinite(cost) else math.nan,
                "assignment_time_s": safe_float(row.get("itacbs_TA_runtime_ms")) / 1000.0,
                "assignment_calls": safe_float(row.get("itacbs_numTaskAssignments")),
                "lowlevel_search_s": safe_float(row.get("itacbs_lowlevel_search_time_ms")) / 1000.0,
                "conflict_detection_s": safe_float(row.get("itacbs_firstconflict_runtime_ms")) / 1000.0,
                "node_count": safe_float(row.get("itacbs_total_lowlevel_node")),
                "time_limit_s": timeout_s,
                "source": str(path),
            }
        )
    return pd.DataFrame.from_records(records)


def read_ecbs_success(path: Path, weight: str) -> pd.DataFrame:
    if not path.exists():
        return pd.DataFrame()
    raw = pd.read_csv(path, sep="\t")
    raw = raw[
        (raw["method"] == "ITA_ECBS_v2")
        & (raw["w"].astype(str) == str(weight))
    ].copy()
    if raw.empty:
        return pd.DataFrame()
    raw["map"] = raw["map"].map(normalize_map)
    raw["scenario"] = raw["common_portion"].astype(str).str.zfill(3)
    raw["agents"] = raw["agent_n"].astype(int)
    raw["success_rate"] = raw["succ_rate"].astype(float)
    raw["method"] = "ita_ecbs_v2"
    raw["method_label"] = METHOD_LABELS["ita_ecbs_v2"]
    raw["source"] = str(path)
    return raw[
        ["method", "method_label", "map", "scenario", "agents", "success_rate", "source"]
    ]


def make_success_rates(rows: pd.DataFrame, ecbs_success: pd.DataFrame) -> pd.DataFrame:
    row_rates = pd.DataFrame()
    if not rows.empty:
        grouped = (
            rows.groupby(["method", "method_label", "map", "scenario", "agents"], dropna=False)
            .agg(success_rate=("solved", "mean"), n_cases=("solved", "size"))
            .reset_index()
        )
        row_rates = grouped
    if ecbs_success.empty:
        return row_rates
    ecbs = ecbs_success.copy()
    ecbs["n_cases"] = np.nan
    return pd.concat([row_rates, ecbs], ignore_index=True, sort=False)


def save_figure(fig: Any, out_dir: Path, stem: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for suffix in PAPER_FORMATS:
        kwargs: dict[str, Any] = {"bbox_inches": "tight"}
        if suffix == "png":
            kwargs["dpi"] = PAPER_DPI
        fig.savefig(out_dir / f"{stem}.{suffix}", **kwargs)
    plt.close(fig)


def plot_figure2_subset(
    success: pd.DataFrame,
    out_dir: Path,
    *,
    stem: str,
    scenarios: list[str],
    title: str,
    max_agents: int | None = None,
) -> None:
    subset_all = success[success["scenario"].isin(scenarios)].copy()
    if max_agents is not None:
        subset_all = subset_all[subset_all["agents"] <= max_agents].copy()
    if subset_all.empty:
        return

    fig, axes = plt.subplots(2, 4, figsize=(15.5, 7.4), sharey=True)
    handles: dict[str, Any] = {}
    for ax, map_name in zip(axes.flat, MAP_ORDER):
        subset = subset_all[subset_all["map"] == map_name]
        for method in METHOD_ORDER:
            for scenario in scenarios:
                cur = subset[(subset["method"] == method) & (subset["scenario"] == scenario)]
                if cur.empty:
                    continue
                cur = cur.sort_values("agents")
                label = f"{METHOD_LABELS.get(method, method)} {scenario}_"
                line = ax.plot(
                    cur["agents"],
                    cur["success_rate"],
                    color=METHOD_COLORS.get(method),
                    linestyle=SCENARIO_STYLES.get(scenario, "-"),
                    marker=METHOD_MARKERS.get(method, "o"),
                    markersize=2.4,
                    linewidth=1.0,
                    alpha=0.9 if scenario in {"G", "100"} else 0.65,
                    label=label,
                )[0]
                handles.setdefault(label, line)
        ax.set_title(MAP_TITLES.get(map_name, map_name))
        ax.set_ylim(-0.03, 1.03)
        ax.set_xlim(left=0, right=max_agents * 1.03 if max_agents is not None else None)
        ax.grid(True, alpha=0.35)
        ax.set_xlabel("# agents")
    axes[0, 0].set_ylabel("success rate")
    axes[1, 0].set_ylabel("success rate")
    legend_items = sorted(handles.items(), key=lambda kv: (kv[0].split()[0], kv[0]))
    fig.legend(
        [h for _, h in legend_items],
        [label for label, _ in legend_items],
        loc="lower center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, -0.05),
    )
    fig.suptitle(title)
    save_figure(fig, out_dir, stem)


def plot_figure2(success: pd.DataFrame, out_dir: Path) -> None:
    plot_figure2_subset(
        success,
        out_dir,
        stem="figure2a_exp1_success_rates",
        scenarios=["G"],
        title="Figure 2a style: exp1 success rates on G fixtures (10s local rerun)",
    )
    plot_figure2_subset(
        success,
        out_dir,
        stem="figure2b_exp2_success_rates",
        scenarios=["000", "030", "060", "100"],
        title="Figure 2b style: exp2 success rates on ratio fixtures (10s local rerun)",
        max_agents=60,
    )


def capped_runtime_for_scatter(series: pd.Series, timeout_s: float) -> pd.Series:
    values = pd.to_numeric(series, errors="coerce")
    return values.clip(upper=timeout_s)


def capped_first_solution_for_scatter(series: pd.Series, timeout_s: float) -> pd.Series:
    values = pd.to_numeric(series, errors="coerce").fillna(timeout_s)
    return values.clip(lower=0.001, upper=timeout_s)


def plot_figure3(rows: pd.DataFrame, out_dir: Path, timeout_s: float) -> pd.DataFrame:
    pivot = rows.pivot_table(
        index="case_key",
        columns="method",
        values="runtime_s",
        aggfunc="min",
    )
    solved = rows.pivot_table(
        index="case_key",
        columns="method",
        values="solved",
        aggfunc="max",
    )
    comparisons = [
        m
        for m in ["lacam_focal_h", "ir", "opt_dbs_hungarian"]
        if "itacbs" in pivot and m in pivot
    ]
    if not comparisons:
        return pd.DataFrame()
    fig, axes = plt.subplots(1, len(comparisons), figsize=(5.2 * len(comparisons), 4.6), squeeze=False)
    scatter_rows: list[dict[str, Any]] = []
    x = capped_runtime_for_scatter(pivot["itacbs"], timeout_s)
    for ax, method in zip(axes.flat, comparisons):
        y = capped_runtime_for_scatter(pivot[method], timeout_s)
        common = x.notna() & y.notna()
        xs = x[common]
        ys = y[common]
        ax.scatter(xs, ys, s=8, alpha=0.35, color=METHOD_COLORS.get(method), edgecolors="none")
        ax.plot([0.001, timeout_s], [0.001, timeout_s], color="black", linestyle="--", linewidth=0.8)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlim(0.001, timeout_s * 1.08)
        ax.set_ylim(0.001, timeout_s * 1.08)
        ax.set_xlabel("ITA-CBS runtime (s)")
        ax.set_ylabel(f"{METHOD_LABELS.get(method, method)} runtime (s)")
        ax.set_title(f"ITA-CBS vs {METHOD_LABELS.get(method, method)}")
        ax.grid(True, which="both", alpha=0.35)
        faster = (ys < xs).mean() if len(xs) else math.nan
        both_solved = (
            solved.reindex(xs.index).get("itacbs", pd.Series(index=xs.index, dtype=float)).astype(float)
            * solved.reindex(xs.index).get(method, pd.Series(index=xs.index, dtype=float)).astype(float)
        ).sum()
        scatter_rows.append(
            {
                "comparison": f"itacbs_vs_{method}",
                "cases": int(len(xs)),
                "other_faster_fraction": faster,
                "both_solved_cases": int(both_solved),
            }
        )
    fig.suptitle("Figure 3 style: runtime scatter, timeouts capped at 10s")
    save_figure(fig, out_dir, "figure3_runtime_scatter")
    return pd.DataFrame.from_records(scatter_rows)


def plot_first_solution_scatter(
    rows: pd.DataFrame,
    out_dir: Path,
    timeout_s: float,
) -> pd.DataFrame:
    methods = {"lacam_focal_h", "ir"}
    subset = rows[rows["method"].isin(methods)].copy()
    if subset.empty or "first_solution_s" not in subset:
        return pd.DataFrame()
    pivot = subset.pivot_table(
        index="case_key",
        columns="method",
        values="first_solution_s",
        aggfunc="min",
    )
    found = subset.pivot_table(
        index="case_key",
        columns="method",
        values="first_solution_found",
        aggfunc="max",
    )
    if not methods.issubset(set(pivot.columns)):
        return pd.DataFrame()

    x = capped_first_solution_for_scatter(pivot["ir"], timeout_s)
    y = capped_first_solution_for_scatter(pivot["lacam_focal_h"], timeout_s)
    common = x.notna() & y.notna()
    xs = x[common]
    ys = y[common]
    if xs.empty:
        return pd.DataFrame()

    fig, ax = plt.subplots(figsize=(5.2, 4.6))
    ax.scatter(xs, ys, s=8, alpha=0.35, color=METHOD_COLORS["lacam_focal_h"], edgecolors="none")
    ax.plot([0.001, timeout_s], [0.001, timeout_s], color="black", linestyle="--", linewidth=0.8)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(0.001, timeout_s * 1.08)
    ax.set_ylim(0.001, timeout_s * 1.08)
    ax.set_xlabel("IR-TAPF time to first solution (s)")
    ax.set_ylabel("LaCAM-TAPF time to first solution (s)")
    ax.set_title("LaCAM-TAPF vs IR-TAPF first-solution time")
    ax.grid(True, which="both", alpha=0.35)
    fig.suptitle("Figure 3b style: first-solution scatter, failures capped at 10s")
    save_figure(fig, out_dir, "figure3b_first_solution_scatter")

    ir_found = found.reindex(xs.index).get("ir", pd.Series(index=xs.index, dtype=float)).astype(float)
    lacam_found = found.reindex(xs.index).get(
        "lacam_focal_h",
        pd.Series(index=xs.index, dtype=float),
    ).astype(float)
    summary = pd.DataFrame.from_records(
        [
            {
                "comparison": "ir_vs_lacam_focal_h_first_solution",
                "cases": int(len(xs)),
                "lacam_faster_fraction": float((ys < xs).mean()),
                "ir_faster_fraction": float((xs < ys).mean()),
                "tie_fraction": float((xs == ys).mean()),
                "both_found_cases": int((ir_found * lacam_found).sum()),
                "ir_timeout_or_missing_cases": int((ir_found < 0.5).sum()),
                "lacam_timeout_or_missing_cases": int((lacam_found < 0.5).sum()),
                "median_ir_first_solution_s": float(xs.median()),
                "median_lacam_first_solution_s": float(ys.median()),
            }
        ]
    )
    return summary


def profile_metrics(rows: pd.DataFrame) -> pd.DataFrame:
    rows = rows.copy()
    rows = rows[rows["solved"] == True].copy()
    rows["per_assignment_time_s"] = rows["assignment_time_s"] / rows["assignment_calls"]
    rows.loc[~np.isfinite(rows["per_assignment_time_s"]), "per_assignment_time_s"] = np.nan
    return rows


def plot_figure4(rows: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    prof = profile_metrics(rows)
    methods = [m for m in FIGURE4_METHOD_ORDER if m in set(prof["method"])]
    if not methods:
        return pd.DataFrame()
    fig, axes = plt.subplots(1, 3, figsize=(14.2, 4.3))
    metrics = [
        ("per_assignment_time_s", "mean TA time per call (s)", "log"),
        ("assignment_calls", "TA / assignment calls", "log"),
        ("node_count", "expanded high-level nodes", "log"),
    ]
    summary_records: list[dict[str, Any]] = []
    for ax, (field, ylabel, scale) in zip(axes, metrics):
        for method in methods:
            cur = prof[(prof["method"] == method) & np.isfinite(prof[field])]
            if cur.empty:
                continue
            agg = (
                cur.groupby("agents")[field]
                .median()
                .reset_index()
                .sort_values("agents")
            )
            ax.plot(
                agg["agents"],
                agg[field],
                marker=METHOD_MARKERS.get(method, "o"),
                color=METHOD_COLORS.get(method),
                label=METHOD_LABELS.get(method, method),
            )
            summary_records.append(
                {
                    "metric": field,
                    "method": method,
                    "median_over_cases": float(cur[field].median()),
                    "n_cases": int(cur[field].notna().sum()),
                }
            )
        if scale == "log":
            ax.set_yscale("log")
        ax.set_xlabel("# agents")
        ax.set_ylabel(ylabel)
        ax.grid(True, which="both", alpha=0.35)
    axes[0].legend(frameon=False)
    fig.suptitle("Figure 4 style: assignment runtime and node expansions")
    save_figure(fig, out_dir, "figure4_ta_runtime_nodes")
    return pd.DataFrame.from_records(summary_records)


def plot_figure5(rows: pd.DataFrame, out_dir: Path) -> pd.DataFrame:
    solved = rows[rows["solved"] == True].copy()
    if solved.empty:
        return pd.DataFrame()
    records: list[dict[str, Any]] = []
    for method, cur in solved.groupby("method"):
        runtime = pd.to_numeric(cur["runtime_s"], errors="coerce")
        assignment = pd.to_numeric(cur["assignment_time_s"], errors="coerce").fillna(0.0)
        lowlevel = pd.to_numeric(cur["lowlevel_search_s"], errors="coerce").fillna(0.0)
        conflict = pd.to_numeric(cur["conflict_detection_s"], errors="coerce").fillna(0.0)
        if method in {"lacam_focal_h", "lacam_dfs"}:
            search = (runtime - assignment).clip(lower=0.0)
            lowlevel = search
            conflict = conflict * 0.0
        elif method == "ir" or method in OPT_METHODS:
            assignment = assignment * 0.0
            lowlevel = runtime
            conflict = conflict * 0.0
        other = (runtime - assignment - lowlevel - conflict).clip(lower=0.0)
        records.append(
            {
                "method": method,
                "method_label": METHOD_LABELS.get(method, method),
                "Target assignment": float(assignment.mean()),
                "Search/refinement": float(lowlevel.mean()),
                "Conflict detection": float(conflict.mean()),
                "Other": float(other.mean()),
                "n_cases": int(len(cur)),
            }
        )
    breakdown = pd.DataFrame.from_records(records)
    breakdown = breakdown.set_index("method").reindex([m for m in METHOD_ORDER if m in set(breakdown["method"])])
    labels = breakdown["method_label"].tolist()
    components = ["Target assignment", "Search/refinement", "Conflict detection", "Other"]
    x = np.arange(len(labels))
    bottom = np.zeros(len(labels))
    fig, ax = plt.subplots(figsize=(7.8, 4.5))
    component_colors = {
        "Target assignment": "#1f77b4",
        "Search/refinement": "#ff7f0e",
        "Conflict detection": "#2ca02c",
        "Other": "#7f7f7f",
    }
    for component in components:
        vals = breakdown[component].to_numpy(dtype=float)
        ax.bar(x, vals, bottom=bottom, label=component, color=component_colors[component])
        bottom += vals
    ax.set_xticks(x, labels, rotation=18, ha="right")
    ax.set_ylabel("average runtime over solved cases (s)")
    ax.set_title("Figure 5 style: runtime breakdown")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend(frameon=False)
    save_figure(fig, out_dir, "figure5_runtime_breakdown")
    return breakdown.reset_index(drop=True)


def write_manifest(
    out_dir: Path,
    rows: pd.DataFrame,
    success: pd.DataFrame,
    args: argparse.Namespace,
    scatter_summary: pd.DataFrame,
    first_solution_summary: pd.DataFrame,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    coverage = (
        rows.groupby("method")
        .agg(
            rows=("method", "size"),
            solved=("solved", "sum"),
            maps=("map", lambda s: ",".join(sorted(set(s)))),
            scenarios=("scenario", lambda s: ",".join(sorted(set(s), key=lambda v: SCENARIO_ORDER.index(v) if v in SCENARIO_ORDER else 99))),
            min_agents=("agents", "min"),
            max_agents=("agents", "max"),
            median_runtime_s=("runtime_s", "median"),
            median_cost=("cost", "median"),
        )
        .reset_index()
    )
    coverage.to_csv(out_dir / "coverage_by_method.csv", index=False)
    success.to_csv(out_dir / "figure2_success_rates.csv", index=False)
    rows.to_csv(out_dir / "merged_case_rows.csv", index=False)
    if not scatter_summary.empty:
        scatter_summary.to_csv(out_dir / "figure3_scatter_summary.csv", index=False)
    if not first_solution_summary.empty:
        first_solution_summary.to_csv(
            out_dir / "figure3b_first_solution_summary.csv",
            index=False,
        )

    ecbs_coverage = pd.DataFrame()
    if not success.empty and "method" in success:
        ecbs_success = success[success["method"] == "ita_ecbs_v2"]
        if not ecbs_success.empty:
            ecbs_coverage = (
                ecbs_success.groupby("method")
                .agg(
                    success_points=("method", "size"),
                    maps=("map", lambda s: ",".join(sorted(set(s)))),
                    scenarios=(
                        "scenario",
                        lambda s: ",".join(
                            sorted(
                                set(s),
                                key=lambda v: SCENARIO_ORDER.index(v)
                                if v in SCENARIO_ORDER
                                else 99,
                            )
                        ),
                    ),
                    min_agents=("agents", "min"),
                    max_agents=("agents", "max"),
                )
                .reset_index()
            )
            ecbs_coverage.to_csv(out_dir / "ecbs_success_coverage.csv", index=False)

    opt_rows = rows[rows["method"].astype(str).str.startswith("opt_")]
    has_exp_opt = not opt_rows.empty
    lines = [
        "# arXiv 2307.00663 comparison figures",
        "",
        "Generated from local result files.",
        "",
        "## Inputs",
        f"- LaCAM/IR rows: `{args.lacam_ir_rows}`",
        f"- Extra row files: `{', '.join(str(p) for p in args.extra_rows) if args.extra_rows else ''}`",
        f"- ITA-CBS rows: `{args.itacbs_rows}`",
        f"- ITA-ECBS success rates: `{args.ecbs_success}`",
        f"- LaCAM method plotted as LaCAM-TAPF: `{args.lacam_method}`",
        f"- Timeout cap used in plots: `{args.timeout_s:g}s`",
        "",
        "## Output figures",
        "- `figure2a_exp1_success_rates.{png,pdf}`",
        "- `figure2b_exp2_success_rates.{png,pdf}`",
        "- `figure3_runtime_scatter.{png,pdf}`",
        "- `figure3b_first_solution_scatter.{png,pdf}`",
        "- `figure4_ta_runtime_nodes.{png,pdf}`",
        "- `figure5_runtime_breakdown.{png,pdf}`",
        "",
        "## Data caveats",
        "- The paper uses 30s optimal-solution limits; these local LaCAM/IR/ITA-CBS rows use a 10s rerun.",
        "- ITA-ECBS is included in Figure 2 from precomputed success-rate rows for `ITA_ECBS_v2`, weight 1.10; those rows are aggregate success rates, not per-case runtime/profile records.",
        "- If multiple LaCAM/IR row files contain the same `(case_key, method)`, later inputs override earlier rows. This is used to replace stale IR rows with the latest rerun without double-counting cases.",
        "- Complete ITA-ECBS row-level runtime/profile data on the same exp1/exp2 fixtures was not available in the normalized inputs, so Figures 3-5 omit ITA-ECBS.",
        "- Figure 3b compares LaCAM-TAPF `first_solution_time_ms` against IR-TAPF `initial_solution_time_ms`; missing or unsolved cases are plotted at the timeout cap.",
        "- Figure 4 includes only methods with comparable assignment/profile instrumentation. IR and IR+Opt rows are omitted there because the normalized exp1/exp2 rows do not expose comparable TA/node fields.",
        "- Figure 5 maps non-CBS methods to the closest available instrumentation. For LaCAM, target-assignment time is measured directly; the rest is search/refinement. For IR and IR+Opt, only total solver time is used in these exp1/exp2 rows.",
        f"- exp1/exp2 `opt_*` IR rows present in the plotted inputs: `{has_exp_opt}`.",
        "",
        "## Coverage",
        "",
        coverage.to_markdown(index=False),
    ]
    if not ecbs_coverage.empty:
        lines.extend(
            [
                "",
                "## ITA-ECBS Figure 2 Coverage",
                "",
                ecbs_coverage.to_markdown(index=False),
            ]
        )
    (out_dir / "MANIFEST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lacam-ir-rows",
        type=Path,
        default=Path("build/results/full_three_method_10s_rerun/rows.csv"),
    )
    parser.add_argument("--extra-rows", type=Path, action="append", default=[])
    parser.add_argument(
        "--itacbs-rows",
        type=Path,
        default=Path("build/results/full_10s_solver_parallel.csv"),
    )
    parser.add_argument(
        "--ecbs-success",
        type=Path,
        default=Path("/home/yimin/research/ITA-CBS2/plot_figure_ecbs/cvsdata.csv"),
    )
    parser.add_argument("--ecbs-weight", default="110")
    parser.add_argument("--lacam-method", default="lacam_focal_h")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("build/results/paper_2307_00663_figures"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_style()
    lacam_ir = merge_lacam_ir_sources(
        [args.lacam_ir_rows, *args.extra_rows],
        args.lacam_method,
        args.timeout_s,
    )
    itacbs = read_itacbs(args.itacbs_rows, args.timeout_s)
    rows = pd.concat([itacbs, lacam_ir], ignore_index=True, sort=False)
    if rows.empty:
        raise SystemExit("No row-level data loaded.")
    rows = rows[rows["map"].isin(MAP_ORDER)].copy()
    rows["agents"] = rows["agents"].astype(int)
    rows["test"] = rows["test"].astype(int)

    ecbs_success = read_ecbs_success(args.ecbs_success, args.ecbs_weight)
    success = make_success_rates(rows, ecbs_success)
    success = success[success["map"].isin(MAP_ORDER)].copy()
    success["agents"] = success["agents"].astype(int)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    plot_figure2(success, args.out_dir)
    scatter_summary = plot_figure3(rows, args.out_dir, args.timeout_s)
    first_solution_summary = plot_first_solution_scatter(rows, args.out_dir, args.timeout_s)
    fig4_summary = plot_figure4(rows, args.out_dir)
    fig5_breakdown = plot_figure5(rows, args.out_dir)
    if not fig4_summary.empty:
        fig4_summary.to_csv(args.out_dir / "figure4_profile_summary.csv", index=False)
    if not fig5_breakdown.empty:
        fig5_breakdown.to_csv(args.out_dir / "figure5_runtime_breakdown.csv", index=False)
    write_manifest(args.out_dir, rows, success, args, scatter_summary, first_solution_summary)
    print(f"Wrote figures and tables to {args.out_dir}")


if __name__ == "__main__":
    main()
