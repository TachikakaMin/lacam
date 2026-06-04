#!/usr/bin/env python3
"""Plot arXiv 2605.07744-style figures from paper experiment rows."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PAPER_DPI = 220
PAPER_FORMATS = ("png", "pdf")

METHOD_LABELS = {
    "dbs_hungarian": "DBS-Hungarian",
    "sbs_hungarian": "SBS-Hungarian",
    "random_hungarian": "Random-Hungarian",
    "dbs_pibt": "DBS-PIBT",
    "sbs_pibt": "SBS-PIBT",
    "random_pibt": "Random-PIBT",
    "lacam_dfs": "LaCAM-TAPF",
    "lacam_focal_h": "FOCAL-LaCAM-TAPF",
    "ita_ecbs": "ITA-ECBS",
    "dbs_hungarian_k1": "DBS-Hungarian k=1",
    "dbs_hungarian_k3": "DBS-Hungarian k=3",
    "dbs_hungarian_k10": "DBS-Hungarian k=10",
    "dbs_pibt_k1": "DBS-PIBT k=1",
    "dbs_pibt_k3": "DBS-PIBT k=3",
    "dbs_pibt_k10": "DBS-PIBT k=10",
    "opt_dbs_hungarian": "DBS-Hungarian+Opt",
    "opt_sbs_hungarian": "SBS-Hungarian+Opt",
    "opt_random_hungarian": "Random-Hungarian+Opt",
    "opt_dbs_pibt": "DBS-PIBT+Opt",
    "opt_sbs_pibt": "SBS-PIBT+Opt",
    "opt_random_pibt": "Random-PIBT+Opt",
    "opt_initial": "No Target Refinement",
}

PAPER_METHOD_ORDER = [
    "dbs_hungarian",
    "sbs_hungarian",
    "random_hungarian",
    "dbs_pibt",
    "sbs_pibt",
    "random_pibt",
    "lacam_dfs",
    "lacam_focal_h",
]

PAPER_METHOD_COLORS = {
    "dbs_hungarian": "#d81b7a",
    "sbs_hungarian": "#f2a1d3",
    "random_hungarian": "#ff31d7",
    "dbs_pibt": "#7fbf3f",
    "sbs_pibt": "#9bd463",
    "random_pibt": "#4f9a29",
    "lacam_dfs": "#1f77b4",
    "lacam_focal_h": "#111111",
    "dbs_hungarian_k1": "#f06292",
    "dbs_hungarian_k3": "#d81b7a",
    "dbs_hungarian_k10": "#ff31d7",
    "dbs_pibt_k1": "#7fbf3f",
    "dbs_pibt_k3": "#4f9a29",
    "dbs_pibt_k10": "#9bd463",
    "opt_dbs_hungarian": "#d81b7a",
    "opt_sbs_hungarian": "#f2a1d3",
    "opt_random_hungarian": "#ff31d7",
    "opt_dbs_pibt": "#7fbf3f",
    "opt_sbs_pibt": "#9bd463",
    "opt_random_pibt": "#4f9a29",
    "opt_initial": "#1f77b4",
}

MARKERS = {
    "dbs_hungarian": "o",
    "sbs_hungarian": "s",
    "random_hungarian": "^",
    "dbs_pibt": "D",
    "sbs_pibt": "v",
    "random_pibt": "P",
    "lacam_dfs": "X",
    "lacam_focal_h": "*",
    "dbs_hungarian_k1": "o",
    "dbs_hungarian_k3": "s",
    "dbs_hungarian_k10": "^",
    "dbs_pibt_k1": "o",
    "dbs_pibt_k3": "s",
    "dbs_pibt_k10": "^",
}


def configure_paper_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 9,
            "axes.labelsize": 10,
            "axes.titlesize": 10,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.5,
            "lines.linewidth": 1.5,
            "lines.markersize": 5,
        }
    )


def save_figure(fig: Any, out_dir: Path, stem: str, manifest: list[dict[str, str]], note: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for suffix in PAPER_FORMATS:
        kwargs = {"bbox_inches": "tight"}
        if suffix == "png":
            kwargs["dpi"] = PAPER_DPI
        fig.savefig(out_dir / f"{stem}.{suffix}", **kwargs)
    manifest.append({"figure": stem, "note": note})
    plt.close(fig)


def method_sort_key(method: str) -> tuple[int, str]:
    if method in PAPER_METHOD_ORDER:
        return PAPER_METHOD_ORDER.index(method), method
    return len(PAPER_METHOD_ORDER), method


def plot_method_line(ax: Any, xs: list[float], ys: list[float], method: str, *, linestyle: str = "-") -> None:
    ax.plot(
        xs,
        ys,
        marker=MARKERS.get(method, "o"),
        color=PAPER_METHOD_COLORS.get(method, None),
        linestyle=linestyle,
        label=METHOD_LABELS.get(method, method),
    )


def style_axis(ax: Any, *, grid_axis: str = "both") -> None:
    ax.grid(True, axis=grid_axis, alpha=0.35)
    ax.spines["top"].set_visible(True)
    ax.spines["right"].set_visible(True)


def safe_float(value: Any) -> float:
    try:
        if value == "" or value is None:
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_rows(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def normalized_cost(row: dict[str, Any]) -> float:
    soc = safe_float(row.get("soc"))
    lb = safe_float(row.get("sum_shortest_distances"))
    if not math.isfinite(lb) or lb <= 0:
        return math.nan
    return soc / lb


def improvement_pct(row: dict[str, Any]) -> float:
    initial = effective_initial_cost(row)
    soc = safe_float(row.get("soc"))
    if not math.isfinite(initial) or initial <= 0:
        return math.nan
    return 100.0 * (initial - soc) / initial


def effective_initial_cost(row: dict[str, Any]) -> float:
    initial = safe_float(row.get("initial_solution_cost"))
    if math.isfinite(initial) and initial > 0:
        return initial
    return safe_float(row.get("first_solution_cost"))


def group_rows(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[tuple[Any, ...], list[dict[str, Any]]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[tuple(row.get(k) for k in keys)].append(row)
    return groups


def finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def median_or_nan(values: list[float]) -> float:
    vals = finite(values)
    return median(vals) if vals else math.nan


def mean_or_nan(values: list[float]) -> float:
    vals = finite(values)
    return sum(vals) / len(vals) if vals else math.nan


def minmax(values: list[float]) -> tuple[float, float]:
    vals = finite(values)
    if not vals:
        return math.nan, math.nan
    return min(vals), max(vals)


def plot_fig3(rows: list[dict[str, Any]], out_dir: Path) -> None:
    fig3_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig3_") and int(r.get("solved") or 0)]
    if not fig3_rows:
        return
    out = out_dir / "fig3_components"
    out.mkdir(parents=True, exist_ok=True)

    by_map_mode = group_rows(fig3_rows, ("map_name", "target_mode"))
    for (map_name, target_mode), group in sorted(by_map_mode.items()):
        plt.figure(figsize=(8.5, 5.2))
        methods = sorted({r["method"] for r in group})
        for method in methods:
            pts = []
            for agents, rows_at_agents in group_rows([r for r in group if r["method"] == method], ("agents",)).items():
                vals = [normalized_cost(r) for r in rows_at_agents]
                med = median_or_nan(vals)
                lo, hi = minmax(vals)
                if math.isfinite(med):
                    pts.append((int(agents[0]), med, lo, hi))
            pts.sort()
            if not pts:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            los = [p[2] for p in pts]
            his = [p[3] for p in pts]
            plt.plot(xs, ys, marker=MARKERS.get(method, "o"), label=METHOD_LABELS.get(method, method))
            plt.fill_between(xs, los, his, alpha=0.12)
        plt.xlabel("agents")
        plt.ylabel("normalized flowtime")
        plt.title(f"Figure 3 style: {map_name} {target_mode.upper()}")
        plt.grid(True, alpha=0.25)
        plt.legend(fontsize=8, ncol=2)
        plt.tight_layout()
        plt.savefig(out / f"cost_{map_name}_{target_mode}.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.5, 5.2))
        for method in methods:
            pts = []
            for agents, rows_at_agents in group_rows([r for r in group if r["method"] == method], ("agents",)).items():
                vals = [improvement_pct(r) for r in rows_at_agents]
                med = median_or_nan(vals)
                if math.isfinite(med):
                    pts.append((int(agents[0]), med))
            pts.sort()
            if pts:
                plt.plot(
                    [p[0] for p in pts],
                    [p[1] for p in pts],
                    marker=MARKERS.get(method, "o"),
                    label=METHOD_LABELS.get(method, method),
                )
        plt.xlabel("agents")
        plt.ylabel("improvement from initial (%)")
        plt.title(f"Figure 3 style improvement: {map_name} {target_mode.upper()}")
        plt.grid(True, alpha=0.25)
        plt.legend(fontsize=8, ncol=2)
        plt.tight_layout()
        plt.savefig(out / f"improvement_{map_name}_{target_mode}.png", dpi=180)
        plt.close()


def plot_fig5(rows: list[dict[str, Any]], out_dir: Path) -> None:
    fig5_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig5_") and int(r.get("solved") or 0)]
    if not fig5_rows:
        return
    out = out_dir / "fig5_scalability"
    out.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(8.5, 5.2))
    for method, group in group_rows(fig5_rows, ("method",)).items():
        pts = []
        for agents, rows_at_agents in group_rows(group, ("agents",)).items():
            init_time = median_or_nan([safe_float(r.get("initial_solution_time_ms")) / 1000.0 for r in rows_at_agents])
            wall = median_or_nan([safe_float(r.get("wall_time_s")) for r in rows_at_agents])
            if math.isfinite(init_time):
                pts.append((int(agents[0]), init_time, wall))
        pts.sort()
        if pts:
            plt.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=f"{METHOD_LABELS.get(method, method)} initial")
            plt.plot([p[0] for p in pts], [p[2] for p in pts], marker="s", linestyle="--", label=f"{METHOD_LABELS.get(method, method)} wall")
    plt.xlabel("agents")
    plt.ylabel("runtime (s)")
    plt.title("Figure 5 style scalability")
    plt.grid(True, alpha=0.25)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(out / "runtime_scalability.png", dpi=180)
    plt.close()

    plt.figure(figsize=(8.5, 5.2))
    for method, group in group_rows(fig5_rows, ("method",)).items():
        pts = []
        for agents, rows_at_agents in group_rows(group, ("agents",)).items():
            imp = median_or_nan([improvement_pct(r) for r in rows_at_agents])
            if math.isfinite(imp):
                pts.append((int(agents[0]), imp))
        pts.sort()
        if pts:
            plt.plot([p[0] for p in pts], [p[1] for p in pts], marker=MARKERS.get(method, "o"), label=METHOD_LABELS.get(method, method))
    plt.xlabel("agents")
    plt.ylabel("improvement from initial (%)")
    plt.title("Figure 5 style improvement")
    plt.grid(True, alpha=0.25)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(out / "improvement_scalability.png", dpi=180)
    plt.close()


def plot_fig6(rows: list[dict[str, Any]], out_dir: Path) -> None:
    fig6_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig6_") and int(r.get("solved") or 0)]
    if not fig6_rows:
        return
    out = out_dir / "fig6_k_sweep"
    out.mkdir(parents=True, exist_ok=True)
    lacam_rows = [r for r in fig6_rows if str(r.get("method", "")).startswith("lacam_")]

    for family, title in (("dbs_pibt", "DBS-PIBT"), ("dbs_hungarian", "DBS-Hungarian")):
        family_rows = [r for r in fig6_rows if str(r.get("method", "")).startswith(f"{family}_k")]
        if not family_rows:
            continue

        plt.figure(figsize=(8.5, 5.2))
        for method, group in sorted(group_rows(family_rows + lacam_rows, ("method",)).items()):
            pts = []
            for agents, rows_at_agents in group_rows(group, ("agents",)).items():
                imp = median_or_nan([improvement_pct(r) for r in rows_at_agents])
                if math.isfinite(imp):
                    pts.append((int(agents[0]), imp))
            pts.sort()
            if pts:
                plt.plot(
                    [p[0] for p in pts],
                    [p[1] for p in pts],
                    marker=MARKERS.get(method[0], "o"),
                    label=METHOD_LABELS.get(method[0], method[0]),
                )
        plt.xlabel("agents")
        plt.ylabel("improvement from initial (%)")
        plt.title(f"Figure 6 style improvement: {title} with LaCAM baselines")
        plt.grid(True, alpha=0.25)
        plt.legend(fontsize=8)
        plt.tight_layout()
        plt.savefig(out / f"improvement_{family}.png", dpi=180)
        plt.close()

        plt.figure(figsize=(8.5, 5.2))
        for method, group in sorted(group_rows(family_rows, ("method",)).items()):
            pts = []
            for agents, rows_at_agents in group_rows(group, ("agents",)).items():
                iters = median_or_nan([safe_float(r.get("iterations_used")) for r in rows_at_agents])
                if math.isfinite(iters):
                    pts.append((int(agents[0]), iters))
            pts.sort()
            if pts:
                plt.plot(
                    [p[0] for p in pts],
                    [p[1] for p in pts],
                    marker=MARKERS.get(method[0], "o"),
                    label=METHOD_LABELS.get(method[0], method[0]),
                )
        plt.xlabel("agents")
        plt.ylabel("iterations")
        plt.title(f"Figure 6 style iterations: {title}")
        plt.grid(True, alpha=0.25)
        plt.legend(fontsize=8)
        plt.tight_layout()
        plt.savefig(out / f"iterations_{family}.png", dpi=180)
        plt.close()


def plot_fig7(rows: list[dict[str, Any]], out_dir: Path) -> None:
    fig_rows = [r for r in rows if int(r.get("solved") or 0) and r.get("iterations_time_ms") not in ("", None)]
    if not fig_rows:
        return
    out = out_dir / "fig7_profiling"
    out.mkdir(parents=True, exist_ok=True)

    methods = sorted({r["method"] for r in fig_rows if not r["method"].startswith("lacam_")})
    for method in methods:
        group = [r for r in fig_rows if r["method"] == method]
        pts = []
        for agents, rows_at_agents in group_rows(group, ("agents",)).items():
            path_ms = median_or_nan([safe_float(r.get("iterations_time_ms")) for r in rows_at_agents])
            reassign_ms = median_or_nan([safe_float(r.get("refinement_time_ms")) for r in rows_at_agents])
            if math.isfinite(path_ms) or math.isfinite(reassign_ms):
                pts.append((int(agents[0]), path_ms, reassign_ms))
        pts.sort()
        if not pts:
            continue
        xs = [p[0] for p in pts]
        path_vals = [0 if not math.isfinite(p[1]) else p[1] / 1000.0 for p in pts]
        reass_vals = [0 if not math.isfinite(p[2]) else p[2] / 1000.0 for p in pts]
        plt.figure(figsize=(8.5, 5.2))
        plt.bar(xs, path_vals, width=35, label="pathfinding/iterations")
        plt.bar(xs, reass_vals, width=35, bottom=path_vals, label="reassignment")
        plt.xlabel("agents")
        plt.ylabel("time (s)")
        plt.title(f"Figure 7 style profiling: {METHOD_LABELS.get(method, method)}")
        plt.grid(True, axis="y", alpha=0.25)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out / f"profile_{method}.png", dpi=180)
        plt.close()


def plot_table4(rows: list[dict[str, Any]], out_dir: Path) -> None:
    table_rows = [r for r in rows if str(r.get("scenario", "")).startswith("table4_")]
    if not table_rows:
        return
    out = out_dir / "table4_ita_ecbs"
    out.mkdir(parents=True, exist_ok=True)

    methods = ["ita_ecbs", "dbs_hungarian", "lacam_focal_h"]
    for scenario, group in sorted(group_rows(table_rows, ("scenario",)).items()):
        agents = sorted({int(r["agents"]) for r in group if str(r.get("agents", "")).isdigit()})
        if not agents:
            continue
        width = 0.25
        xs = list(range(len(agents)))

        plt.figure(figsize=(9, 5.2))
        for offset, method in enumerate(methods):
            vals = []
            for n in agents:
                rows_at = [r for r in group if r.get("method") == method and int(r.get("agents") or 0) == n]
                vals.append(mean_or_nan([float(int(r.get("solved") or 0)) for r in rows_at]) * 100.0 if rows_at else math.nan)
            plt.bar(
                [x + (offset - 1) * width for x in xs],
                vals,
                width=width,
                label=METHOD_LABELS.get(method, method),
            )
        plt.xticks(xs, agents)
        plt.xlabel("agents")
        plt.ylabel("success rate (%)")
        plt.ylim(0, 105)
        plt.title(f"Table 4 style success: {scenario[0].replace('table4_', '')}")
        plt.grid(True, axis="y", alpha=0.25)
        plt.legend(fontsize=8)
        plt.tight_layout()
        plt.savefig(out / f"success_{scenario[0]}.png", dpi=180)
        plt.close()

        plt.figure(figsize=(9, 5.2))
        for method in methods:
            pts = []
            for n in agents:
                rows_at = [
                    r
                    for r in group
                    if r.get("method") == method and int(r.get("agents") or 0) == n and int(r.get("solved") or 0)
                ]
                cost = median_or_nan([safe_float(r.get("soc")) for r in rows_at])
                if math.isfinite(cost):
                    pts.append((n, cost))
            if pts:
                plt.plot(
                    [p[0] for p in pts],
                    [p[1] for p in pts],
                    marker=MARKERS.get(method, "o"),
                    label=METHOD_LABELS.get(method, method),
                )
        plt.xlabel("agents")
        plt.ylabel("SOC")
        plt.title(f"Table 4 style cost: {scenario[0].replace('table4_', '')}")
        plt.grid(True, alpha=0.25)
        plt.legend(fontsize=8)
        plt.tight_layout()
        plt.savefig(out / f"cost_{scenario[0]}.png", dpi=180)
        plt.close()


def write_paper_tables(rows: list[dict[str, Any]], out_dir: Path) -> None:
    out = out_dir / "paper_tables"
    out.mkdir(parents=True, exist_ok=True)

    table_rows = []
    for prefix, table_name in (
        ("table1_", "table1_time_limited"),
        ("table2_", "table2_fixed_iterations"),
        ("table3_", "table3_random_initial"),
    ):
        selected = [r for r in rows if str(r.get("scenario", "")).startswith(prefix)]
        if not selected:
            continue
        for key, group in sorted(group_rows(selected, ("scenario", "method", "agents")).items()):
            solved = [r for r in group if int(r.get("solved") or 0)]
            table_rows.append(
                {
                    "table": table_name,
                    "scenario": key[0],
                    "method": key[1],
                    "agents": key[2],
                    "cases": len(group),
                    "solved": len(solved),
                    "solve_rate": len(solved) / len(group) if group else math.nan,
                    "mean_soc": mean_or_nan([safe_float(r.get("soc")) for r in solved]),
                    "median_soc": median_or_nan([safe_float(r.get("soc")) for r in solved]),
                    "mean_iterations_used": mean_or_nan([safe_float(r.get("iterations_used")) for r in solved]),
                    "mean_wall_time_s": mean_or_nan([safe_float(r.get("wall_time_s")) for r in group]),
                }
            )

    if table_rows:
        fields = [
            "table",
            "scenario",
            "method",
            "agents",
            "cases",
            "solved",
            "solve_rate",
            "mean_soc",
            "median_soc",
            "mean_iterations_used",
            "mean_wall_time_s",
        ]
        with (out / "tables_1_2_3.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(table_rows)

    table4_rows = [r for r in rows if str(r.get("scenario", "")).startswith("table4_")]
    if table4_rows:
        out_rows = []
        for key, group in sorted(group_rows(table4_rows, ("scenario", "method", "agents")).items()):
            solved = [r for r in group if int(r.get("solved") or 0)]
            out_rows.append(
                {
                    "scenario": key[0],
                    "method": key[1],
                    "agents": key[2],
                    "cases": len(group),
                    "solved": len(solved),
                    "solve_rate": len(solved) / len(group) if group else math.nan,
                    "mean_soc": mean_or_nan([safe_float(r.get("soc")) for r in solved]),
                    "median_soc": median_or_nan([safe_float(r.get("soc")) for r in solved]),
                    "external_timeouts": sum(int(r.get("external_timed_out") or 0) for r in group),
                    "mean_wall_time_s": mean_or_nan([safe_float(r.get("wall_time_s")) for r in group]),
                }
            )
        fields = [
            "scenario",
            "method",
            "agents",
            "cases",
            "solved",
            "solve_rate",
            "mean_soc",
            "median_soc",
            "external_timeouts",
            "mean_wall_time_s",
        ]
        with (out / "table4_ita_ecbs.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(out_rows)


def write_derived_csv(rows: list[dict[str, Any]], out_dir: Path) -> None:
    out_path = out_dir / "derived_metrics.csv"
    fields = [
        "scenario",
        "map_name",
        "target_mode",
        "method",
        "agents",
        "seed",
        "solved",
        "soc",
        "sum_shortest_distances",
        "normalized_cost",
        "initial_solution_cost",
        "first_solution_cost",
        "effective_initial_cost",
        "improvement_pct",
        "wall_time_s",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    **{k: row.get(k, "") for k in fields},
                    "normalized_cost": normalized_cost(row),
                    "effective_initial_cost": effective_initial_cost(row),
                    "improvement_pct": improvement_pct(row),
                }
            )


def plot_paper_figure3(rows: list[dict[str, Any]], out_dir: Path, manifest: list[dict[str, str]]) -> None:
    fig_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig3_") and int(r.get("solved") or 0)]
    if not fig_rows:
        return
    map_order = [
        "random-64-64-20",
        "warehouse-10-20-10-2-2",
        "ost003d",
        "lak303d",
        "den520d",
        "Boston_0_256",
    ]
    maps = [m for m in map_order if any(r.get("map_name") == m for r in fig_rows)]
    if not maps:
        maps = sorted({r.get("map_name", "") for r in fig_rows})
    maps = maps[:6]

    def build_grid(stem: str, methods_allowed: set[str], note: str, allow_log_y: bool) -> None:
        selected_rows = [r for r in fig_rows if r.get("method") in methods_allowed]
        if not selected_rows:
            return
        fig, axes = plt.subplots(4, 6, figsize=(13.2, 8.6))
        for ax in axes.flat:
            ax.set_visible(False)

        for idx, map_name in enumerate(maps):
            block = 0 if idx < 3 else 2
            col0 = (idx % 3) * 2
            for offset, target_mode in enumerate(("hotspot", "random")):
                col = col0 + offset
                group = [
                    r
                    for r in selected_rows
                    if r.get("map_name") == map_name and str(r.get("target_mode", "")).lower() == target_mode
                ]
                if not group:
                    continue

                ax_cost = axes[block][col]
                ax_cost.set_visible(True)
                ax_cost.set_title(target_mode.upper(), pad=2)
                cost_upper = 0.0
                cost_lower = math.inf
                for method in sorted({r["method"] for r in group}, key=method_sort_key):
                    pts = []
                    for agents, rows_at_agents in group_rows([r for r in group if r["method"] == method], ("agents",)).items():
                        vals = [normalized_cost(r) for r in rows_at_agents]
                        med = median_or_nan(vals)
                        lo, hi = minmax(vals)
                        if math.isfinite(med):
                            pts.append((int(agents[0]), med, lo, hi))
                    pts.sort()
                    if not pts:
                        continue
                    xs = [p[0] for p in pts]
                    ys = [p[1] for p in pts]
                    plot_method_line(ax_cost, xs, ys, method)
                    ax_cost.fill_between(xs, [p[2] for p in pts], [p[3] for p in pts], color=PAPER_METHOD_COLORS.get(method), alpha=0.10)
                    cost_upper = max(cost_upper, *[p[3] for p in pts if math.isfinite(p[3])])
                    cost_lower = min(cost_lower, *[p[2] for p in pts if math.isfinite(p[2])])
                if allow_log_y and math.isfinite(cost_lower) and cost_lower > 0 and cost_upper / cost_lower > 12:
                    ax_cost.set_yscale("log")
                    ax_cost.text(0.03, 0.92, "log y", transform=ax_cost.transAxes, fontsize=7, color="0.35")
                ax_cost.set_xlabel("agents")
                if col == 0:
                    ax_cost.set_ylabel("normalized\nflowtime")
                style_axis(ax_cost)

                ax_imp = axes[block + 1][col]
                ax_imp.set_visible(True)
                for method in sorted({r["method"] for r in group}, key=method_sort_key):
                    pts = []
                    for agents, rows_at_agents in group_rows([r for r in group if r["method"] == method], ("agents",)).items():
                        med = median_or_nan([improvement_pct(r) for r in rows_at_agents])
                        if math.isfinite(med):
                            pts.append((int(agents[0]), med))
                    pts.sort()
                    if pts:
                        plot_method_line(ax_imp, [p[0] for p in pts], [p[1] for p in pts], method)
                ax_imp.set_xlabel("agents")
                if col == 0:
                    ax_imp.set_ylabel("final\nimprv (%)")
                style_axis(ax_imp)

            axes[block][col0].text(
                0.0,
                1.18,
                map_name,
                transform=axes[block][col0].transAxes,
                fontsize=9,
                fontstyle="italic",
            )

        handles_by_label: dict[str, Any] = {}
        for ax in axes.flat:
            handles, labels = ax.get_legend_handles_labels()
            for handle, label in zip(handles, labels):
                handles_by_label.setdefault(label, handle)
        if handles_by_label:
            fig.legend(
                list(handles_by_label.values()),
                list(handles_by_label.keys()),
                loc="lower center",
                ncol=4,
                frameon=False,
            )
        fig.subplots_adjust(left=0.06, right=0.99, top=0.93, bottom=0.12, hspace=0.72, wspace=0.36)
        save_figure(fig, out_dir, stem, manifest, note)

    build_grid(
        "figure3_components_grid_with_lacam",
        set(PAPER_METHOD_ORDER),
        "Fig.3-style grid with LaCAM-TAPF baselines. Panels with large LaCAM outliers switch to log y for readability. Bottom panels use final improvement vs agent count because rows do not contain per-refinement cost histories.",
        allow_log_y=True,
    )


def plot_paper_figure4(rows: list[dict[str, Any]], out_dir: Path, manifest: list[dict[str, str]]) -> None:
    fig_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig4_") and int(r.get("solved") or 0)]
    if not fig_rows:
        return
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    method_order = [
        "opt_dbs_hungarian",
        "opt_sbs_hungarian",
        "opt_random_hungarian",
        "opt_dbs_pibt",
        "opt_sbs_pibt",
        "opt_random_pibt",
        "opt_initial",
        "lacam_dfs",
        "lacam_focal_h",
    ]
    for method in [m for m in method_order if any(r.get("method") == m for r in fig_rows)]:
        group = [r for r in fig_rows if r.get("method") == method]
        lb_vals = [safe_float(r.get("sum_shortest_distances")) for r in group]
        ref_vals = [
            safe_float(r.get("target_refinement_cost")) / safe_float(r.get("sum_shortest_distances"))
            for r in group
            if safe_float(r.get("target_refinement_cost")) > 0 and safe_float(r.get("sum_shortest_distances")) > 0
        ]
        final_vals = [normalized_cost(r) for r in group]
        pts = []
        if ref_vals:
            pts.append((20.0, median_or_nan(ref_vals)))
        first_vals = [
            safe_float(r.get("first_solution_cost")) / safe_float(r.get("sum_shortest_distances"))
            for r in group
            if safe_float(r.get("first_solution_cost")) > 0 and safe_float(r.get("sum_shortest_distances")) > 0
        ]
        if not ref_vals and first_vals:
            pts.append((safe_float(group[0].get("first_solution_time_ms")) / 1000.0, median_or_nan(first_vals)))
        final = median_or_nan(final_vals)
        if math.isfinite(final):
            pts.append((30.0, final))
        pts = [(x, y) for x, y in pts if math.isfinite(x) and math.isfinite(y)]
        pts.sort()
        if pts:
            linestyle = "--" if method.startswith("lacam_") else "-"
            plot_method_line(ax, [p[0] for p in pts], [p[1] for p in pts], method, linestyle=linestyle)
    ax.axvline(20, color="0.5", linewidth=0.8)
    ax.text(10, ax.get_ylim()[1], "Target refinement", ha="center", va="bottom")
    ax.text(25, ax.get_ylim()[1], "Path optimization", ha="center", va="bottom")
    ax.set_xlabel("runtime (s)")
    ax.set_ylabel("normalized flowtime")
    style_axis(ax)
    ax.legend(ncol=3, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.20))
    fig.subplots_adjust(bottom=0.30)
    save_figure(
        fig,
        out_dir,
        "figure4_final_path_optimization",
        manifest,
        "Fig.4-style two-stage view. Points use recorded target-refinement cost and final SOC; continuous refinement trajectories are not available in rows.",
    )


def plot_paper_figure5(rows: list[dict[str, Any]], out_dir: Path, manifest: list[dict[str, str]]) -> None:
    fig_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig5_") and int(r.get("solved") or 0)]
    if not fig_rows:
        return
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.1))
    for method in sorted({r["method"] for r in fig_rows}, key=method_sort_key):
        group = [r for r in fig_rows if r["method"] == method]
        runtime_pts = []
        improvement_pts = []
        for agents, rows_at_agents in group_rows(group, ("agents",)).items():
            initial_times = []
            for r in rows_at_agents:
                t = safe_float(r.get("initial_solution_time_ms"))
                if not math.isfinite(t):
                    t = safe_float(r.get("first_solution_time_ms"))
                if math.isfinite(t):
                    initial_times.append(t / 1000.0)
            init_time = median_or_nan(initial_times)
            imp = median_or_nan([improvement_pct(r) for r in rows_at_agents])
            n = int(agents[0])
            if math.isfinite(init_time):
                runtime_pts.append((n, init_time))
            if math.isfinite(imp):
                improvement_pts.append((n, imp))
        runtime_pts.sort()
        improvement_pts.sort()
        if runtime_pts:
            plot_method_line(axes[0], [p[0] for p in runtime_pts], [p[1] for p in runtime_pts], method)
        if improvement_pts:
            plot_method_line(axes[1], [p[0] for p in improvement_pts], [p[1] for p in improvement_pts], method)
    axes[0].set_xlabel("agents")
    axes[0].set_ylabel("initial solution time (s)")
    axes[0].set_yscale("log")
    axes[1].set_xlabel("agents")
    axes[1].set_ylabel("final imprv (%)")
    for ax in axes:
        style_axis(ax)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False)
    fig.subplots_adjust(bottom=0.26, wspace=0.32)
    save_figure(
        fig,
        out_dir,
        "figure5_scalability",
        manifest,
        "Fig.5-style scalability summary. Left is initial-solution runtime; right is final improvement because per-refinement improvement histories are not stored in rows.",
    )


def plot_paper_figure6(rows: list[dict[str, Any]], out_dir: Path, manifest: list[dict[str, str]]) -> None:
    fig_rows = [r for r in rows if str(r.get("scenario", "")).startswith("fig6_") and int(r.get("solved") or 0)]
    if not fig_rows:
        return
    fig, axes = plt.subplots(2, 2, figsize=(6.8, 5.0))
    families = [("dbs_pibt", "DBS-PIBT"), ("dbs_hungarian", "DBS-Hungarian")]
    lacam_rows = [r for r in fig_rows if str(r.get("method", "")).startswith("lacam_")]
    for col, (family, title) in enumerate(families):
        family_rows = [r for r in fig_rows if str(r.get("method", "")).startswith(f"{family}_k")]
        for method in sorted({r["method"] for r in family_rows + lacam_rows}, key=method_sort_key):
            group = [r for r in family_rows + lacam_rows if r["method"] == method]
            pts = []
            for agents, rows_at_agents in group_rows(group, ("agents",)).items():
                imp = median_or_nan([improvement_pct(r) for r in rows_at_agents])
                if math.isfinite(imp):
                    pts.append((int(agents[0]), imp))
            pts.sort()
            if pts:
                plot_method_line(axes[0][col], [p[0] for p in pts], [p[1] for p in pts], method)
        for method in sorted({r["method"] for r in family_rows}, key=method_sort_key):
            group = [r for r in family_rows if r["method"] == method]
            pts = []
            for agents, rows_at_agents in group_rows(group, ("agents",)).items():
                iters = median_or_nan([safe_float(r.get("iterations_used")) for r in rows_at_agents])
                if math.isfinite(iters):
                    pts.append((int(agents[0]), iters))
            pts.sort()
            if pts:
                plot_method_line(axes[1][col], [p[0] for p in pts], [p[1] for p in pts], method)
        axes[0][col].set_title(title)
        axes[0][col].set_ylabel("final imprv (%)")
        axes[1][col].set_ylabel("iterations")
        axes[1][col].set_xlabel("agents")
        style_axis(axes[0][col])
        style_axis(axes[1][col])
    handles_by_label: dict[str, Any] = {}
    for ax in axes.flat:
        handles, labels = ax.get_legend_handles_labels()
        for handle, label in zip(handles, labels):
            handles_by_label.setdefault(label, handle)
    fig.legend(
        list(handles_by_label.values()),
        list(handles_by_label.keys()),
        loc="lower center",
        ncol=3,
        frameon=False,
    )
    fig.subplots_adjust(bottom=0.20, hspace=0.38, wspace=0.30)
    save_figure(
        fig,
        out_dir,
        "figure6_multibottleneck",
        manifest,
        "Fig.6-style k sweep. LaCAM-TAPF baselines are shown on improvement panels; iteration panels include only IR k-sweep methods.",
    )


def write_manifest(out_dir: Path, rows_paths: list[Path], manifest: list[dict[str, str]]) -> None:
    lines = [
        "# Paper Figure Manifest",
        "",
        "Input rows:",
        *[f"- {path}" for path in rows_paths],
        "",
        "Generated figures:",
    ]
    for item in manifest:
        lines.append(f"- `{item['figure']}.png` / `{item['figure']}.pdf`: {item['note']}")
    lines.extend(
        [
            "",
            "Notes:",
            "- Current experiment rows do not contain per-refinement cost histories, so figures that correspond to paper improvement-over-time plots use recorded final improvement instead of synthetic time curves.",
            "- All plotted LaCAM-TAPF rows come from the same-instance paper-suite reruns.",
        ]
    )
    (out_dir / "MANIFEST.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_paper_summary(rows: list[dict[str, Any]], rows_paths: list[Path], out_dir: Path) -> None:
    manifest: list[dict[str, str]] = []
    plot_paper_figure3(rows, out_dir, manifest)
    plot_paper_figure4(rows, out_dir, manifest)
    plot_paper_figure5(rows, out_dir, manifest)
    plot_paper_figure6(rows, out_dir, manifest)
    plot_table4(rows, out_dir)
    write_paper_tables(rows, out_dir)
    write_manifest(out_dir, rows_paths, manifest)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=Path, nargs="+", default=[Path("build/results/paper_2605_07744/rows.csv")])
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/paper_2605_07744/plots"))
    parser.add_argument("--paper-summary", action="store_true", help="write paper-numbered figures into one folder")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    configure_paper_style()

    rows = []
    for path in args.rows:
        rows.extend(read_rows(path))
    write_derived_csv(rows, args.out_dir)
    if args.paper_summary:
        plot_paper_summary(rows, args.rows, args.out_dir)
        print(f"wrote paper summary figures under {args.out_dir}")
        return 0

    plot_fig3(rows, args.out_dir)
    plot_fig5(rows, args.out_dir)
    plot_fig6(rows, args.out_dir)
    plot_fig7(rows, args.out_dir)
    plot_table4(rows, args.out_dir)
    write_paper_tables(rows, args.out_dir)
    print(f"wrote plots under {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
