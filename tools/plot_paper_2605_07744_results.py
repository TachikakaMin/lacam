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
    initial = safe_float(row.get("initial_solution_cost"))
    soc = safe_float(row.get("soc"))
    if not math.isfinite(initial) or initial <= 0:
        return math.nan
    return 100.0 * (initial - soc) / initial


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

    for family, title in (("dbs_pibt", "DBS-PIBT"), ("dbs_hungarian", "DBS-Hungarian")):
        family_rows = [r for r in fig6_rows if str(r.get("method", "")).startswith(f"{family}_k")]
        if not family_rows:
            continue

        plt.figure(figsize=(8.5, 5.2))
        for method, group in sorted(group_rows(family_rows, ("method",)).items()):
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
        plt.title(f"Figure 6 style improvement: {title}")
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
                    "improvement_pct": improvement_pct(row),
                }
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=Path, default=Path("build/results/paper_2605_07744/rows.csv"))
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/paper_2605_07744/plots"))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(args.rows)
    write_derived_csv(rows, args.out_dir)
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
