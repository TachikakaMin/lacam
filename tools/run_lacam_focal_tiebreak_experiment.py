#!/usr/bin/env python3
"""Compare LaCAM FOCAL tie-breaks on an existing IR-vs-LaCAM case set."""
from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import subprocess
import time
from pathlib import Path
from typing import Any


def parse_kv(text: str) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for line in text.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip()
        if value in {"0", "1"}:
            row[key] = int(value)
            continue
        try:
            row[key] = int(value)
            continue
        except ValueError:
            pass
        try:
            row[key] = float(value)
            continue
        except ValueError:
            pass
        row[key] = value
    return row


def safe_float(value: Any) -> float:
    try:
        if value == "" or value is None:
            return math.nan
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return sum(finite) / len(finite) if finite else math.nan


def variant_name(mode: str, tie_break: str, weight: float) -> str:
    if mode == "dfs":
        return "dfs"
    return f"focal_w{weight:g}_{tie_break}"


def read_cases(base_rows: Path, ir_solver: str) -> list[dict[str, Any]]:
    ir_name = f"ir_tapf:{ir_solver}"
    cases: dict[tuple[str, str], dict[str, Any]] = {}
    with base_rows.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (row["suite"], row["case_id"])
            case = cases.setdefault(
                key,
                {
                    "suite": row["suite"],
                    "case_id": row["case_id"],
                    "fixture_file": row["fixture_file"],
                    "matrix_file": row.get("matrix_file", ""),
                    "map_file": row.get("map_file", ""),
                    "num_agents": row.get("num_agents", ""),
                    "num_unique_tasks": row.get("num_unique_tasks", ""),
                    "ir_soc": "",
                    "ir_solved": 0,
                    "ir_wall_time_s": "",
                },
            )
            if row.get("solver") == ir_name:
                case["ir_soc"] = row.get("soc", "")
                case["ir_solved"] = int(row.get("solved") or 0)
                case["ir_wall_time_s"] = row.get("wall_time_s", "")
    return [cases[key] for key in sorted(cases)]


def run_variant(
    task: tuple[dict[str, Any], tuple[str, str, float]],
    lacam_bin: Path,
    time_limit: float,
    timeout: float,
    anytime: bool,
    full_ta: bool,
) -> dict[str, Any]:
    case, (mode, tie_break, weight) = task
    name = variant_name(mode, tie_break, weight)
    cmd = [
        str(lacam_bin),
        case["fixture_file"],
        "",
        str(time_limit),
        "",
        "1" if anytime else "0",
        "1" if full_ta else "0",
        "-1",
        mode,
        str(weight),
        tie_break,
    ]
    start = time.time()
    try:
        cp = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout)
        row = parse_kv(cp.stdout)
        row.update(
            {
                "exit_code": cp.returncode,
                "external_timed_out": 0,
                "stderr": cp.stderr.strip(),
            }
        )
    except subprocess.TimeoutExpired as exc:
        row = {
            "solved": 0,
            "valid_solution": 0,
            "timed_out": 1,
            "external_timed_out": 1,
            "exit_code": 124,
            "stderr": str(exc),
        }

    row.update(
        {
            **case,
            "solver": "lacam_tapf",
            "variant": name,
            "variant_mode": mode,
            "variant_tie_break": tie_break,
            "variant_focal_weight": weight,
            "wall_time_s": time.time() - start,
        }
    )
    row.setdefault("solved", 0)
    row.setdefault("valid_solution", 0)
    row.setdefault("timed_out", 0)
    return row


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def append_jsonl(path: Path, row: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def summarize(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in rows:
        groups.setdefault((row["suite"], row["variant"]), []).append(row)

    out = []
    for (suite, variant), group in sorted(groups.items()):
        solved = [r for r in group if int(r.get("solved") or 0)]
        both = [
            r
            for r in solved
            if int(r.get("ir_solved") or 0)
            and math.isfinite(safe_float(r.get("soc")))
            and math.isfinite(safe_float(r.get("ir_soc")))
        ]
        diffs = [safe_float(r.get("soc")) - safe_float(r.get("ir_soc")) for r in both]
        out.append(
            {
                "suite": suite,
                "variant": variant,
                "cases": len(group),
                "solved": len(solved),
                "solve_rate": len(solved) / len(group) if group else math.nan,
                "both_solved_with_ir": len(both),
                "mean_soc": mean([safe_float(r.get("soc")) for r in solved]),
                "mean_sum_of_loss": mean([safe_float(r.get("sum_of_loss")) for r in solved]),
                "mean_ir_soc": mean([safe_float(r.get("ir_soc")) for r in both]),
                "mean_soc_minus_ir": mean(diffs),
                "ir_better": sum(1 for d in diffs if d > 0),
                "lacam_better": sum(1 for d in diffs if d < 0),
                "ties": sum(1 for d in diffs if d == 0),
                "mean_runtime_ms": mean([safe_float(r.get("runtime_ms")) for r in solved]),
                "mean_wall_time_s": mean([safe_float(r.get("wall_time_s")) for r in group]),
                "external_timeouts": sum(int(r.get("external_timed_out") or 0) for r in group),
            }
        )
    return out


def plot_results(out_dir: Path, rows: list[dict[str, Any]], summary_rows: list[dict[str, Any]]) -> None:
    import matplotlib.pyplot as plt

    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    suites = sorted({r["suite"] for r in summary_rows})
    variants = sorted({r["variant"] for r in summary_rows})
    for metric, ylabel, filename in [
        ("mean_soc_minus_ir", "mean LaCAM SOC - IR SOC", "mean_soc_minus_ir.png"),
        ("solve_rate", "solve rate", "solve_rate.png"),
        ("mean_wall_time_s", "mean wall time (s)", "mean_wall_time_s.png"),
    ]:
        fig, axes = plt.subplots(1, len(suites), figsize=(6 * len(suites), 4), squeeze=False)
        for ax, suite in zip(axes[0], suites):
            vals = []
            labels = []
            for variant in variants:
                row = next((r for r in summary_rows if r["suite"] == suite and r["variant"] == variant), None)
                if row is None:
                    continue
                vals.append(safe_float(row.get(metric)))
                labels.append(variant)
            ax.bar(range(len(vals)), vals)
            ax.axhline(0, color="black", linewidth=0.8)
            ax.set_title(suite)
            ax.set_ylabel(ylabel)
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=35, ha="right")
        fig.tight_layout()
        fig.savefig(plot_dir / filename, dpi=180)
        plt.close(fig)

    for suite in suites:
        fig, ax = plt.subplots(figsize=(7, 5))
        for variant in variants:
            xs = []
            ys = []
            for row in rows:
                if row["suite"] != suite or row["variant"] != variant:
                    continue
                if not (int(row.get("solved") or 0) and int(row.get("ir_solved") or 0)):
                    continue
                x = safe_float(row.get("ir_soc"))
                y = safe_float(row.get("soc"))
                if math.isfinite(x) and math.isfinite(y):
                    xs.append(x)
                    ys.append(y)
            ax.scatter(xs, ys, s=14, alpha=0.55, label=variant)
        all_vals = [
            safe_float(v)
            for row in rows
            for v in (row.get("ir_soc"), row.get("soc"))
            if row["suite"] == suite
        ]
        finite = [v for v in all_vals if math.isfinite(v)]
        if finite:
            lo, hi = min(finite), max(finite)
            ax.plot([lo, hi], [lo, hi], color="black", linewidth=1, linestyle="--")
        ax.set_title(suite)
        ax.set_xlabel("IR SOC")
        ax.set_ylabel("LaCAM SOC")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(plot_dir / f"scatter_{suite}.png", dpi=180)
        plt.close(fig)


def parse_variants(values: list[str], weight: float) -> list[tuple[str, str, float]]:
    variants = []
    for value in values:
        if value == "dfs":
            variants.append(("dfs", "h", weight))
        elif value.startswith("focal:"):
            variants.append(("focal", value.split(":", 1)[1], weight))
        else:
            variants.append(("focal", value, weight))
    return variants


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-rows", type=Path, default=Path("build/results/ir_lacam_cross_100/rows.csv"))
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument("--ir-solver", default="dbs_hungarian")
    parser.add_argument("--lacam-bin", type=Path, default=Path("build/tapf_benchmark"))
    parser.add_argument("--time-limit", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--variant", action="append", default=None)
    parser.add_argument("--focal-weight", type=float, default=1.5)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=Path("build/results/lacam_focal_tiebreak_cross_100"))
    parser.set_defaults(anytime=True)
    parser.add_argument("--anytime", dest="anytime", action="store_true")
    parser.add_argument("--no-anytime", dest="anytime", action="store_false")
    parser.add_argument("--full-ta", action="store_true")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows_jsonl = args.out_dir / "rows.jsonl"
    rows_csv = args.out_dir / "rows.csv"
    summary_csv = args.out_dir / "summary.csv"

    cases = read_cases(args.base_rows, args.ir_solver)
    if args.max_cases > 0:
        cases = cases[: args.max_cases]
    variant_args = args.variant or [
        "dfs",
        "focal:h",
        "focal:anti_wait",
        "focal:anti_zigzag",
        "focal:anti_push",
        "focal:anti_all",
    ]
    variants = parse_variants(variant_args, args.focal_weight)
    completed: set[tuple[str, str, str]] = set()
    rows: list[dict[str, Any]] = []
    if args.resume and rows_jsonl.exists():
        for line in rows_jsonl.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            rows.append(row)
            completed.add((row["suite"], row["case_id"], row["variant"]))
    elif rows_jsonl.exists():
        rows_jsonl.unlink()

    tasks = []
    for case in cases:
        for variant in variants:
            name = variant_name(*variant)
            if (case["suite"], case["case_id"], name) not in completed:
                tasks.append((case, variant))

    print(f"cases={len(cases)} variants={len(variants)} tasks={len(tasks)} jobs={args.jobs}")
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = [
            executor.submit(
                run_variant,
                task,
                args.lacam_bin.resolve(),
                args.time_limit,
                args.timeout,
                args.anytime,
                args.full_ta,
            )
            for task in tasks
        ]
        for done, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            row = future.result()
            rows.append(row)
            append_jsonl(rows_jsonl, row)
            print(
                f"[{done}/{len(tasks)}] {row['suite']} {row['case_id']} "
                f"{row['variant']} solved={row.get('solved')} "
                f"soc={row.get('soc', '')} ir={row.get('ir_soc', '')} "
                f"wall={safe_float(row.get('wall_time_s')):.2f}s"
            )

    summary_rows = summarize(rows)
    write_csv(rows_csv, rows)
    write_csv(summary_csv, summary_rows)
    plot_results(args.out_dir, rows, summary_rows)
    print(f"wrote {rows_csv}")
    print(f"wrote {summary_csv}")
    print(f"wrote {args.out_dir / 'plots'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
