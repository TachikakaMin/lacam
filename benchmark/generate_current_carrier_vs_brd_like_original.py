#!/usr/bin/env python3
"""Render the current full-518 comparison with the original report UI.

The original generator lives in the archival dd-lacam workspace.  This adapter
keeps that generator and its plan-player implementation intact, while routing
instance lookups and result inputs to this workspace's current full benchmark.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
ARCHIVE_GENERATOR = (
    HERE.parent.parent / "dd-lacam" /
    "benchmark/generate_carrier_vs_brd_first_solution_page.py"
)


def load_original():
    spec = importlib.util.spec_from_file_location(
        "archived_carrier_brd_report", ARCHIVE_GENERATOR
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load original report generator: {ARCHIVE_GENERATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def common_first_solution_geometric_ratio(
    carrier_run: Path, brd_run: Path
) -> float:
    def successful_times(run: Path) -> dict[str, float]:
        with (run / "rows.csv").open(
            encoding="utf-8", newline=""
        ) as source:
            return {
                row["instance"]: float(row["first_solution_ms"])
                for row in csv.DictReader(source)
                if row.get("success") == "1"
                and row.get("first_solution_ms") not in ("", None)
                and float(row["first_solution_ms"]) > 0
            }

    carrier = successful_times(carrier_run)
    baseline = successful_times(brd_run)
    common = sorted(carrier.keys() & baseline.keys())
    if not common:
        raise ValueError("no common positive solve times for report ratio")
    ratios = [carrier[name] / baseline[name] for name in common]
    return math.exp(sum(math.log(value) for value in ratios) / len(ratios))


def update_comparison_semantics(out_dir: Path, time_ratio: float) -> None:
    replacements = {
        "index.html": (
            "对角线下方 = carrier 数值更小。",
            "对角线上方 = carrier 数值更小。<br>"
            f"<strong>首解时间几何比 Carrier / baseline = "
            f"{time_ratio:.3f}，小于 1 表示 carrier 更快。</strong>",
        ),
        "index_en.html": (
            "Below the diagonal = a lower Carrier value.",
            "Above the diagonal = a lower Carrier value.<br>"
            f"<strong>First-solution geometric ratio "
            f"Carrier / baseline = {time_ratio:.3f}; values below 1 mean "
            f"Carrier is faster.</strong>",
        ),
    }
    for name, (old, new) in replacements.items():
        page = out_dir / name
        text = page.read_text(encoding="utf-8")
        if old not in text:
            raise ValueError(f"missing comparison-direction text in {page}")
        page.write_text(text.replace(old, new), encoding="utf-8")


def main() -> None:
    if not ARCHIVE_GENERATOR.is_file():
        raise FileNotFoundError(f"original generator not found: {ARCHIVE_GENERATOR}")

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--carrier-run",
        type=Path,
        default=HERE / "results_full_carrier_baseline_complete_replan_20260909",
    )
    parser.add_argument(
        "--brd-run",
        type=Path,
        default=HERE / "results_full_carrier_brd_complete_replan_20260909",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=HERE / "viz_web/carrier_vs_brd_first_solution_full518_20260909",
    )
    args = parser.parse_args()

    with (args.brd_run / "rows.csv").open(
        encoding="utf-8", newline=""
    ) as source:
        for row in csv.DictReader(source):
            if row.get("success") != "1":
                continue
            if row.get("first_solution_ms") != row.get("deliverable_ms"):
                raise ValueError(
                    "baseline is expected to expose only final time, but "
                    f"{row['instance']} has first_solution_ms="
                    f"{row.get('first_solution_ms')} and deliverable_ms="
                    f"{row.get('deliverable_ms')}"
                )

    original = load_original()
    original.REPO = HERE
    original.INSTANCE_ROOTS = {
        "warehouse_blocks": HERE / "viz_web/warehouse_block_suite/instances",
        "warehouse_random_factorial":
            HERE / "viz_web/warehouse_case_proposal/factorial_suite/instances",
        "dense_channel_block_edge_40x40":
            HERE / "viz_web/dense_channel_block_edge_40x40_suite_v1_20260907/instances",
    }
    original.CARRIER_RUN = args.carrier_run.resolve()
    original.BRD_RUN = args.brd_run.resolve()

    out_dir = args.out_dir.resolve()
    sys.argv = [
        str(ARCHIVE_GENERATOR),
        "--carrier-run", str(original.CARRIER_RUN),
        "--brd-run", str(original.BRD_RUN),
        "--out-dir", str(out_dir),
    ]
    original.main()

    # The archived template's only dataset-specific wording is the report
    # size.  Preserve all its interactions and translations, while correcting
    # that label for the current manifest.
    for page in (out_dir / "index.html", out_dir / "index_en.html"):
        text = page.read_text(encoding="utf-8")
        text = text.replace("full 509", "full 518").replace("full-509", "full-518")
        if page.name == "index.html":
            text = text.replace(
                "首解时间与解质量",
                "我们首解 vs baseline final 与解质量",
            )
            text = text.replace(
                "首解时间（ms，log-log）",
                "求解时间（我们首解 / baseline final，ms，log-log）",
            )
            text = text.replace(
                "baseline 首解时间 (ms)",
                "baseline final 时间 (ms)",
            )
            text = text.replace(
                "`首解 ${s.first} ms，mk ${s.mk}，soc ${s.soc}`",
                "`时间 ${s.first} ms，mk ${s.mk}，soc ${s.soc}`",
            )
            text = text.replace(
                "`解出：首解 ${s.first} ms ｜ makespan ${s.mk} ｜ soc ${s.soc}`",
                "`解出：时间 ${s.first} ms ｜ makespan ${s.mk} ｜ soc ${s.soc}`",
            )
        else:
            text = text.replace(
                "first-solution time and solution quality",
                "our first solution vs baseline final and solution quality",
            )
            text = text.replace(
                "First-solution time (ms, log-log)",
                "Solve time (our first solution / baseline final, ms, log-log)",
            )
            text = text.replace(
                "Baseline first-solution time (ms)",
                "Baseline final time (ms)",
            )
            text = text.replace(
                "`first ${s.first} ms, mk ${s.mk}, soc ${s.soc}`",
                "`time ${s.first} ms, mk ${s.mk}, soc ${s.soc}`",
            )
            text = text.replace(
                "`Solved: first solution ${s.first} ms ｜ makespan ${s.mk} ｜ SOC ${s.soc}`",
                "`Solved: time ${s.first} ms ｜ makespan ${s.mk} ｜ SOC ${s.soc}`",
            )
        page.write_text(text, encoding="utf-8")
    update_comparison_semantics(
        out_dir,
        common_first_solution_geometric_ratio(
            args.carrier_run, args.brd_run
        ),
    )
    print(f"updated title labels to full-518 in {out_dir}")


if __name__ == "__main__":
    main()
