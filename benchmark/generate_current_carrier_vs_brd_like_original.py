#!/usr/bin/env python3
"""Render the current full-518 comparison with the original report UI.

The original generator lives in the archival dd-lacam workspace.  This adapter
keeps that generator and its plan-player implementation intact, while routing
instance lookups and result inputs to this workspace's current full benchmark.
"""

from __future__ import annotations

import argparse
import importlib.util
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
        page.write_text(text, encoding="utf-8")
    print(f"updated title labels to full-518 in {out_dir}")


if __name__ == "__main__":
    main()
