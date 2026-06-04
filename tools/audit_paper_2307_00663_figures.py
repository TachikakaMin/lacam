#!/usr/bin/env python3
"""Audit arXiv 2307.00663-style figure outputs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd
from PIL import Image


EXPECTED_MAPS = {
    "Boston_0_256",
    "den312d",
    "empty-32-32",
    "maze-32-32-2",
    "orz900d",
    "random-32-32-10",
    "room-64-64-8",
    "warehouse-10-20-10-2-1",
}
EXPECTED_SCENARIOS = {"G", "000", "030", "060", "100"}
EXPECTED_ECBS_SCENARIOS = {"000", "030", "060", "100"}
FIGURE_STEMS = [
    "figure2a_exp1_success_rates",
    "figure2b_exp2_success_rates",
    "figure3_runtime_scatter",
    "figure4_ta_runtime_nodes",
    "figure5_runtime_breakdown",
]


def split_set(value: object) -> set[str]:
    if pd.isna(value):
        return set()
    return {part for part in str(value).split(",") if part}


def fail(errors: list[str], message: str) -> None:
    errors.append(message)
    print(f"FAIL {message}")


def ok(message: str) -> None:
    print(f"OK   {message}")


def audit_images(out_dir: Path, errors: list[str]) -> None:
    for stem in FIGURE_STEMS:
        for suffix in ("png", "pdf"):
            path = out_dir / f"{stem}.{suffix}"
            if not path.exists() or path.stat().st_size <= 0:
                fail(errors, f"missing or empty {path}")
                continue
            if suffix == "png":
                try:
                    with Image.open(path) as img:
                        img.verify()
                    with Image.open(path) as img:
                        width, height = img.size
                    if width <= 100 or height <= 100:
                        fail(errors, f"{path} has suspicious dimensions {width}x{height}")
                    else:
                        ok(f"{path.name} readable {width}x{height}")
                except Exception as exc:  # pragma: no cover - diagnostic path
                    fail(errors, f"{path} is not a readable PNG: {exc}")
            else:
                ok(f"{path.name} present")


def audit_row_coverage(
    out_dir: Path,
    require_opt: bool,
    expected_row_cases: int,
    errors: list[str],
) -> None:
    path = out_dir / "coverage_by_method.csv"
    if not path.exists():
        fail(errors, f"missing {path}")
        return
    coverage = pd.read_csv(path)
    expected = ["itacbs", "lacam_focal_h", "ir"]
    if require_opt:
        expected.append("opt_dbs_hungarian")

    for method in expected:
        cur = coverage[coverage["method"] == method]
        if cur.empty:
            fail(errors, f"coverage missing method {method}")
            continue
        row = cur.iloc[0]
        rows = int(row["rows"])
        if rows != expected_row_cases:
            fail(errors, f"{method} rows={rows}, expected {expected_row_cases}")
        else:
            ok(f"{method} rows={expected_row_cases}")
        maps = split_set(row["maps"])
        scenarios = split_set(row["scenarios"])
        if maps != EXPECTED_MAPS:
            fail(errors, f"{method} maps={sorted(maps)}, expected {sorted(EXPECTED_MAPS)}")
        else:
            ok(f"{method} covers all 8 maps")
        if scenarios != EXPECTED_SCENARIOS:
            fail(
                errors,
                f"{method} scenarios={sorted(scenarios)}, expected {sorted(EXPECTED_SCENARIOS)}",
            )
        else:
            ok(f"{method} covers G/000/030/060/100")
        if int(row["min_agents"]) != 5 or int(row["max_agents"]) != 200:
            fail(
                errors,
                f"{method} agent range={row['min_agents']}..{row['max_agents']}, expected 5..200",
            )
        else:
            ok(f"{method} agent range 5..200")


def audit_ecbs(out_dir: Path, errors: list[str]) -> None:
    path = out_dir / "ecbs_success_coverage.csv"
    if not path.exists():
        fail(errors, f"missing {path}")
        return
    coverage = pd.read_csv(path)
    cur = coverage[coverage["method"] == "ita_ecbs_v2"]
    if cur.empty:
        fail(errors, "ITA-ECBS success coverage missing")
        return
    row = cur.iloc[0]
    if int(row["success_points"]) != 480:
        fail(errors, f"ITA-ECBS success_points={row['success_points']}, expected 480")
    else:
        ok("ITA-ECBS has 480 Figure 2 success points")
    maps = split_set(row["maps"])
    scenarios = split_set(row["scenarios"])
    if maps != EXPECTED_MAPS:
        fail(errors, f"ITA-ECBS maps={sorted(maps)}, expected {sorted(EXPECTED_MAPS)}")
    else:
        ok("ITA-ECBS covers all 8 maps in Figure 2")
    if scenarios != EXPECTED_ECBS_SCENARIOS:
        fail(
            errors,
            f"ITA-ECBS scenarios={sorted(scenarios)}, expected {sorted(EXPECTED_ECBS_SCENARIOS)}",
        )
    else:
        ok("ITA-ECBS covers 000/030/060/100 in Figure 2")
    if int(row["min_agents"]) != 10 or int(row["max_agents"]) != 150:
        fail(
            errors,
            f"ITA-ECBS agent range={row['min_agents']}..{row['max_agents']}, expected 10..150",
        )
    else:
        ok("ITA-ECBS agent range 10..150 in Figure 2")


def audit_figure2_maze_exp2(out_dir: Path, errors: list[str]) -> None:
    path = out_dir / "figure2_success_rates.csv"
    if not path.exists() or path.stat().st_size <= 0:
        return
    success = pd.read_csv(path)
    expected_agents = set(range(5, 65, 5))
    expected_scenarios = {"000", "030", "060", "100"}
    for method in ("itacbs", "lacam_focal_h", "ir"):
        cur = success[
            (success["method"] == method)
            & (success["map"] == "maze-32-32-2")
            & (success["scenario"].isin(expected_scenarios))
        ].copy()
        found_scenarios = set(cur["scenario"].astype(str))
        if found_scenarios != expected_scenarios:
            fail(
                errors,
                f"{method} maze exp2 scenarios={sorted(found_scenarios)}, "
                f"expected {sorted(expected_scenarios)}",
            )
            continue
        bad = []
        for scenario, group in cur.groupby("scenario"):
            agents = set(group["agents"].astype(int))
            if agents != expected_agents:
                bad.append(f"{scenario}:{sorted(agents)}")
        if bad:
            fail(
                errors,
                f"{method} maze exp2 agents mismatch; expected 5..60 step 5, found {bad}",
            )
        else:
            ok(f"{method} maze exp2 Figure 2 coverage is 5..60 for all ratios")


def audit_tables(
    out_dir: Path,
    require_opt: bool,
    expected_row_cases: int,
    errors: list[str],
) -> None:
    required = [
        "figure2_success_rates.csv",
        "figure3_scatter_summary.csv",
        "figure4_profile_summary.csv",
        "figure5_runtime_breakdown.csv",
        "merged_case_rows.csv",
        "MANIFEST.md",
    ]
    for name in required:
        path = out_dir / name
        if not path.exists() or path.stat().st_size <= 0:
            fail(errors, f"missing or empty {path}")
        else:
            ok(f"{name} present")

    scatter_path = out_dir / "figure3_scatter_summary.csv"
    if scatter_path.exists() and scatter_path.stat().st_size > 0:
        scatter = pd.read_csv(scatter_path)
        expected = {"itacbs_vs_lacam_focal_h", "itacbs_vs_ir"}
        if require_opt:
            expected.add("itacbs_vs_opt_dbs_hungarian")
        found = set(scatter["comparison"].astype(str))
        for comparison in expected:
            if comparison not in found:
                fail(errors, f"Figure 3 summary missing {comparison}")
            else:
                cases = int(scatter.loc[scatter["comparison"] == comparison, "cases"].iloc[0])
                if cases != expected_row_cases:
                    fail(errors, f"{comparison} cases={cases}, expected {expected_row_cases}")
                else:
                    ok(f"{comparison} cases={expected_row_cases}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--require-opt", action="store_true")
    parser.add_argument("--expected-row-cases", type=int, default=9760)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    errors: list[str] = []
    if not args.out_dir.exists():
        fail(errors, f"missing output directory {args.out_dir}")
        return 1

    audit_images(args.out_dir, errors)
    audit_row_coverage(args.out_dir, args.require_opt, args.expected_row_cases, errors)
    audit_ecbs(args.out_dir, errors)
    audit_figure2_maze_exp2(args.out_dir, errors)
    audit_tables(args.out_dir, args.require_opt, args.expected_row_cases, errors)

    if errors:
        print(f"\nAudit failed: {len(errors)} issue(s)")
        return 1
    print("\nAudit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
