#!/usr/bin/env python3
"""Verify generated warehouse-block YAML files against their layout contract."""

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, List, Set, Tuple

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ddbench.instance import Cell, load_instance


def _block_cells(origin: Cell, block_size: int) -> Set[Cell]:
    r0, c0 = origin
    return {
        (r, c)
        for r in range(r0, r0 + block_size)
        for c in range(c0, c0 + block_size)
    }


def verify_case(path: Path) -> Dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    layout = raw.get("warehouse_layout")
    if not isinstance(layout, dict):
        raise ValueError(f"{path.name}: missing warehouse_layout metadata")

    ins = load_instance(path)
    errors = ins.validate_static()
    if errors:
        raise ValueError(f"{path.name}: DD validation failed: {errors}")

    block_size = int(layout["block_size"])
    aisle_width = int(layout["aisle_width"])
    shelves_per_block = int(layout["shelves_per_block"])
    origins = [tuple(origin) for origin in layout["block_origins"]]
    if aisle_width != 1:
        raise ValueError(f"{path.name}: aisle width is {aisle_width}, expected 1")

    period = block_size + aisle_width
    if ins.height % period or ins.width % period:
        raise ValueError(f"{path.name}: block/aisle pattern does not tile map")
    expected_origins = [
        (base_r + aisle_width, base_c + aisle_width)
        for base_r in range(0, ins.height, period)
        for base_c in range(0, ins.width, period)
    ]
    if origins != expected_origins:
        raise ValueError(f"{path.name}: block origins do not match 1-cell aisles")

    blocks = [_block_cells(origin, block_size) for origin in origins]
    storage = set().union(*blocks)
    corridor = {
        (r, c)
        for r in range(ins.height)
        for c in range(ins.width)
        if (r, c) not in storage
    }
    shelves = set(ins.shelves)
    if not shelves <= storage:
        raise ValueError(f"{path.name}: shelf placed in corridor")
    if not set(ins.robots) <= corridor:
        raise ValueError(f"{path.name}: robot does not start in corridor")

    counts = [len(block & shelves) for block in blocks]
    if any(count != shelves_per_block for count in counts):
        raise ValueError(
            f"{path.name}: non-uniform block shelf counts {counts}"
        )

    block_of: Dict[Cell, int] = {}
    for index, block in enumerate(blocks):
        for cell in block:
            block_of[cell] = index
    for target in ins.targets:
        start = tuple(target.start)
        goals = [tuple(goal) for goal in target.eligible_goals()]
        if start not in storage or any(goal not in storage for goal in goals):
            raise ValueError(f"{path.name}: target endpoint in corridor")
        if len(blocks) > 1 and any(
            block_of[start] == block_of[goal] for goal in goals
        ):
            raise ValueError(
                f"{path.name}: target does not relocate across blocks"
            )

    actual_density = shelves_per_block / (block_size * block_size)
    metadata_density = float(layout["actual_density"])
    if abs(actual_density - metadata_density) > 1e-8:
        raise ValueError(f"{path.name}: actual density metadata mismatch")

    return {
        "name": ins.name,
        "block_size": block_size,
        "requested_density": float(layout["requested_density"]),
        "actual_density": actual_density,
        "blocks": len(blocks),
        "shelves_per_block": shelves_per_block,
        "shelves": len(shelves),
    }


def verify_suite(suite_dir: Path) -> List[Dict]:
    manifest_path = suite_dir / "manifest.csv"
    with manifest_path.open(encoding="utf-8", newline="") as stream:
        manifest = list(csv.DictReader(stream))
    if not manifest:
        raise ValueError("manifest.csv is empty")

    summaries: List[Dict] = []
    for row in manifest:
        yaml_path = suite_dir / row["yaml"]
        if not yaml_path.is_file():
            raise ValueError(f"missing YAML: {yaml_path}")
        html_path = suite_dir / row["html"]
        if not html_path.is_file():
            raise ValueError(f"missing visualization: {html_path}")
        summary = verify_case(yaml_path)
        if summary["name"] != row["name"]:
            raise ValueError(f"{yaml_path.name}: manifest name mismatch")
        summaries.append(summary)

    combinations = {
        (summary["block_size"], round(summary["requested_density"], 2))
        for summary in summaries
    }
    expected = {
        (block_size, density)
        for block_size in (3, 4, 9)
        for density in (0.25, 0.50, 0.75)
    }
    if combinations != expected:
        raise ValueError(
            f"suite matrix mismatch: got {sorted(combinations)}"
        )
    if len(summaries) != len(expected):
        raise ValueError(
            f"suite has {len(summaries)} cases, expected {len(expected)}"
        )
    return summaries


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "suite_dir",
        nargs="?",
        type=Path,
        default=here / "viz_web" / "warehouse_block_suite",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summaries = verify_suite(args.suite_dir)
    for summary in summaries:
        print(
            f"{summary['name']}: block={summary['block_size']}x"
            f"{summary['block_size']}, requested="
            f"{summary['requested_density']:.0%}, actual="
            f"{summary['actual_density']:.1%}, blocks={summary['blocks']}, "
            f"shelves/block={summary['shelves_per_block']}, "
            f"total={summary['shelves']}"
        )
    print(f"verified_cases={len(summaries)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
