"""Warehouse-block generator contracts."""

import tempfile
import unittest
from pathlib import Path

from ddbench.instance import load_instance
from generate_web_viz import TEMPLATE as PLAN_VIZ_TEMPLATE
from generate_warehouse_block_sample import build_case, write_yaml
from generate_warehouse_block_suite import (
    BLOCK_SIZES,
    DENSITY_LEVELS,
    generate_suite,
)
from verify_warehouse_block_suite import verify_suite


class WarehouseGeneratorTest(unittest.TestCase):
    def test_plan_visualizer_uses_frame_interpolation(self):
        self.assertIn("requestAnimationFrame", PLAN_VIZ_TEMPLATE)
        self.assertIn("lerpPos", PLAN_VIZ_TEMPLATE)
        self.assertIn("p * p * (3 - 2 * p)", PLAN_VIZ_TEMPLATE)
        self.assertNotIn("setInterval", PLAN_VIZ_TEMPLATE)

    def test_uniform_blocks_and_one_cell_corridors(self):
        for block_size in (3, 4, 9):
            case = build_case(
                height=20,
                width=20,
                block_size=block_size,
                aisle_width=1,
                density=0.75,
                n_robots=8,
                n_targets=12,
                seed=0,
            )
            shelves = set(case["shelves"])
            counts = [len(set(block) & shelves) for block in case["blocks"]]
            self.assertEqual(len(set(counts)), 1)
            self.assertFalse(shelves & case["corridor"])
            self.assertTrue(set(case["robots"]) <= case["corridor"])
            for target in case["targets"]:
                self.assertIn(tuple(target["start"]), case["storage"])
                self.assertIn(tuple(target["goal"]), case["storage"])

            period = block_size + 1
            for r in range(0, 20, period):
                self.assertTrue(
                    all((r, c) in case["corridor"] for c in range(20))
                )
            for c in range(0, 20, period):
                self.assertTrue(
                    all((r, c) in case["corridor"] for r in range(20))
                )

    def test_generated_yaml_is_valid_and_respects_layout_contract(self):
        case = build_case(20, 20, 4, 1, 0.75, 8, 12, 0)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "case.yaml"
            write_yaml(case, path)
            loaded = load_instance(path)
        self.assertEqual(loaded.validate_static(), [])
        self.assertTrue(set(loaded.shelves) <= case["storage"])
        self.assertTrue(
            all(tuple(target.goal) in case["storage"] for target in loaded.targets)
        )

    def test_suite_contains_block_size_by_density_matrix(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows = generate_suite(root / "suite", root / "dashboard.html")
            self.assertEqual(len(rows), len(BLOCK_SIZES) * len(DENSITY_LEVELS))
            self.assertEqual(
                {int(row["block_size"]) for row in rows}, set(BLOCK_SIZES)
            )
            self.assertEqual(
                {float(row["requested_density"]) for row in rows},
                set(DENSITY_LEVELS),
            )
            self.assertTrue((root / "suite" / "manifest.csv").is_file())
            self.assertTrue((root / "suite" / "manifest.json").is_file())
            self.assertTrue((root / "dashboard.html").is_file())
            self.assertEqual(len(verify_suite(root / "suite")), len(rows))


if __name__ == "__main__":
    unittest.main()
