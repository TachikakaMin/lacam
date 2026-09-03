"""Warehouse-block generator contracts."""

import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, load_instance, parse_map_str
from ddbench.validator import (
    TransitionError,
    apply_joint_action,
    initial_state,
    legal_actions_for_robot,
)
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
        self.assertIn("function stopPlayback()", PLAN_VIZ_TEMPLATE)
        self.assertIn(
            "drawTween(states[t], states[t + 1], ease(tweenProgress))",
            PLAN_VIZ_TEMPLATE,
        )
        self.assertIn(
            "segmentStart = performance.now()"
            " - tweenProgress * stepDuration()",
            PLAN_VIZ_TEMPLATE,
        )
        self.assertNotIn("tweenProgress >= 0.5", PLAN_VIZ_TEMPLATE)
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
            block_of = {
                cell: index
                for index, block in enumerate(case["blocks"])
                for cell in block
            }
            block_cols = 20 // (block_size + 1)
            for target in case["targets"]:
                self.assertIn(tuple(target["start"]), case["storage"])
                self.assertIn(tuple(target["goal"]), case["storage"])
                source = block_of[tuple(target["start"])]
                destination = block_of[tuple(target["goal"])]
                source_rc = divmod(source, block_cols)
                destination_rc = divmod(destination, block_cols)
                self.assertEqual(
                    abs(source_rc[0] - destination_rc[0])
                    + abs(source_rc[1] - destination_rc[1]),
                    1,
                    "cross-block targets should cross one adjacent aisle",
                )

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
        self.assertEqual(loaded.storage_cells, case["storage"])
        self.assertTrue(set(loaded.shelves) <= case["storage"])
        self.assertTrue(
            all(tuple(target.goal) in case["storage"] for target in loaded.targets)
        )

    def test_storage_map_allows_transit_but_rejects_corridor_drop(self):
        ins = Instance(
            grid=parse_map_str("...."),
            robots=[(0, 1)],
            shelves=[(0, 1)],
            targets=[Target("b0", (0, 1), (0, 2))],
            storage_cells={(0, 1), (0, 2)},
        )
        self.assertEqual(ins.validate_static(), [])
        state = initial_state(ins)
        state = apply_joint_action(ins, state, [("lift",)])
        state = apply_joint_action(ins, state, [("move", (0, 0))])
        self.assertNotIn(("drop",), legal_actions_for_robot(ins, state, 0))
        with self.assertRaisesRegex(TransitionError, "storage|corridor"):
            apply_joint_action(ins, state, [("drop",)])

        state = apply_joint_action(ins, state, [("move", (0, 1))])
        apply_joint_action(ins, state, [("drop",)])

    def test_static_validation_rejects_storage_violations(self):
        bad_shelf = Instance(
            grid=parse_map_str("...."),
            robots=[(0, 0)],
            shelves=[(0, 0)],
            targets=[],
            storage_cells={(0, 1), (0, 2)},
        )
        self.assertTrue(
            any("storage" in error for error in bad_shelf.validate_static())
        )

        bad_goal = Instance(
            grid=parse_map_str("...."),
            robots=[(0, 0)],
            shelves=[(0, 1)],
            targets=[Target("b0", (0, 1), (0, 3))],
            storage_cells={(0, 1), (0, 2)},
        )
        self.assertTrue(
            any("storage" in error for error in bad_goal.validate_static())
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
