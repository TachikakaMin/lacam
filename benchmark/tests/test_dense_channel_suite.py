"""Dense-channel benchmark generation contracts."""

import hashlib
import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, load_instance
from ddbench.validator import (
    apply_joint_action,
    initial_state,
    is_goal,
    validate_plan,
)
from generate_web_viz import parse_plan
from generate_dense_channel_suite import (
    DENSE_SHELF_COUNTS,
    INTERIOR_TO_EDGE,
    INTERIOR_TO_BLOCK_EDGE_SET,
    TARGETS,
    build_dense_case,
    cell_depth,
    generate_suite,
)


class DenseChannelSuiteTest(unittest.TestCase):
    def test_block_edge_set_limits_each_targets_final_goal_set(self):
        generated = build_dense_case(
            block_size=3,
            shelves_per_block=8,
            seed=0,
            target_profile=INTERIOR_TO_BLOCK_EDGE_SET,
            height=20,
            width=20,
            n_robots=6,
            n_targets=8,
        )
        ins = generated["instance"]
        self.assertIn("dense_channel_bedge_h20w20", ins.name)
        self.assertEqual((ins.height, ins.width), (20, 20))
        self.assertEqual(len(ins.robots), 6)
        self.assertEqual(len(ins.targets), 8)
        self.assertIsNone(ins.goal_pool)

        block_sets = [set(block) for block in generated["blocks"]]
        for target in ins.targets:
            source_block = next(
                block for block in block_sets if target.start in block
            )
            goals = set(target.eligible_goals())
            self.assertEqual(len(goals), 8)
            self.assertTrue(goals <= source_block)
            self.assertTrue(
                all(cell_depth(goal, 3) == 0 for goal in goals)
            )
            self.assertGreaterEqual(cell_depth(target.start, 3), 1)

        self.assertEqual(ins.validate_static(), [])
        ok, errors, _ = validate_plan(ins, generated["witness"])
        self.assertTrue(ok, errors)

    def test_goal_set_does_not_limit_transit_or_temporary_storage(self):
        ins = Instance(
            grid=[[False] * 5],
            robots=[(0, 1)],
            shelves=[(0, 1)],
            targets=[
                Target(
                    id="b0",
                    start=(0, 1),
                    goal=(0, 0),
                    goals=[(0, 0), (0, 2)],
                )
            ],
            storage_cells={(0, col) for col in range(5)},
        )
        state = initial_state(ins)
        state = apply_joint_action(ins, state, [("lift",)])
        state = apply_joint_action(ins, state, [("move", (0, 2))])
        state = apply_joint_action(ins, state, [("move", (0, 3))])
        state = apply_joint_action(ins, state, [("drop",)])

        self.assertEqual(state.targets()["b0"], (0, 3))
        self.assertFalse(is_goal(ins, state))

        state = apply_joint_action(ins, state, [("lift",)])
        state = apply_joint_action(ins, state, [("move", (0, 2))])
        state = apply_joint_action(ins, state, [("drop",)])
        self.assertTrue(is_goal(ins, state))

    def test_interior_to_edge_cases_have_only_internal_starts_and_edge_goals(
        self,
    ):
        for block_size in (3, 4, 9):
            shelves_per_block = DENSE_SHELF_COUNTS[block_size][-1]
            generated = build_dense_case(
                block_size=block_size,
                shelves_per_block=shelves_per_block,
                seed=0,
                target_profile=INTERIOR_TO_EDGE,
            )
            ins = generated["instance"]
            self.assertIn("dense_channel_i2e_", ins.name)
            self.assertEqual(len(ins.targets), TARGETS)
            self.assertTrue(
                all(
                    cell_depth(target.start, block_size) >= 1
                    for target in ins.targets
                )
            )
            self.assertTrue(
                all(
                    cell_depth(target.goal, block_size) == 0
                    for target in ins.targets
                )
            )
            self.assertEqual(ins.validate_static(), [])
            ok, errors, _ = validate_plan(ins, generated["witness"])
            self.assertTrue(ok, errors)

    def test_interior_to_edge_suite_manifest_records_the_direction(self):
        expected_count = sum(
            len(counts) for counts in DENSE_SHELF_COUNTS.values()
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "suite"
            rows = generate_suite(
                root, target_profile=INTERIOR_TO_EDGE
            )
            self.assertEqual(len(rows), expected_count)
            for row in rows:
                self.assertEqual(row["task_profile"], INTERIOR_TO_EDGE)
                self.assertEqual(row["start_edge"], 0)
                self.assertEqual(row["start_inner"], TARGETS)
                self.assertEqual(row["goal_edge"], TARGETS)
                self.assertEqual(row["goal_inner"], 0)
                self.assertIn("dense_channel_i2e_", row["id"])

    def test_high_density_cases_have_interior_targets_and_valid_witnesses(self):
        for block_size in (3, 4, 9):
            shelves_per_block = DENSE_SHELF_COUNTS[block_size][-1]
            generated = build_dense_case(
                block_size=block_size,
                shelves_per_block=shelves_per_block,
                seed=0,
            )
            ins = generated["instance"]
            witness = generated["witness"]
            self.assertIn("_r8_t12_seed0", ins.name)
            start_depths = [
                cell_depth(target.start, block_size)
                for target in ins.targets
            ]
            goal_depths = [
                cell_depth(target.goal, block_size)
                for target in ins.targets
            ]

            self.assertEqual(len(ins.targets), TARGETS)
            self.assertGreaterEqual(
                sum(depth >= 1 for depth in start_depths),
                TARGETS // 2,
            )
            self.assertGreaterEqual(
                sum(depth >= 1 for depth in goal_depths),
                TARGETS // 3,
            )
            if block_size == 9:
                self.assertGreaterEqual(
                    sum(depth >= 2 for depth in start_depths),
                    TARGETS // 3,
                )
            self.assertTrue(
                all(robot in generated["corridor"] for robot in ins.robots)
            )
            self.assertEqual(ins.validate_static(), [])
            ok, errors, _ = validate_plan(ins, witness)
            self.assertTrue(ok, errors)

    def test_generated_suite_has_every_declared_density_once(self):
        expected = {
            (block_size, shelves_per_block)
            for block_size, counts in DENSE_SHELF_COUNTS.items()
            for shelves_per_block in counts
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows = generate_suite(root / "suite")
            actual = {
                (row["block_size"], row["shelves_per_block"])
                for row in rows
            }
            self.assertEqual(actual, expected)
            self.assertEqual(len(rows), len(expected))
            self.assertTrue((root / "suite" / "manifest.json").is_file())
            for row in rows:
                instance = load_instance(root / "suite" / row["yaml"])
                self.assertEqual(instance.validate_static(), [])
                shelf_cells = set(instance.shelves)
                generated = build_dense_case(
                    row["block_size"],
                    row["shelves_per_block"],
                    seed=0,
                )
                for block in generated["blocks"]:
                    self.assertEqual(
                        len(shelf_cells & set(block)),
                        row["shelves_per_block"],
                    )
                self.assertEqual(
                    len(shelf_cells),
                    len(generated["blocks"])
                    * row["shelves_per_block"],
                )
                self.assertAlmostEqual(
                    len(shelf_cells) / len(instance.storage_cells),
                    row["actual_density"],
                )
                certificate = root / "suite" / row["certificate"]
                self.assertTrue(certificate.is_file())
                ok, errors, _ = validate_plan(
                    instance, parse_plan(certificate)
                )
                self.assertTrue(ok, errors)

    def test_generation_is_byte_deterministic(self):
        def file_hashes(root):
            return {
                str(path.relative_to(root)): hashlib.sha256(
                    path.read_bytes()
                ).hexdigest()
                for path in sorted(root.rglob("*"))
                if path.is_file()
            }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = root / "first"
            second = root / "second"
            generate_suite(first)
            generate_suite(second)
            self.assertEqual(file_hashes(first), file_hashes(second))


if __name__ == "__main__":
    unittest.main()
