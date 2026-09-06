"""Random paired-factorial warehouse testcase generator contracts."""

import unittest
from collections import Counter, defaultdict

from ddbench.validator import validate_plan
from generate_warehouse_factorial_suite import (
    DENSITY_LEVELS,
    GOAL_MODES,
    MAP_FAMILIES,
    TASK_PROFILES,
    factorial_case_specs,
    materialize_pairing_group,
)


class TestWarehouseFactorialSuite(unittest.TestCase):
    def test_matrix_is_the_complete_432_case_cartesian_product(self):
        rows = factorial_case_specs()
        self.assertEqual(len(rows), 432)
        self.assertEqual(len({row["id"] for row in rows}), 432)
        self.assertEqual(
            Counter(row["map_family"] for row in rows),
            {family: 72 for family in MAP_FAMILIES},
        )

        by_family = defaultdict(set)
        for row in rows:
            by_family[row["map_family"]].add(
                (
                    row["density_level"],
                    row["agent_level"],
                    row["task_profile"],
                    row["goal_mode"],
                )
            )
        expected = {
            (density, agent, profile, goal_mode)
            for density in DENSITY_LEVELS
            for agent in ("scarce", "baseline", "equal", "surplus")
            for profile in TASK_PROFILES
            for goal_mode in GOAL_MODES
        }
        self.assertTrue(
            all(combinations == expected for combinations in by_family.values())
        )

    def test_one_pairing_group_is_nested_and_validator_certified(self):
        variants = materialize_pairing_group(
            "g1", "mixed", "shared_pool", seed=0
        )
        self.assertEqual(len(variants), 12)

        target_signatures = {
            tuple(
                (
                    target.id,
                    target.start,
                    tuple(target.eligible_goals()),
                )
                for target in variant["instance"].targets
            )
            for variant in variants
        }
        self.assertEqual(len(target_signatures), 1)

        by_density = defaultdict(list)
        by_agent = defaultdict(list)
        for variant in variants:
            ins = variant["instance"]
            self.assertEqual(ins.validate_static(), [])
            ok, errors, _ = validate_plan(ins, variant["certificate"])
            self.assertTrue(ok, errors)
            self.assertEqual(
                variant["metadata"]["relation_counts"],
                variant["spec"]["relation_counts"],
            )
            by_density[variant["spec"]["density_level"]].append(variant)
            by_agent[variant["spec"]["agent_level"]].append(variant)

        for density_variants in by_density.values():
            ordered = sorted(
                density_variants,
                key=lambda item: item["spec"]["robots"],
            )
            for smaller, larger in zip(ordered, ordered[1:]):
                self.assertEqual(
                    larger["instance"].robots[: len(smaller["instance"].robots)],
                    smaller["instance"].robots,
                )

        for agent_variants in by_agent.values():
            shelves = {
                item["spec"]["density_level"]: set(item["instance"].shelves)
                for item in agent_variants
            }
            self.assertLess(shelves["low"], shelves["medium"])
            self.assertLess(shelves["medium"], shelves["high"])


if __name__ == "__main__":
    unittest.main()
