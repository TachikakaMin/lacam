"""Tests for the B4 single-robot sequential baseline (Theorem 1 construction)."""

import unittest

from ddbench.b4_baseline import B4Failure, solve_b4
from ddbench.generators import ddmapd_instance, scramble_with_witness
from ddbench.instance import Instance, Target, parse_map_str
from ddbench.validator import is_goal, plan_cost, validate_plan


class TestB4(unittest.TestCase):
    def test_simple_carry(self):
        ins = Instance(
            grid=parse_map_str("....\n...."),
            robots=[(1, 0)],
            shelves=[(0, 1)],
            targets=[Target("b0", (0, 1), (0, 3))],
        )
        plan = solve_b4(ins)
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)
        self.assertTrue(is_goal(ins, s))

    def test_blocker_clearing(self):
        ins = Instance(
            grid=parse_map_str(".....\n....."),
            robots=[(1, 0)],
            shelves=[(0, 0), (0, 1), (0, 2)],
            targets=[Target("b0", (0, 0), (0, 4))],
        )
        plan = solve_b4(ins)
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)

    def test_multi_robot_instance_sequential_semantics(self):
        ins = Instance(
            grid=parse_map_str("....\n....\n...."),
            robots=[(2, 0), (2, 3)],
            shelves=[(0, 1)],
            targets=[Target("b0", (0, 1), (0, 3))],
        )
        plan = solve_b4(ins)
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)
        # sequential simulation: at most one robot acts per timestep
        for joint in plan:
            acting = sum(1 for a in joint if a[0] != "wait")
            self.assertLessEqual(acting, 1)

    def test_on_scrambled_instances(self):
        solved = 0
        for seed in range(8):
            ins, _ = scramble_with_witness(
                8, 8, n_robots=2, n_shelves=10, n_targets=3, k=40, seed=seed
            )
            try:
                plan = solve_b4(ins)
            except B4Failure:
                continue
            ok, errs, s = validate_plan(ins, plan)
            self.assertTrue(ok, f"seed={seed}: {errs}")
            solved += 1
        self.assertGreater(solved, 0, "B4 solved nothing on easy instances")

    def test_on_ddmapd_instance(self):
        ins = ddmapd_instance(12, 12, n_robots=4, block_density=0.3,
                              n_targets=4, seed=3)
        plan = solve_b4(ins)
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)
        cost = plan_cost(ins, plan)
        self.assertGreater(cost["executed_makespan"], 0)


if __name__ == "__main__":
    unittest.main()
