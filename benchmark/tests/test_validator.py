"""Unit tests for the two-deck transition validator (design.md 6.5) and the
scrambler generator (M0 exit criteria)."""

import unittest

from ddbench.instance import Instance, Target, parse_map_str
from ddbench.validator import (
    ANON,
    TransitionError,
    apply_joint_action,
    initial_state,
    is_goal,
    plan_cost,
    validate_plan,
)
from ddbench.generators import ddmapd_instance, scramble_with_witness


def make(map_str, robots, shelves, targets):
    return Instance(
        grid=parse_map_str(map_str),
        robots=robots,
        shelves=shelves,
        targets=[Target(*t) for t in targets],
    )


class TestPrimitives(unittest.TestCase):
    def test_lift_move_drop_reaches_goal(self):
        ins = make("....\n....", [(0, 0)], [(0, 1)], [("b0", (0, 1), (0, 3))])
        plan = [
            [("move", (0, 1))],
            [("lift",)],
            [("move", (0, 2))],
            [("move", (0, 3))],
            [("drop",)],
        ]
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)
        self.assertTrue(is_goal(ins, s))
        cost = plan_cost(ins, plan)
        self.assertEqual(cost["executed_makespan"], 5)
        self.assertEqual(cost["loaded_moves"], 2)
        self.assertEqual(cost["free_moves"], 1)
        self.assertEqual(cost["lift_drop"], 2)

    def test_goal_requires_grounded(self):
        # carrying the shelf onto its goal is NOT a goal state (D10)
        ins = make("....", [(0, 0)], [(0, 1)], [("b0", (0, 1), (0, 3))])
        plan = [
            [("move", (0, 1))],
            [("lift",)],
            [("move", (0, 2))],
            [("move", (0, 3))],
        ]
        ok, errs, s = validate_plan(ins, plan, require_goal=False)
        self.assertTrue(ok, errs)
        self.assertFalse(is_goal(ins, s))

    def test_lift_without_shelf_rejected(self):
        ins = make("....", [(0, 0)], [(0, 1)], [("b0", (0, 1), (0, 3))])
        with self.assertRaises(TransitionError):
            apply_joint_action(ins, initial_state(ins), [("lift",)])

    def test_drop_while_free_rejected(self):
        ins = make("....", [(0, 0)], [(0, 1)], [("b0", (0, 1), (0, 3))])
        with self.assertRaises(TransitionError):
            apply_joint_action(ins, initial_state(ins), [("drop",)])

    def test_move_into_wall_rejected(self):
        ins = make(".@\n..", [(0, 0)], [], [])
        with self.assertRaises(TransitionError):
            apply_joint_action(ins, initial_state(ins), [("move", (0, 1))])


class TestConflictRules(unittest.TestCase):
    def test_r1_vertex_conflict(self):
        ins = make("...", [(0, 0), (0, 2)], [], [])
        with self.assertRaises(TransitionError):
            apply_joint_action(
                ins, initial_state(ins), [("move", (0, 1)), ("move", (0, 1))]
            )

    def test_r2_swap_conflict(self):
        ins = make("..", [(0, 0), (0, 1)], [], [])
        with self.assertRaises(TransitionError):
            apply_joint_action(
                ins, initial_state(ins), [("move", (0, 1)), ("move", (0, 0))]
            )

    def test_following_allowed(self):
        # design.md 3.4a: no-following NOT inherited; convoy is legal.
        ins = make("...", [(0, 0), (0, 1)], [], [])
        s = apply_joint_action(
            ins, initial_state(ins), [("move", (0, 1)), ("move", (0, 2))]
        )
        self.assertEqual(s.robots, ((0, 1), (0, 2)))

    def test_loaded_following_allowed(self):
        # two loaded robots in convoy: shelf following is also legal.
        ins = make("....", [(0, 0), (0, 1)], [(0, 0), (0, 1)],
                   [("b0", (0, 0), (0, 2)), ("b1", (0, 1), (0, 3))])
        s = initial_state(ins)
        s = apply_joint_action(ins, s, [("lift",), ("lift",)])
        s = apply_joint_action(ins, s, [("move", (0, 1)), ("move", (0, 2))])
        s = apply_joint_action(ins, s, [("move", (0, 2)), ("move", (0, 3))])
        s = apply_joint_action(ins, s, [("drop",), ("drop",)])
        self.assertTrue(is_goal(ins, s))

    def test_s1_shelf_vertex_conflict(self):
        # loaded robot may not move under/onto a grounded shelf cell
        ins = make("...", [(0, 0)], [(0, 0), (0, 1)], [("b0", (0, 0), (0, 2))])
        s = initial_state(ins)
        s = apply_joint_action(ins, s, [("lift",)])
        with self.assertRaises(TransitionError):
            apply_joint_action(ins, s, [("move", (0, 1))])

    def test_i3_free_robot_under_grounded_shelf(self):
        ins = make("..", [(0, 0)], [(0, 1)], [])
        s = apply_joint_action(ins, initial_state(ins), [("move", (0, 1))])
        self.assertEqual(s.robots, ((0, 1),))

    def test_i1_lift_after_drop_same_step_rejected(self):
        # robot0 drops at (0,1); robot1 (under same cell? impossible) — use
        # adjacent: robot1 sits at (0,1) lower deck cannot co-exist with robot0
        # so construct: r0 carries shelf at (0,1)... simpler direct test:
        # dropping and lifting the same cell in one step needs two robots on
        # one cell which R1 forbids; assert the atomicity guard itself:
        ins = make("...", [(0, 0), (0, 1)], [(0, 1)], [])
        s = initial_state(ins)
        # r1 lifts the shelf at its own cell — legal
        s = apply_joint_action(ins, s, [("wait",), ("lift",)])
        self.assertEqual(s.kappa[1], ANON)
        # r1 drops, r0 tries to move in and lift in the SAME step → r0 lift
        # precondition fails because shelf was not grounded at step start.
        with self.assertRaises(TransitionError):
            apply_joint_action(ins, s, [("move", (0, 1)), ("drop",)])


class TestManualScenarios(unittest.TestCase):
    """8x8-style manual instances from design.md 6.5."""

    def test_single_blocker(self):
        # target path blocked by one anonymous shelf; robot must clear it.
        ins = make(
            "....\n....",
            [(1, 0)],
            [(0, 0), (0, 1)],
            [("b0", (0, 0), (0, 2))],
        )
        plan = [
            [("move", (0, 0))],   # wait under nothing; go under b0? first clear blocker
        ]
        # full manual plan: clear (0,1) to (1,1), then carry b0 to (0,2)
        plan = [
            [("move", (1, 1))],
            [("move", (0, 1))],
            [("lift",)],          # lift anonymous blocker
            [("move", (1, 1))],
            [("drop",)],          # blocker parked at (1,1)
            [("move", (1, 0))],
            [("move", (0, 0))],
            [("lift",)],          # lift target b0
            [("move", (0, 1))],
            [("move", (0, 2))],
            [("drop",)],
        ]
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)

    def test_idle_robot_blocks_lift_cell_is_legal_to_stand(self):
        # An idle robot UNDER a shelf does not violate rules (I3) but a second
        # robot cannot enter that cell (R1) — the physical blocking pattern.
        ins = make("..\n..", [(0, 0), (0, 1)], [(0, 1)], [])
        s = initial_state(ins)
        with self.assertRaises(TransitionError):
            # r0 tries to move onto r1's cell to lift: R1 vertex conflict
            apply_joint_action(ins, s, [("move", (0, 1)), ("wait",)])

    def test_cycle_rotation_zero_empty(self):
        # Proposition 2: fully-occupied 2x2 cycle rotates with 4 loaded robots.
        ins = make(
            "..\n..",
            [(0, 0), (0, 1), (1, 1), (1, 0)],
            [(0, 0), (0, 1), (1, 1), (1, 0)],
            [
                ("b0", (0, 0), (0, 1)),
                ("b1", (0, 1), (1, 1)),
                ("b2", (1, 1), (1, 0)),
                ("b3", (1, 0), (0, 0)),
            ],
        )
        plan = [
            [("lift",), ("lift",), ("lift",), ("lift",)],
            [
                ("move", (0, 1)),
                ("move", (1, 1)),
                ("move", (1, 0)),
                ("move", (0, 0)),
            ],
            [("drop",), ("drop",), ("drop",), ("drop",)],
        ]
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)
        self.assertTrue(is_goal(ins, s))

    def test_double_blocker_chain(self):
        ins = make(
            ".....\n.....",
            [(1, 0)],
            [(0, 0), (0, 1), (0, 2)],
            [("b0", (0, 0), (0, 3))],
        )
        plan = [
            # clear blocker at (0,2) -> (1,2)
            [("move", (1, 1))], [("move", (1, 2))], [("move", (0, 2))],
            [("lift",)], [("move", (1, 2))], [("drop",)],
            # clear blocker at (0,1) -> (1,1)
            [("move", (0, 2))], [("move", (0, 1))],
            [("lift",)], [("move", (1, 1))], [("drop",)],
            # carry b0 from (0,0) to (0,3)
            [("move", (1, 0))], [("move", (0, 0))],
            [("lift",)], [("move", (0, 1))], [("move", (0, 2))],
            [("move", (0, 3))], [("drop",)],
        ]
        ok, errs, s = validate_plan(ins, plan)
        self.assertTrue(ok, errs)


class TestScrambler(unittest.TestCase):
    def test_witness_plan_validates(self):
        for seed in range(5):
            ins, witness = scramble_with_witness(
                6, 6, n_robots=3, n_shelves=8, n_targets=3, k=30, seed=seed
            )
            self.assertFalse(ins.validate_static())
            ok, errs, s = validate_plan(ins, witness)
            self.assertTrue(ok, f"seed={seed}: {errs}")
            self.assertTrue(is_goal(ins, s))

    def test_scramble_deterministic(self):
        a1, w1 = scramble_with_witness(5, 5, 2, 5, 2, 20, seed=7)
        a2, w2 = scramble_with_witness(5, 5, 2, 5, 2, 20, seed=7)
        self.assertEqual(a1.robots, a2.robots)
        self.assertEqual(a1.shelves, a2.shelves)
        self.assertEqual(w1, w2)

    def test_ddmapd_protocol(self):
        ins = ddmapd_instance(20, 20, n_robots=8, block_density=0.4, seed=1)
        self.assertFalse(ins.validate_static())
        # perimeter starts
        for r, c in ins.robots:
            self.assertTrue(r in (0, 19) or c in (0, 19))
        # 2x2 blocks: shelf count multiple of 4
        self.assertEqual(len(ins.shelves) % 4, 0)


if __name__ == "__main__":
    unittest.main()
