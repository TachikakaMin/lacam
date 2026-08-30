"""Theorem 1 boundary regression (debug.md round-2 P0-1, design.md 4.1).

The audit counterexample, frozen as an executable test: on a 1x2 connected
lower deck fully occupied by TWO robots, the upper-deck sequential
pebble-motion move (shelf left -> right) exists, yet the Carrier problem is
infeasible — exhaustive reachability over the authoritative validator finds
exactly 2 physical states and no goal.  With |R| = 1 (the corrected theorem
statement) the same layout is solvable.

These tests guard the validator semantics the theorem relies on: if a
future change accidentally legalised swaps (R2) or same-cell stacking (R1),
the 2-robot instance would become solvable and this test would fail.
"""

import itertools
import unittest

from ddbench.instance import Instance, Target, parse_map_str
from ddbench.validator import (
    TransitionError,
    apply_joint_action,
    initial_state,
    is_goal,
    legal_actions_for_robot,
)


def make(map_str, robots, shelves, targets):
    return Instance(
        grid=parse_map_str(map_str),
        robots=robots,
        shelves=shelves,
        targets=[Target(*t) for t in targets],
    )


def reachable_states(ins, cap=10000):
    """Exhaustive BFS over joint actions; returns (states, goal_reached)."""
    s0 = initial_state(ins)
    seen = {s0}
    frontier = [s0]
    goal = is_goal(ins, s0)
    while frontier and len(seen) < cap:
        nxt = []
        for s in frontier:
            per_robot = [
                legal_actions_for_robot(ins, s, i) for i in range(len(s.robots))
            ]
            for joint in itertools.product(*per_robot):
                try:
                    s2 = apply_joint_action(ins, s, list(joint))
                except TransitionError:
                    continue
                if s2 not in seen:
                    seen.add(s2)
                    nxt.append(s2)
                    if is_goal(ins, s2):
                        goal = True
        frontier = nxt
    return seen, goal


class TestTheorem1Boundary(unittest.TestCase):
    def test_two_robots_saturate_lower_deck_infeasible(self):
        # 1x2 corridor, both cells hold a robot, shelf must go left->right.
        ins = make("..", [(0, 0), (0, 1)], [(0, 0)], [("b0", (0, 0), (0, 1))])
        states, goal = reachable_states(ins)
        self.assertFalse(goal, "goal must be unreachable with |R|=2 here")
        # audit-verified state count: {initial, r0-carrying}
        self.assertEqual(len(states), 2)

    def test_single_robot_control_is_solvable(self):
        # identical layout minus the second robot: theorem 1 construction
        # (lift -> loaded move -> drop) applies.
        ins = make("..", [(0, 0)], [(0, 0)], [("b0", (0, 0), (0, 1))])
        states, goal = reachable_states(ins)
        self.assertTrue(goal, "|R|=1 must solve the same layout")
        self.assertGreater(len(states), 2)


if __name__ == "__main__":
    unittest.main()
