"""plan_cost oscillation metric tests (debug.md round-2 P2-13a).

`reversals` counts, per robot and summed, immediate direction flips in the
POSITION history: windows (t, t+1, t+2) where the robot moved at t and is
back on its t-position at t+2 (pos[t+2] == pos[t], pos[t+1] != pos[t]).
A wait between the two moves does NOT count — the metric targets the
visible A->B->A jitter, not legitimate return trips.
"""

import unittest

from ddbench.instance import Instance, Target, parse_map_str
from ddbench.validator import plan_cost, validate_plan


def make(map_str, robots, shelves, targets):
    return Instance(
        grid=parse_map_str(map_str),
        robots=robots,
        shelves=shelves,
        targets=[Target(*t) for t in targets],
    )


class TestReversalsMetric(unittest.TestCase):
    def _cost(self, ins, plan):
        ok, errs, _ = validate_plan(ins, plan, require_goal=False)
        self.assertTrue(ok, errs)
        return plan_cost(ins, plan)

    def test_immediate_flip_counts_once(self):
        ins = make("...", [(0, 0)], [], [])
        plan = [
            [("move", (0, 1))],
            [("move", (0, 0))],  # A->B->A
        ]
        c = self._cost(ins, plan)
        self.assertEqual(c["reversals"], 1)

    def test_straight_walk_counts_zero(self):
        ins = make("....", [(0, 0)], [], [])
        plan = [
            [("move", (0, 1))],
            [("move", (0, 2))],
            [("move", (0, 3))],
        ]
        self.assertEqual(self._cost(ins, plan)["reversals"], 0)

    def test_wait_between_moves_does_not_count(self):
        ins = make("...", [(0, 0)], [], [])
        plan = [
            [("move", (0, 1))],
            [("wait",)],
            [("move", (0, 0))],  # A->B, pause, B->A: legit return trip
        ]
        self.assertEqual(self._cost(ins, plan)["reversals"], 0)

    def test_pingpong_counts_every_flip(self):
        ins = make("...", [(0, 0)], [], [])
        plan = [
            [("move", (0, 1))],
            [("move", (0, 0))],
            [("move", (0, 1))],
            [("move", (0, 0))],
        ]
        # windows: (0,1,2)=flip, (1,2,3)=flip, (2,3,4)=flip -> wait, plan
        # positions: A B A B A -> windows t=0..2 all flips = 3
        self.assertEqual(self._cost(ins, plan)["reversals"], 3)

    def test_per_robot_summed(self):
        ins = make("...\n...", [(0, 0), (1, 2)], [], [])
        plan = [
            [("move", (0, 1)), ("move", (1, 1))],
            [("move", (0, 0)), ("move", (1, 2))],  # both flip
        ]
        self.assertEqual(self._cost(ins, plan)["reversals"], 2)


if __name__ == "__main__":
    unittest.main()
