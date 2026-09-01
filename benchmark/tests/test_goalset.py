"""PROTECTED tests: goal-set instance format + terminal condition on the
Python oracle side (design_final 2.1/2.2/Prop 3; debug.md v4 WP-A T1/T2).
Written BEFORE implementation (TDD RED).
"""
import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, load_instance, save_instance
from ddbench.validator import State, initial_state, is_goal

GOALSET_YAML = """\
name: goalset_fixture
map: |
  .....
  .....
  .....
robots:
  - [2, 0]
shelves:
  - [0, 1]
  - [0, 2]
  - [1, 1]
goal_pool:
  - [2, 4]
  - [2, 3]
targets:
  - {id: b0, start: [0, 1], goal: [0, 4]}
  - {id: b1, start: [0, 2], goals: [[1, 4], [1, 3]]}
  - {id: b2, start: [1, 1], goals: pool}
"""

UNCOVERABLE_YAML = """\
name: goalset_uncoverable
map: |
  ....
  ....
robots:
  - [1, 0]
shelves:
  - [0, 0]
  - [0, 1]
goal_pool:
  - [0, 3]
targets:
  - {id: b0, start: [0, 0], goals: pool}
  - {id: b1, start: [0, 1], goals: pool}
"""


def _write(tmp, name, text):
    p = Path(tmp) / name
    p.write_text(text)
    return p


class GoalSetLoaderTest(unittest.TestCase):
    def test_load_pool_list_and_singleton(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = load_instance(_write(tmp, "gs.yaml", GOALSET_YAML))
        by_id = {t.id: t for t in ins.targets}
        # old singleton form: eligible set is {goal}
        self.assertEqual(by_id["b0"].eligible_goals(), [(0, 4)])
        # explicit list (unsorted in YAML) -> sorted unique
        self.assertEqual(by_id["b1"].eligible_goals(), [(1, 3), (1, 4)])
        # pool reference -> shared pool, sorted
        self.assertEqual(by_id["b2"].eligible_goals(), [(2, 3), (2, 4)])
        # representative goal = sorted-first of the set
        self.assertEqual(tuple(by_id["b1"].goal), (1, 3))
        self.assertEqual(tuple(by_id["b2"].goal), (2, 3))
        self.assertEqual(ins.goal_pool, [(2, 3), (2, 4)])

    def test_old_format_has_singleton_eligible(self):
        ins = Instance(
            grid=[[False] * 4],
            robots=[(0, 0)],
            shelves=[(0, 1)],
            targets=[Target(id="b0", start=(0, 1), goal=(0, 3))],
        )
        self.assertEqual(ins.targets[0].eligible_goals(), [(0, 3)])

    def test_validate_rejects_uncoverable(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = load_instance(_write(tmp, "un.yaml", UNCOVERABLE_YAML))
        errors = ins.validate_static()
        self.assertTrue(any("covering" in e for e in errors), errors)

    def test_validate_ok_when_pool_covers(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = load_instance(_write(tmp, "gs.yaml", GOALSET_YAML))
        self.assertEqual(ins.validate_static(), [])

    def test_save_load_roundtrip_preserves_sets(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = load_instance(_write(tmp, "gs.yaml", GOALSET_YAML))
            out = Path(tmp) / "rt.yaml"
            save_instance(ins, out)
            back = load_instance(out)
        for a, b in zip(ins.targets, back.targets):
            self.assertEqual(a.eligible_goals(), b.eligible_goals())
        self.assertEqual(ins.goal_pool, back.goal_pool)


class GoalSetTerminalTest(unittest.TestCase):
    def _pool_instance(self):
        return Instance(
            grid=[[False] * 5 for _ in range(2)],
            robots=[(1, 0)],
            shelves=[(0, 0), (0, 1)],
            targets=[
                Target(id="b0", start=(0, 0), goal=(0, 2),
                       goals=[(0, 2), (0, 3), (0, 4)]),
                Target(id="b1", start=(0, 1), goal=(0, 2),
                       goals=[(0, 2), (0, 3), (0, 4)]),
            ],
            goal_pool=[(0, 2), (0, 3), (0, 4)],
        )

    def test_is_goal_accepts_any_eligible(self):
        ins = self._pool_instance()
        s0 = initial_state(ins)
        # both grounded on NON-representative pool cells => goal (Prop 3)
        s = State(
            robots=s0.robots,
            target_pos=(("b0", (0, 3)), ("b1", (0, 4))),
            anon_occ=frozenset(),
            kappa=(None,),
        )
        self.assertTrue(is_goal(ins, s))
        # non-eligible cell: not a goal
        s2 = State(
            robots=s0.robots,
            target_pos=(("b0", (0, 3)), ("b1", (1, 4))),
            anon_occ=frozenset(),
            kappa=(None,),
        )
        self.assertFalse(is_goal(ins, s2))
        # carried on an eligible cell: not grounded -> not a goal (D10)
        s3 = State(
            robots=((0, 4),),
            target_pos=(("b0", (0, 3)), ("b1", (0, 4))),
            anon_occ=frozenset(),
            kappa=("b1",),
        )
        self.assertFalse(is_goal(ins, s3))


if __name__ == "__main__":
    unittest.main()


DUPLICATE_START_YAML = """\
name: goalset_duplicate_start
map: |
  ....
  ....
robots:
  - [1, 0]
shelves:
  - [0, 0]
  - [0, 1]
targets:
  - {id: b0, start: [0, 0], goal: [0, 2]}
  - {id: b1, start: [0, 0], goal: [0, 3]}
"""


class DuplicateTargetStartTest(unittest.TestCase):
    # review fix batch 2026-09-01 (TDD RED): two labeled targets sharing
    # one initial shelf cell describe one physical shelf with two
    # identities; validate_static must flag it (the shelves-overlap check
    # does not cover it because both targets reference the SAME entry).
    def test_validate_rejects_duplicate_target_starts(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = load_instance(
                _write(tmp, "dup.yaml", DUPLICATE_START_YAML))
            errors = ins.validate_static()
            self.assertTrue(
                any("start" in e and "duplicate" in e for e in errors),
                f"expected a duplicate-target-start error, got {errors}")
