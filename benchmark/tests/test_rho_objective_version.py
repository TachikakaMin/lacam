"""PROTECTED production rho objective-version export contract."""

import unittest


class TestRhoObjectiveVersion(unittest.TestCase):
    def test_runner_persists_explicit_objective_version(self):
        from run_benchmark import FIELDS

        self.assertIn("rho_objective_version", FIELDS)


if __name__ == "__main__":
    unittest.main()
