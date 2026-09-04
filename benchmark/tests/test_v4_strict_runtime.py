"""PROTECTED strict solver-return deadline contract (TDD RED)."""

import unittest


class TestV4StrictRuntime(unittest.TestCase):
    def test_runner_persists_solver_runtime(self):
        from run_benchmark import FIELDS

        self.assertIn("solver_runtime_ms", FIELDS)

    def test_success_solver_return_is_machine_checked(self):
        from run_benchmark import validate_solver_runtime_ms

        self.assertEqual(
            validate_solver_runtime_ms({"runtime_ms": "9999.5"}, 10),
            9999.5,
        )
        with self.assertRaisesRegex(ValueError, "runtime_ms"):
            validate_solver_runtime_ms({"runtime_ms": "10000.1"}, 10)
        with self.assertRaisesRegex(ValueError, "runtime_ms"):
            validate_solver_runtime_ms({}, 10)


if __name__ == "__main__":
    unittest.main()
