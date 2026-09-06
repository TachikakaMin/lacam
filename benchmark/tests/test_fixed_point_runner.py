"""Exact fixed-point work diagnostics at the benchmark trust boundary."""

import unittest

from run_benchmark import validate_carrier_work_metrics


class TestFixedPointRunner(unittest.TestCase):
    def setUp(self):
        self.cost = {
            "loaded_moves": 1,
            "free_moves": 2,
            "lift_drop": 3,
            "anon_moves": 4,
        }
        self.weights = (0.125, 8.0, 0.5, 0.25)
        self.expected = 18_625_000

    def test_lacam_requires_both_replay_and_best_work_to_match(self):
        metrics = {
            "weighted_work_scaled": str(self.expected),
            "best_work_scaled": str(self.expected),
        }
        self.assertEqual(
            validate_carrier_work_metrics(
                metrics, self.cost, self.weights, "lacam"
            ),
            self.expected,
        )

        for field in ("weighted_work_scaled", "best_work_scaled"):
            missing = dict(metrics)
            del missing[field]
            with self.subTest(field=field, case="missing"):
                with self.assertRaisesRegex(ValueError, field):
                    validate_carrier_work_metrics(
                        missing, self.cost, self.weights, "lacam"
                    )

            malformed = dict(metrics)
            malformed[field] = "18.625"
            with self.subTest(field=field, case="malformed"):
                with self.assertRaisesRegex(ValueError, field):
                    validate_carrier_work_metrics(
                        malformed, self.cost, self.weights, "lacam"
                    )

            mismatched = dict(metrics)
            mismatched[field] = str(self.expected + 1)
            with self.subTest(field=field, case="mismatch"):
                with self.assertRaisesRegex(ValueError, "work mismatch"):
                    validate_carrier_work_metrics(
                        mismatched, self.cost, self.weights, "lacam"
                    )

    def test_legacy_modes_require_final_replay_but_not_v5_best_work(self):
        metrics = {"weighted_work_scaled": str(self.expected)}
        for mode in ("b0", "b1"):
            with self.subTest(mode=mode):
                self.assertEqual(
                    validate_carrier_work_metrics(
                        metrics, self.cost, self.weights, mode
                    ),
                    self.expected,
                )

    def test_unrepresentable_weight_is_rejected(self):
        metrics = {
            "weighted_work_scaled": "0",
            "best_work_scaled": "0",
        }
        with self.assertRaisesRegex(ValueError, "1e-6"):
            validate_carrier_work_metrics(
                metrics, self.cost, (0.0000004, 0, 0, 0), "lacam"
            )


if __name__ == "__main__":
    unittest.main()
