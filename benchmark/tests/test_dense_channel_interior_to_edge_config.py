"""Contracts for the interior-to-edge dense-channel benchmark."""

import json
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent


class DenseChannelInteriorToEdgeConfigTest(unittest.TestCase):
    def test_suite_config_points_at_all_eight_generated_cases(self):
        config = json.loads(
            (
                BENCH
                / "dense_channel_interior_to_edge_benchmark_v1.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(config["name"], "dense_channel_i2e_v1_8")
        self.assertEqual(config["protocol"]["methods"], ["carrier"])
        self.assertEqual(config["protocol"]["timeout_sec"], 10)
        self.assertEqual(config["protocol"]["solver_seed"], 0)
        self.assertEqual(len(config["groups"]), 1)
        group = config["groups"][0]
        self.assertEqual(group["expected_cases"], 8)
        self.assertEqual(group["family"], "dense_channel_i2e")
        self.assertEqual(
            group["root"],
            "viz_web/dense_channel_i2e_suite_v1_20260907",
        )


if __name__ == "__main__":
    unittest.main()
