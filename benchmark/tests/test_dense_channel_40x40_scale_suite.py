"""Contracts for the 40x40 block-edge goal-set scaling benchmark."""

import json
import unittest
from pathlib import Path

from generate_dense_channel_scale_suite import (
    HEIGHT,
    LOAD_LEVELS,
    WIDTH,
    build_case_specs,
)


BENCH = Path(__file__).resolve().parent.parent


class DenseChannel40x40ScaleSuiteTest(unittest.TestCase):
    def test_case_specs_cross_three_blocks_with_three_load_levels(self):
        specs = build_case_specs()
        self.assertEqual((HEIGHT, WIDTH), (40, 40))
        self.assertEqual(
            LOAD_LEVELS,
            (
                ("small", 8, 12),
                ("medium", 16, 24),
                ("large", 32, 48),
            ),
        )
        self.assertEqual(len(specs), 9)
        self.assertEqual(
            {
                (spec["block_size"], spec["shelves_per_block"])
                for spec in specs
            },
            {(3, 8), (4, 15), (9, 76)},
        )
        self.assertEqual(
            {(spec["robots"], spec["targets"]) for spec in specs},
            {(8, 12), (16, 24), (32, 48)},
        )

    def test_suite_config_points_at_the_nine_case_matrix(self):
        config = json.loads(
            (
                BENCH
                / "dense_channel_block_edge_40x40_benchmark_v1.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(
            config["name"], "dense_channel_block_edge_40x40_v1_9"
        )
        self.assertEqual(config["groups"][0]["expected_cases"], 9)
        self.assertEqual(
            config["groups"][0]["root"],
            "viz_web/dense_channel_block_edge_40x40_suite_v1_20260907",
        )
        self.assertEqual(
            config["groups"][0]["family"],
            "dense_channel_block_edge_40x40",
        )


if __name__ == "__main__":
    unittest.main()
