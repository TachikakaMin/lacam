"""PROTECTED release-benchmark membership and protocol regression.

The formal Carrier-LaCAM release suite is the original 68-case BRaP pool
plus every real warehouse-block YAML represented by the visualization suite.
Written before implementation on 2026-09-04 and intentionally observed RED.
"""

import json
import sys
import unittest
from pathlib import Path

from ddbench.instance import load_instance

BENCH = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BENCH))

from run_benchmark import discover_suite_cases, load_suite_definition


SUITE = BENCH / "release_benchmark.json"
WAREHOUSE_MANIFEST = (
    BENCH / "viz_web" / "warehouse_block_suite" / "manifest.json"
)


class TestReleaseBenchmarkSuite(unittest.TestCase):
    def test_protocol_is_the_fixed_release_protocol(self):
        definition = load_suite_definition(SUITE)
        self.assertEqual(definition["name"], "carrier_release_77")
        self.assertEqual(
            definition["protocol"],
            {
                "methods": ["carrier"],
                "timeout_sec": 10,
                "jobs": 14,
                "solver_seed": 0,
                "objective_weights": [1, 1, 1, 1],
                "following": "allowed",
            },
        )

    def test_suite_contains_original_68_plus_all_9_warehouse_cases(self):
        _, cases, group_counts = discover_suite_cases(SUITE)
        self.assertEqual(group_counts, {"brap_pool": 68, "warehouse_blocks": 9})
        self.assertEqual(len(cases), 77)
        self.assertEqual(len({path.resolve() for path, _ in cases}), 77)
        self.assertEqual(len({path.stem for path, _ in cases}), 77)

        expected_warehouse = {
            row["name"]
            for row in json.loads(WAREHOUSE_MANIFEST.read_text())
        }
        actual_warehouse = {
            path.stem
            for path, family in cases
            if family == "warehouse_blocks"
        }
        self.assertEqual(actual_warehouse, expected_warehouse)
        self.assertEqual(len(actual_warehouse), 9)

    def test_every_release_case_is_a_real_valid_yaml_instance(self):
        _, cases, _ = discover_suite_cases(SUITE)
        for path, _ in cases:
            self.assertEqual(
                load_instance(path).validate_static(),
                [],
                path.name,
            )


if __name__ == "__main__":
    unittest.main()
