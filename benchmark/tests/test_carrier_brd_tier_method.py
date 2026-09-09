"""PROTECTED contract for running carrier_brd on frozen benchmark tiers."""

import sys
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BENCH))

from run_benchmark import resolve_suite_methods  # noqa: E402


class TestCarrierBRDTierMethod(unittest.TestCase):
    def test_quick_and_full_may_replace_the_carrier_slot_with_brd(self):
        for tier in ("quick", "full"):
            self.assertEqual(
                resolve_suite_methods(
                    ["carrier"], ["carrier_brd"], tier
                ),
                ["carrier_brd"],
            )

    def test_omitted_or_identical_method_keeps_frozen_protocol(self):
        self.assertEqual(
            resolve_suite_methods(["carrier"], None, "quick"),
            ["carrier"],
        )
        self.assertEqual(
            resolve_suite_methods(
                ["carrier"], ["carrier"], "quick"
            ),
            ["carrier"],
        )

    def test_override_is_narrow_and_tier_only(self):
        with self.assertRaisesRegex(ValueError, "benchmark tier"):
            resolve_suite_methods(
                ["carrier"], ["carrier_brd"], None
            )
        with self.assertRaisesRegex(ValueError, "fixes methods"):
            resolve_suite_methods(
                ["carrier"], ["carrier_b0"], "quick"
            )
        with self.assertRaisesRegex(ValueError, "fixes methods"):
            resolve_suite_methods(
                ["carrier"], ["carrier", "carrier_brd"], "quick"
            )


if __name__ == "__main__":
    unittest.main()
