"""PROTECTED vacancy/churn telemetry contract for the carrier method.

Written before implementation on 2026-09-07 and intentionally observed RED.
"""

import sys
import tempfile
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build" / "dd_benchmark"
CASE = (
    BENCH
    / "instances_brap_pool"
    / "g10x10"
    / "brap_h10w10_a1_e1_B_seed0_pool.yaml"
)
sys.path.insert(0, str(BENCH))

import run_benchmark as runner  # noqa: E402
from ddbench.instance import load_instance  # noqa: E402


CARRIER_VACANCY_FIELDS = {
    "vacancy_potential_builds",
    "vacancy_potential_time_ms",
    "vacancy_potential_unreachable_cells",
    "clearance_first_choice_fallbacks",
    "guidance_version",
    "epoch_first_transfer_comparisons",
    "epoch_first_transfer_flips",
    "epoch_chain_overlap_samples",
    "epoch_chain_overlap_intersection",
    "epoch_chain_overlap_union",
    "epoch_chain_overlap_pct",
    "upper_epoch_cache_evictions",
}


@unittest.skipUnless(BIN.is_file(), "dd_benchmark not built")
class TestCarrierVacancyTelemetry(unittest.TestCase):
    def test_cli_and_runner_publish_complete_carrier_contract(self):
        self.assertTrue(
            CARRIER_VACANCY_FIELDS.issubset(
                runner.CARRIER_GUIDANCE_METRIC_FIELDS
            )
        )
        self.assertTrue(
            CARRIER_VACANCY_FIELDS.issubset(runner.FIELDS)
        )

        with tempfile.TemporaryDirectory() as tmp:
            row = runner.row_carrier(
                load_instance(CASE),
                CASE,
                CASE.stem,
                "brap_pool",
                Path(tmp),
                5,
                mode="lacam",
                carrier_bin=BIN,
            )

        self.assertEqual(row["success"], 1, row)
        metrics = {
            field: str(row[field])
            for field in CARRIER_VACANCY_FIELDS
        }
        parsed = runner.validate_carrier_guidance_metrics(metrics)

        self.assertEqual(
            parsed["guidance_version"], "TASKBR_VACANCY_V1"
        )
        self.assertGreater(parsed["vacancy_potential_builds"], 0)
        self.assertGreaterEqual(
            parsed["vacancy_potential_time_ms"], 0
        )
        self.assertGreaterEqual(
            parsed["vacancy_potential_unreachable_cells"], 0
        )
        self.assertGreaterEqual(
            parsed["clearance_first_choice_fallbacks"], 0
        )
        self.assertLessEqual(
            parsed["epoch_first_transfer_flips"],
            parsed["epoch_first_transfer_comparisons"],
        )
        self.assertLessEqual(
            parsed["epoch_chain_overlap_intersection"],
            parsed["epoch_chain_overlap_union"],
        )
        self.assertGreaterEqual(parsed["epoch_chain_overlap_pct"], 0)
        self.assertLessEqual(parsed["epoch_chain_overlap_pct"], 100)
        self.assertGreaterEqual(
            parsed["upper_epoch_cache_evictions"], 0
        )

        missing = dict(metrics)
        missing.pop("guidance_version")
        with self.assertRaises(ValueError):
            runner.validate_carrier_guidance_metrics(missing)


if __name__ == "__main__":
    unittest.main()
