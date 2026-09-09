"""PROTECTED regression for preserving BRD telemetry on failed rows."""

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
    / "g80x80"
    / "brap_h80w80_a128_e1600_B_seed0_pool.yaml"
)
sys.path.insert(0, str(BENCH))

import run_benchmark as runner  # noqa: E402
from ddbench.instance import load_instance  # noqa: E402


@unittest.skipUnless(BIN.is_file(), "dd_benchmark not built")
class TestCarrierBRDFailureTelemetry(unittest.TestCase):
    def test_internal_tau_timeout_keeps_brd_phase_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            row = runner.row_carrier(
                load_instance(CASE),
                CASE,
                CASE.stem,
                "brap_pool",
                Path(tmp),
                1.5,
                mode="brd",
                carrier_bin=BIN,
            )

        self.assertEqual(row["method"], "carrier_brd")
        self.assertEqual(row["success"], 0)
        self.assertEqual(row["status"], "timeout")
        self.assertEqual(row["brd_exit_reason"], "SEARCH_TIMEOUT")
        self.assertGreater(float(row["brd_tau_ms"]), 0)
        self.assertEqual(float(row["brd_upper_ms"]), 0)


if __name__ == "__main__":
    unittest.main()
