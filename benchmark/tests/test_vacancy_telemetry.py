"""PROTECTED vacancy-aware Carrier BRD telemetry contract."""

import subprocess
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


VACANCY_FIELDS = {
    "brd_vacancy_potential_builds",
    "brd_vacancy_potential_time_ms",
    "brd_vacancy_potential_unreachable_cells",
    "brd_selected_clearance_pushes",
    "brd_selected_clearance_loaded_steps",
    "brd_clearance_first_choice_fallbacks",
    "brd_guidance_version",
}


def parse_metrics(stdout):
    metrics = {}
    for line in stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metrics[key.strip()] = value.strip()
    return metrics


@unittest.skipUnless(BIN.is_file(), "dd_benchmark not built")
class TestVacancyTelemetry(unittest.TestCase):
    def test_cli_and_runner_publish_complete_vacancy_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = subprocess.run(
                [
                    str(BIN),
                    str(CASE),
                    "5",
                    str(Path(tmp) / "plan.txt"),
                    "0",
                    "brd",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )

        self.assertEqual(proc.returncode, 0, proc.stderr)
        metrics = parse_metrics(proc.stdout)
        self.assertEqual(metrics.get("solved"), "1", proc.stdout)
        self.assertTrue(
            VACANCY_FIELDS.issubset(metrics),
            sorted(VACANCY_FIELDS - set(metrics)),
        )
        self.assertTrue(
            VACANCY_FIELDS.issubset(runner.BRD_METRIC_FIELDS)
        )
        self.assertEqual(
            metrics["brd_guidance_version"],
            "TASKBR_VACANCY_V1",
        )
        self.assertGreater(
            int(metrics["brd_vacancy_potential_builds"]), 0
        )
        self.assertGreaterEqual(
            float(metrics["brd_vacancy_potential_time_ms"]), 0
        )
        self.assertGreaterEqual(
            int(metrics["brd_vacancy_potential_unreachable_cells"]), 0
        )
        pushes = int(metrics["brd_selected_clearance_pushes"])
        loaded = int(metrics["brd_selected_clearance_loaded_steps"])
        self.assertGreater(pushes, 0)
        self.assertGreaterEqual(loaded, pushes)
        self.assertGreaterEqual(
            int(metrics["brd_clearance_first_choice_fallbacks"]), 0
        )
        runner.validate_carrier_brd_metrics(metrics)

        missing = dict(metrics)
        missing.pop("brd_guidance_version")
        with self.assertRaises(ValueError):
            runner.validate_carrier_brd_metrics(missing)


if __name__ == "__main__":
    unittest.main()
