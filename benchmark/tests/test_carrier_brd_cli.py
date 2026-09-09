"""PROTECTED P8 contract for the Carrier BR-LaCAM benchmark path.

Written before the CLI/runner implementation.  The C++ driver must expose
``brd`` explicitly, and the Python runner must reject missing or malformed
BRD telemetry instead of silently producing partially populated rows.
"""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build" / "dd_benchmark"
sys.path.insert(0, str(BENCH))

import run_benchmark as runner  # noqa: E402
from ddbench.instance import load_instance  # noqa: E402


BRD_FIELDS = {
    "brd_tau_ms",
    "brd_upper_ms",
    "brd_upper_nodes",
    "brd_upper_constraints",
    "brd_upper_steps",
    "brd_upper_transfers",
    "brd_task_compile_ms",
    "brd_waves",
    "brd_tasks",
    "brd_max_wave_width",
    "brd_target_tasks",
    "brd_anon_tasks",
    "brd_match_calls",
    "brd_match_ms",
    "brd_match_max_cardinality_ms",
    "brd_match_max_cardinality_cutoffs",
    "brd_match_max_rows",
    "brd_match_rows_without_finite_real_edge",
    "brd_match_maximum_real_cardinality",
    "brd_match_real_assignments",
    "brd_match_hall_deficient_calls",
    "brd_dispatch_epochs",
    "brd_completion_events",
    "brd_tasks_completed_per_event",
    "brd_locked_carriers_max",
    "brd_provisional_reassignments",
    "brd_segments",
    "brd_segment_ms",
    "brd_segment_nodes",
    "brd_segment_failures",
    "brd_cleanup_ms",
    "brd_replay_ms",
    "brd_raw_ticks",
    "brd_raw_work_scaled",
    "brd_goal_prefix_removed",
    "brd_incidental_goal_prefix_ms",
    "brd_exit_reason",
}


def parse_metrics(stdout):
    metrics = {}
    for line in stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metrics[key.strip()] = value.strip()
    return metrics


def run_brd(instance, plan_out):
    return subprocess.run(
        [str(BIN), str(instance), "5", str(plan_out), "0", "brd"],
        capture_output=True,
        text=True,
        timeout=30,
    )


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestCarrierBRDCli(unittest.TestCase):
    def test_brd_mode_solves_and_emits_complete_telemetry(self):
        with tempfile.TemporaryDirectory() as tmp:
            plan_out = Path(tmp) / "brd.plan"
            proc = run_brd(
                REPO / "tests" / "fixtures" / "dd_tiny.yaml",
                plan_out,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            metrics = parse_metrics(proc.stdout)
            self.assertEqual(metrics.get("mode"), "brd")
            self.assertEqual(metrics.get("solved"), "1", proc.stdout)
            self.assertTrue(
                BRD_FIELDS.issubset(metrics),
                sorted(BRD_FIELDS - set(metrics)),
            )
            self.assertEqual(metrics["brd_exit_reason"], "SOLVED")
            runner.validate_carrier_brd_metrics(metrics)

    def test_runner_rejects_missing_or_malformed_brd_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = run_brd(
                REPO / "tests" / "fixtures" / "dd_tiny.yaml",
                Path(tmp) / "brd.plan",
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            metrics = parse_metrics(proc.stdout)

            missing = dict(metrics)
            missing.pop("brd_dispatch_epochs", None)
            with self.assertRaises(ValueError):
                runner.validate_carrier_brd_metrics(missing)

            malformed = dict(metrics)
            malformed["brd_raw_ticks"] = "-2"
            with self.assertRaises(ValueError):
                runner.validate_carrier_brd_metrics(malformed)

    def test_runner_maps_carrier_brd_to_cpp_brd_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            instance_path = (
                REPO / "tests" / "fixtures" / "dd_tiny.yaml"
            )
            row = runner.row_carrier(
                load_instance(instance_path),
                instance_path,
                "dd_tiny",
                "probe",
                tmp,
                5,
                mode="brd",
                carrier_bin=BIN,
            )
            self.assertEqual(row["method"], "carrier_brd")
            self.assertEqual(row["success"], 1, row)
            self.assertEqual(row["brd_exit_reason"], "SOLVED")
            self.assertGreaterEqual(int(row["brd_dispatch_epochs"]), 1)


if __name__ == "__main__":
    unittest.main()
