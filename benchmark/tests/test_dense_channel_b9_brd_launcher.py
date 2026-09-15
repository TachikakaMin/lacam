"""Protected preflight for the dense-b9 carrier_brd benchmark launcher."""

import importlib.util
import os
import tempfile
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
SCRIPT = BENCH / "run_dense_channel_b9_brd_benchmark.py"
SPEC = importlib.util.spec_from_file_location("dense_b9_brd_launcher", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DenseChannelB9BrdLauncherTest(unittest.TestCase):
    def test_launcher_freezes_reviewed_binary_and_suite(self):
        self.assertEqual(
            MODULE.FROZEN_BINARY_SHA256,
            "d7d76608ba49716f6d07e1b841b8aa729ea73666e6611c68885ee5b949dbe49f",
        )
        self.assertEqual(
            MODULE.SUITE_CONFIG,
            BENCH / "dense_channel_b9_agent_task_brd_benchmark_v1.json",
        )

    def test_wrong_binary_is_rejected_before_output_directory_creation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root / "dd_benchmark"
            binary.write_bytes(b"not the reviewed binary")
            os.chmod(str(binary), 0o700)
            output = root / "results"
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                MODULE.run_benchmark(binary, output)
            self.assertFalse(output.exists())

    def test_runner_command_uses_only_fixed_custom_suite_and_snapshot(self):
        snapshot = Path("/tmp/reviewed-dd-benchmark")
        output = Path("/tmp/dense-b9-brd-results")
        command = MODULE.build_runner_command(snapshot, output)
        self.assertIn("--suite-config", command)
        self.assertEqual(
            command[command.index("--suite-config") + 1],
            str(MODULE.SUITE_CONFIG),
        )
        self.assertEqual(
            command[command.index("--carrier-bin") + 1], str(snapshot)
        )
        self.assertEqual(command[command.index("--out-dir") + 1], str(output))
        self.assertNotIn("--benchmark-tier", command)
        self.assertNotIn("--methods", command)
        self.assertNotIn("--timeout", command)
        self.assertNotIn("--jobs", command)

    def test_design_records_same_binary_method_only_comparison(self):
        design = (BENCH / "dense_channel_b9_agent_task_matrix_v1.md").read_text(
            encoding="utf-8"
        )
        self.assertIn("build-release/dd_benchmark", design)
        self.assertIn(MODULE.FROZEN_BINARY_SHA256, design)
        self.assertNotIn("build-complete-replan/dd_benchmark", design)


if __name__ == "__main__":
    unittest.main()
