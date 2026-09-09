"""The protected full-suite review gate cannot be bypassed by path."""

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from run_benchmark import (
    assert_approved_binary_unchanged,
    corpus_sha256_for_cases,
    create_approved_execution_snapshot,
    full_corpus_sha256,
    guard_full_only_cases,
    guard_protected_suite,
    provenance_info,
    semantic_case_fingerprint,
)


BENCH = Path(__file__).resolve().parent.parent


class TestFullBenchmarkReviewGate(unittest.TestCase):
    def test_direct_full_suite_config_still_requires_review(self):
        with self.assertRaisesRegex(
            ValueError, "independent review approval"
        ):
            guard_protected_suite(BENCH / "full_benchmark.json")

    def test_quick_suite_config_does_not_require_review(self):
        self.assertEqual(
            guard_protected_suite(BENCH / "release_benchmark.json"),
            BENCH / "release_benchmark.json",
        )

    def test_dense_channel_case_cannot_bypass_full_review_by_path(self):
        case = next(
            (
                BENCH
                / "viz_web"
                / "dense_channel_block_edge_40x40_suite_v1_20260907"
                / "instances"
            ).glob("*.yaml")
        )
        with self.assertRaisesRegex(
            ValueError, "independent review approval"
        ):
            guard_full_only_cases(
                [(case, "dense_channel_block_edge_40x40")]
            )

    def test_custom_carrier_binary_must_match_approved_bytes(self):
        suite = BENCH / "full_benchmark.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary_a = root / "carrier-a"
            binary_b = root / "carrier-b"
            binary_a.write_bytes(b"approved carrier bytes")
            binary_b.write_bytes(b"different carrier bytes")
            binary_b.chmod(0o755)
            approval = root / "approval.json"
            approval.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "decision": "APPROVE",
                        "reviewer_model": "openai.gpt-5.6-sol",
                        "reasoning_effort": "high",
                        "reviewer_agent_id": "test-reviewer-agent",
                        "reviewed_at_utc": "2026-09-07T00:00:00Z",
                        "review_summary": "Fixture approval.",
                        "blocking_findings": [],
                        "suite_definition_sha256": hashlib.sha256(
                            suite.read_bytes()
                        ).hexdigest(),
                        "full_corpus_sha256": full_corpus_sha256(),
                        "binary_sha256": hashlib.sha256(
                            binary_a.read_bytes()
                        ).hexdigest(),
                    }
                ),
                encoding="utf-8",
            )
            proc = subprocess.run(
                [
                    sys.executable,
                    str(BENCH / "run_benchmark.py"),
                    "--benchmark-tier",
                    "full",
                    "--review-approval",
                    str(approval),
                    "--carrier-bin",
                    str(binary_b),
                    "--out-dir",
                    str(root / "must-not-run"),
                ],
                capture_output=True,
                text=True,
                timeout=60,
            )
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("binary hash", proc.stderr)
            self.assertFalse((root / "must-not-run").exists())

    def test_approved_execution_uses_sealed_binary_and_yaml_snapshots(self):
        source_case = next(
            (
                BENCH
                / "viz_web"
                / "warehouse_case_proposal"
                / "factorial_suite"
                / "instances"
            ).glob("*.yaml")
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case = root / "case.yaml"
            case.write_bytes(source_case.read_bytes())
            binary = root / "carrier"
            binary.write_bytes(
                b"#!/bin/sh\nprintf 'approved-bytes\\n'\n"
            )
            binary.chmod(0o755)
            cases = [(case, "protected")]
            approval = {
                "binary_sha256": hashlib.sha256(
                    binary.read_bytes()
                ).hexdigest(),
                "full_corpus_sha256": corpus_sha256_for_cases(cases),
            }

            with create_approved_execution_snapshot(
                cases, binary, approval
            ) as snapshot:
                snap_case, family, original_name = snapshot.cases[0]
                self.assertEqual(family, "protected")
                self.assertEqual(original_name, "case")
                expected_fingerprint = semantic_case_fingerprint(snap_case)

                case.write_text("not: the approved testcase\n")
                binary.write_bytes(
                    b"#!/bin/sh\nprintf 'replacement-bytes\\n'\n"
                )

                self.assertEqual(
                    semantic_case_fingerprint(snap_case),
                    expected_fingerprint,
                )
                proc = subprocess.run(
                    [str(snapshot.binary_path)],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                self.assertEqual(proc.returncode, 0)
                self.assertEqual(proc.stdout, "approved-bytes\n")
                with self.assertRaises(PermissionError):
                    Path(snapshot.binary_path).write_bytes(b"mutate")
                with self.assertRaises(PermissionError):
                    Path(snap_case).write_bytes(b"mutate")

    def test_sealed_binary_can_be_rehashed_before_publication(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root / "carrier"
            binary.write_bytes(
                b"#!/bin/sh\nprintf 'approved-bytes\\n'\n"
            )
            binary.chmod(0o755)
            approval = {
                "binary_sha256": hashlib.sha256(
                    binary.read_bytes()
                ).hexdigest(),
                "full_corpus_sha256": corpus_sha256_for_cases([]),
            }

            with create_approved_execution_snapshot(
                [], binary, approval
            ) as snapshot:
                self.assertEqual(
                    assert_approved_binary_unchanged(
                        approval, snapshot.binary_path
                    ),
                    approval["binary_sha256"],
                )
                self.assertEqual(
                    provenance_info(snapshot.binary_path)["binary_sha256"],
                    approval["binary_sha256"],
                )


if __name__ == "__main__":
    unittest.main()
