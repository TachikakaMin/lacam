"""Development quick-suite and post-review full-suite contracts."""

import hashlib
import json
import tempfile
import unittest
from unittest import mock
from pathlib import Path

from run_benchmark import (
    BENCHMARK_TIERS,
    assert_approved_binary_unchanged,
    full_corpus_sha256,
    resolve_benchmark_tier,
    validate_full_review_approval,
)


BENCH = Path(__file__).resolve().parent.parent


class TestBenchmarkTiers(unittest.TestCase):
    def test_quick_and_full_tiers_are_fixed(self):
        self.assertEqual(
            BENCHMARK_TIERS,
            {
                "quick": {
                    "suite": BENCH / "release_benchmark.json",
                    "expected_cases": 77,
                    "requires_review": False,
                },
                "full": {
                    "suite": BENCH / "full_benchmark.json",
                    "expected_cases": 518,
                    "requires_review": True,
                },
            },
        )

    def test_full_manifest_is_quick_plus_441_new_cases(self):
        definition = json.loads(
            (BENCH / "full_benchmark.json").read_text(encoding="utf-8")
        )
        self.assertEqual(definition["name"], "carrier_full_518")
        self.assertEqual(
            {
                group["name"]: group["expected_cases"]
                for group in definition["groups"]
            },
            {
                "brap_pool": 68,
                "warehouse_blocks": 9,
                "warehouse_random_factorial": 432,
                "dense_channel_block_edge_40x40_v1": 9,
            },
        )
        self.assertEqual(
            definition["protocol"],
            json.loads(
                (BENCH / "release_benchmark.json").read_text(
                    encoding="utf-8"
                )
            )["protocol"],
        )

    def test_quick_needs_no_review_but_full_does(self):
        suite = resolve_benchmark_tier("quick")
        self.assertEqual(suite, BENCH / "release_benchmark.json")

        with self.assertRaisesRegex(
            ValueError, "independent review approval"
        ):
            resolve_benchmark_tier("full")

    def test_full_review_approval_is_bound_to_suite_definition(self):
        suite = BENCH / "full_benchmark.json"
        digest = hashlib.sha256(suite.read_bytes()).hexdigest()
        corpus_digest = full_corpus_sha256()
        with tempfile.TemporaryDirectory() as tmp:
            binary_a = Path(tmp) / "carrier-a"
            binary_b = Path(tmp) / "carrier-b"
            binary_a.write_bytes(b"carrier binary A")
            binary_b.write_bytes(b"carrier binary B")
            binary_digest = hashlib.sha256(
                binary_a.read_bytes()
            ).hexdigest()
            approval = Path(tmp) / "approval.json"
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
                        "suite_definition_sha256": digest,
                        "full_corpus_sha256": corpus_digest,
                        "binary_sha256": binary_digest,
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                ),
                suite,
            )
            approved_payload = json.loads(
                approval.read_text(encoding="utf-8")
            )
            binary_a.write_bytes(b"carrier binary A replaced")
            with self.assertRaisesRegex(ValueError, "binary hash"):
                assert_approved_binary_unchanged(
                    approved_payload, binary_a
                )
            binary_a.write_bytes(b"carrier binary A")

            stale = json.loads(approval.read_text(encoding="utf-8"))
            stale["suite_definition_sha256"] = "0" * 64
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "suite hash"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                )

            stale["suite_definition_sha256"] = digest
            stale["full_corpus_sha256"] = "0" * 64
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "corpus hash"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                )

            stale["full_corpus_sha256"] = corpus_digest
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "binary hash"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_b
                )

            del stale["binary_sha256"]
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "binary_sha256"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                )

            stale["binary_sha256"] = "not-a-sha256"
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "binary_sha256"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                )

            stale["binary_sha256"] = binary_digest
            stale["schema_version"] = 1
            approval.write_text(json.dumps(stale), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "schema_version"):
                resolve_benchmark_tier(
                    "full", approval, carrier_bin=binary_a
                )

    def test_full_review_approval_requires_complete_review_metadata(self):
        suite = BENCH / "full_benchmark.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root / "carrier"
            binary.write_bytes(b"approved carrier bytes")
            approval_path = root / "approval.json"
            valid = {
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
                "full_corpus_sha256": "a" * 64,
                "binary_sha256": hashlib.sha256(
                    binary.read_bytes()
                ).hexdigest(),
            }

            invalid_metadata = [
                ("reasoning_effort", None, "reasoning_effort"),
                ("reasoning_effort", "medium", "reasoning_effort"),
                ("reviewer_agent_id", "   ", "reviewer_agent_id"),
                ("reviewed_at_utc", "2026-09-07", "reviewed_at_utc"),
                (
                    "reviewed_at_utc",
                    "2026-09-07T00:00:00+00:00",
                    "reviewed_at_utc",
                ),
                ("review_summary", "", "review_summary"),
                (
                    "blocking_findings",
                    ["unresolved correctness issue"],
                    "blocking_findings",
                ),
                (
                    "blocking_findings",
                    "none",
                    "blocking_findings",
                ),
            ]
            with mock.patch(
                "run_benchmark.full_corpus_sha256",
                return_value="a" * 64,
            ):
                for field, value, message in invalid_metadata:
                    with self.subTest(field=field, value=value):
                        payload = dict(valid)
                        if value is None:
                            del payload[field]
                        else:
                            payload[field] = value
                        approval_path.write_text(
                            json.dumps(payload), encoding="utf-8"
                        )
                        with self.assertRaisesRegex(ValueError, message):
                            validate_full_review_approval(
                                approval_path, carrier_bin=binary
                            )


if __name__ == "__main__":
    unittest.main()
