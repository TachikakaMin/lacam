"""Development quick-suite and post-review full-suite contracts."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from run_benchmark import (
    BENCHMARK_TIERS,
    assert_approved_binary_unchanged,
    full_corpus_sha256,
    resolve_benchmark_tier,
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
                    "expected_cases": 509,
                    "requires_review": True,
                },
            },
        )

    def test_full_manifest_is_quick_plus_432_new_cases(self):
        definition = json.loads(
            (BENCH / "full_benchmark.json").read_text(encoding="utf-8")
        )
        self.assertEqual(definition["name"], "carrier_full_509")
        self.assertEqual(
            {
                group["name"]: group["expected_cases"]
                for group in definition["groups"]
            },
            {
                "brap_pool": 68,
                "warehouse_blocks": 9,
                "warehouse_random_factorial": 432,
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


if __name__ == "__main__":
    unittest.main()
