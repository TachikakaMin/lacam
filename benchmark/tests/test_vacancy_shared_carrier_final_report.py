import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_vacancy_shared_carrier_final_report import generate_report


FIELDS = [
    "instance",
    "family",
    "method",
    "success",
    "executed_makespan",
    "weighted_soc",
    "plan_sha256",
    "status",
    "brd_exit_reason",
    "vacancy_potential_builds",
    "epoch_first_transfer_flips",
    "upper_epoch_cache_evictions",
]


def row(
    name,
    family,
    method,
    success,
    makespan="",
    soc="",
    plan_hash="",
    status=None,
    exit_reason="",
):
    return {
        "instance": name,
        "family": family,
        "method": method,
        "success": int(success),
        "executed_makespan": makespan,
        "weighted_soc": soc,
        "plan_sha256": plan_hash,
        "status": status or ("ok" if success else "timeout"),
        "brd_exit_reason": exit_reason,
        "vacancy_potential_builds": 2 if method == "carrier" else "",
        "epoch_first_transfer_flips": 1 if method == "carrier" else "",
        "upper_epoch_cache_evictions": 0 if method == "carrier" else "",
    }


class VacancySharedCarrierFinalReportTest(unittest.TestCase):
    def _write_rows(self, path, rows):
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def _write_timing(self, path, method, solved, total, wall, binary):
        path.write_text(
            json.dumps(
                {
                    "wall_time_sec": wall,
                    "jobs": 14,
                    "n_tasks": total,
                    "timeout_per_run_sec": 10,
                    "methods": {
                        method: {
                            "solved": solved,
                            "total": total,
                            "solver_time_sum_sec": wall * 2,
                        }
                    },
                    "provenance": {"binary_sha256": binary},
                    "suite": {
                        "definition_sha256": "s" * 64,
                        "tier": "full",
                    },
                }
            ),
            encoding="utf-8",
        )

    def test_report_is_data_driven_and_keeps_full_negative_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_full = root / "baseline-full.csv"
            current_full = root / "current-full.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            phase_quick = root / "phase-quick.csv"
            current_quick = root / "current-quick.csv"
            phase_brd = root / "phase-brd.csv"
            current_brd = root / "current-brd.csv"
            current_brd_timing = root / "current-brd-timing.json"
            manifest = root / "manifest.json"
            approval = root / "approval.json"
            out = root / "site"

            self._write_rows(
                baseline_full,
                [
                    row("q-a", "quick", "carrier", True, 10, 20, "a" * 64),
                    row("q-b", "quick", "carrier", True, 12, 24, "b" * 64),
                    row("f-a", "warehouse_random_factorial", "carrier",
                        True, 10, 20, "c" * 64),
                    row("f-b", "warehouse_random_factorial", "carrier",
                        True, 10, 20, "d" * 64),
                    row("f-c", "warehouse_random_factorial", "carrier", False),
                ],
            )
            self._write_rows(
                current_full,
                [
                    row("q-a", "quick", "carrier", True, 8, 18, "e" * 64),
                    row("q-b", "quick", "carrier", False),
                    row("f-a", "warehouse_random_factorial", "carrier",
                        True, 12, 22, "f" * 64),
                    row("f-b", "warehouse_random_factorial", "carrier",
                        True, 10, 20, "d" * 64),
                    row("f-c", "warehouse_random_factorial", "carrier", False),
                ],
            )
            self._write_timing(
                baseline_timing, "carrier", 4, 5, 20.0, "a" * 64
            )
            self._write_timing(
                current_timing, "carrier", 3, 5, 18.0, "b" * 64
            )
            self._write_rows(
                phase_quick,
                [
                    row("q-a", "quick", "carrier", True, 12, 24, "g" * 64),
                    row("q-b", "quick", "carrier", True, 10, 20, "h" * 64),
                    row("q-c", "quick", "carrier", False),
                ],
            )
            self._write_rows(
                current_quick,
                [
                    row("q-a", "quick", "carrier", True, 8, 18, "e" * 64),
                    row("q-b", "quick", "carrier", True, 10, 22, "i" * 64),
                    row("q-c", "quick", "carrier", False),
                ],
            )
            self._write_rows(
                phase_brd,
                [
                    row("b-a", "factorial", "carrier_brd",
                        True, 5, 9, "j" * 64),
                    row("b-b", "factorial", "carrier_brd", False,
                        exit_reason="SEGMENT_TIMEOUT"),
                ],
            )
            self._write_rows(
                current_brd,
                [
                    row("b-a", "factorial", "carrier_brd",
                        True, 5, 9, "j" * 64),
                    row("b-b", "factorial", "carrier_brd", False,
                        exit_reason="SEARCH_TIMEOUT"),
                ],
            )
            self._write_timing(
                current_brd_timing, "carrier_brd", 1, 2, 4.0, "b" * 64
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "f-a",
                            "map_family": "g1",
                            "density_level": "high",
                            "agent_level": "scarce",
                            "task_profile": "cross_heavy",
                            "goal_mode": "shared_pool",
                        },
                        {
                            "id": "f-b",
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "equal",
                            "task_profile": "local",
                            "goal_mode": "singleton",
                        },
                    ]
                ),
                encoding="utf-8",
            )
            approval.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "decision": "APPROVE",
                        "reviewer_model": "openai.gpt-5.6-sol",
                        "reasoning_effort": "high",
                        "reviewer_agent_id": "reviewer-1",
                        "reviewed_at_utc": "2026-09-07T03:36:40Z",
                        "review_summary": "Fixture approval.",
                        "blocking_findings": [],
                        "suite_definition_sha256": "s" * 64,
                        "full_corpus_sha256": "c" * 64,
                        "binary_sha256": "b" * 64,
                    }
                ),
                encoding="utf-8",
            )

            data = generate_report(
                baseline_full_rows=baseline_full,
                baseline_full_timing=baseline_timing,
                current_full_rows=current_full,
                current_full_timing=current_timing,
                phase_a_quick_rows=phase_quick,
                current_quick_rows=current_quick,
                phase_a_brd_full_rows=phase_brd,
                current_brd_full_rows=current_brd,
                current_brd_full_timing=current_brd_timing,
                manifest_path=manifest,
                approval_path=approval,
                out_dir=out,
                cpp_tests=408,
                python_tests=192,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["full"]["baseline_solved"], 4)
        self.assertEqual(data["full"]["current_solved"], 3)
        self.assertEqual(
            data["full"]["lexicographic"],
            {"better": 1, "equal": 1, "worse": 1},
        )
        self.assertEqual(data["full"]["lost"], ["q-b"])
        self.assertEqual(summary["quick"]["soc_regressions_over_5pct"], 1)
        self.assertEqual(summary["brd"]["successful_semantic_differences"], 0)
        self.assertEqual(summary["brd"]["failure_reason_changes"], 1)
        self.assertIn("1 个新 timeout", page)
        self.assertIn("full 的 1 个丢解", page)
        self.assertNotIn("4 个新 timeout", page)
        for text in (
            "rho V2 4/5 → 当前 3/5",
            "求解率（首要指标）",
            "1 / 1 / 1",
            "q-b",
            "22 → 18",
            "44 → 40",
            "1 个 SOC 超过 5%",
            "成功计划零差异",
            "408 / 408",
            "192 / 192",
            "APPROVE",
            "full_benchmark_vacancy_shared_carrier_20260907",
            "full_comparison_rho_v2_vs_vacancy_shared_carrier_20260907",
            "full_review_approval_vacancy_shared_carrier_20260907.json",
        ):
            self.assertIn(text, page)
        self.assertNotIn("475/509", page)
        self.assertNotIn("141 / 201 / 133", page)
        self.assertIn(
            '<meta name="viewport" '
            'content="width=device-width,initial-scale=1">',
            page,
        )


if __name__ == "__main__":
    unittest.main()
