import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_carrier_brd_final_report import generate_report


FIELDS = [
    "instance",
    "family",
    "method",
    "success",
    "executed_makespan",
    "weighted_soc",
    "runtime_sec",
    "status",
    "plan_sha256",
    "brd_tau_ms",
    "brd_upper_ms",
    "brd_task_compile_ms",
    "brd_match_ms",
    "brd_segment_ms",
    "brd_cleanup_ms",
    "brd_replay_ms",
    "brd_incidental_goal_prefix_ms",
    "brd_exit_reason",
    "brd_match_calls",
    "brd_dispatch_epochs",
    "brd_completion_events",
    "brd_segments",
    "brd_provisional_reassignments",
    "brd_locked_carriers_max",
    "brd_raw_ticks",
    "brd_goal_prefix_removed",
]


def row(
    name,
    method,
    success,
    makespan="",
    work="",
    runtime=1.0,
    exit_reason="",
    raw_ticks="",
    removed="",
):
    return {
        "instance": name,
        "family": "fixture",
        "method": method,
        "success": int(success),
        "executed_makespan": makespan,
        "weighted_soc": work,
        "runtime_sec": runtime,
        "status": "ok" if success else "timeout",
        "plan_sha256": (name * 64)[:64] if success else "",
        "brd_tau_ms": 2 if method == "carrier_brd" else "",
        "brd_upper_ms": 3 if method == "carrier_brd" else "",
        "brd_task_compile_ms": 1 if method == "carrier_brd" else "",
        "brd_match_ms": 4 if method == "carrier_brd" else "",
        "brd_segment_ms": 5 if method == "carrier_brd" else "",
        "brd_cleanup_ms": 1 if method == "carrier_brd" else "",
        "brd_replay_ms": 1 if method == "carrier_brd" else "",
        "brd_incidental_goal_prefix_ms": (
            2 if method == "carrier_brd" and success else -1
        ),
        "brd_exit_reason": exit_reason,
        "brd_match_calls": 3 if method == "carrier_brd" else "",
        "brd_dispatch_epochs": 3 if method == "carrier_brd" else "",
        "brd_completion_events": 2 if method == "carrier_brd" else "",
        "brd_segments": 3 if method == "carrier_brd" else "",
        "brd_provisional_reassignments": (
            1 if method == "carrier_brd" else ""
        ),
        "brd_locked_carriers_max": 1 if method == "carrier_brd" else "",
        "brd_raw_ticks": raw_ticks,
        "brd_goal_prefix_removed": removed,
    }


class CarrierBrdFinalReportTest(unittest.TestCase):
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
                    "jobs": 2,
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

    def test_report_is_data_driven_and_explains_event_rematching(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_rows = root / "baseline.csv"
            current_rows = root / "current.csv"
            historical_rows = root / "historical.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            manifest = root / "manifest.json"
            approval = root / "approval.json"
            out = root / "site"

            self._write_rows(
                baseline_rows,
                [
                    row("a", "carrier", True, 10, 20),
                    row("b", "carrier", True, 8, 18),
                    row("c", "carrier", True, 9, 19),
                    row("d", "carrier", False),
                ],
            )
            self._write_rows(
                current_rows,
                [
                    row(
                        "a", "carrier_brd", True, 9, 21, 0.5,
                        "SOLVED", 12, 3,
                    ),
                    row(
                        "b", "carrier_brd", True, 9, 17, 0.6,
                        "SOLVED", 9, 0,
                    ),
                    row(
                        "c", "carrier_brd", False, runtime=10,
                        exit_reason="SEGMENT_TIMEOUT",
                    ),
                    row(
                        "d", "carrier_brd", False, runtime=10,
                        exit_reason="UPPER_TIMEOUT",
                    ),
                ],
            )
            historical = []
            for method, solved in (
                ("carrier", 2),
                ("carrier_b0", 1),
                ("carrier_b1", 1),
            ):
                historical.extend(
                    [
                        row(
                            "{}-1".format(method),
                            method,
                            solved >= 1,
                            4 if solved >= 1 else "",
                            8 if solved >= 1 else "",
                        ),
                        row(
                            "{}-2".format(method),
                            method,
                            solved >= 2,
                            5 if solved >= 2 else "",
                            9 if solved >= 2 else "",
                        ),
                    ]
                )
            self._write_rows(historical_rows, historical)
            self._write_timing(
                baseline_timing, "carrier", 3, 4, 4.0, "a" * 64
            )
            self._write_timing(
                current_timing, "carrier_brd", 2, 4, 3.0, "b" * 64
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "a",
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "equal",
                            "task_profile": "local",
                            "goal_mode": "shared_pool",
                        },
                        {
                            "id": "b",
                            "map_family": "g1",
                            "density_level": "high",
                            "agent_level": "scarce",
                            "task_profile": "mixed",
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
                        "suite_definition_sha256": "s" * 64,
                        "full_corpus_sha256": "c" * 64,
                        "binary_sha256": "b" * 64,
                    }
                ),
                encoding="utf-8",
            )

            data = generate_report(
                baseline_rows_path=baseline_rows,
                baseline_timing_path=baseline_timing,
                current_rows_path=current_rows,
                current_timing_path=current_timing,
                manifest_path=manifest,
                historical_rows_path=historical_rows,
                approval_path=approval,
                out_dir=out,
                cpp_tests=12,
                python_tests=34,
                compatibility_tests=7,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["comparison"]["baseline_solved"], 3)
        self.assertEqual(data["comparison"]["current_solved"], 2)
        self.assertEqual(
            data["comparison"]["lexicographic"],
            {"better": 1, "equal": 0, "worse": 1},
        )
        self.assertEqual(
            summary["failures"],
            {"SEGMENT_TIMEOUT": 1, "UPPER_TIMEOUT": 1},
        )
        self.assertEqual(summary["raw_vs_deliverable"]["trimmed_cases"], 1)
        self.assertEqual(summary["raw_vs_deliverable"]["ticks_removed"], 3)
        self.assertEqual(summary["historical"]["carrier_b0"]["solved"], 1)
        for text in (
            "tau → upper → waves → matching → lower → return",
            "一次 solve 只到下一个 Drop",
            "所有 free robots × 全部 PENDING tasks",
            "已 Lift 保持 hard lock",
            "R6：未 Lift，立即重匹配",
            "R7：已 Lift，继续锁定",
            "production Carrier 3/4 → Carrier BRD 2/4",
            "1 / 0 / 1",
            "历史对照（不同 corpus）",
            "SEGMENT_TIMEOUT",
            "12 / 12",
            "34 / 34",
            "7 / 7",
            "raw 计划与最终交付计划",
            "baseline.csv",
            "current.csv",
            "approval.json",
        ):
            self.assertIn(text, page)
        self.assertNotIn("326/509", page)
        self.assertIn("overflow-x:auto", page)
        self.assertIn(
            '<meta name="viewport" '
            'content="width=device-width,initial-scale=1">',
            page,
        )


if __name__ == "__main__":
    unittest.main()
