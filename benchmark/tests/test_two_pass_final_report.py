import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from generate_two_pass_final_report import generate_report


FIELDS = [
    "instance",
    "family",
    "success",
    "status",
    "executed_makespan",
    "weighted_soc",
    "first_solution_ms",
    "first_solution_makespan",
    "first_solution_soc",
    "solver_runtime_ms",
    "runtime_sec",
    "improvement_attempts",
    "improvement_candidates",
    "improvement_improvements",
    "improvement_exit_reason",
    "reference_checkpoint_hits",
    "reference_action_hints",
    "reference_suffix_attempts",
    "reference_suffix_accepted",
    "plan_sha256",
]


def row(
    name,
    success,
    makespan="",
    work="",
    first_ms="",
    first_makespan="",
    first_work="",
    runtime_ms="",
    exit_reason="",
    suffix_accepted=0,
):
    return {
        "instance": name,
        "family": "fixture",
        "success": int(success),
        "status": "ok" if success else "timeout",
        "executed_makespan": makespan,
        "weighted_soc": work,
        "first_solution_ms": first_ms,
        "first_solution_makespan": first_makespan,
        "first_solution_soc": first_work,
        "solver_runtime_ms": runtime_ms,
        "runtime_sec": (
            float(runtime_ms) / 1000 if runtime_ms != "" else 10.0
        ),
        "improvement_attempts": int(success),
        "improvement_candidates": int(
            exit_reason in {"STRICT_IMPROVEMENT", "REFERENCE_SUFFIX_ACCEPTED"}
        ),
        "improvement_improvements": int(
            exit_reason in {"STRICT_IMPROVEMENT", "REFERENCE_SUFFIX_ACCEPTED"}
        ),
        "improvement_exit_reason": exit_reason,
        "reference_checkpoint_hits": 7 if success else 0,
        "reference_action_hints": 11 if success else 0,
        "reference_suffix_attempts": suffix_accepted,
        "reference_suffix_accepted": suffix_accepted,
        "plan_sha256": (name[0] * 64) if success else "",
    }


class TwoPassFinalReportTest(unittest.TestCase):
    def _write_rows(self, path, rows):
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def _write_timing(self, path, total, solved, binary):
        path.write_text(
            json.dumps(
                {
                    "wall_time_sec": 12.5,
                    "jobs": 2,
                    "n_tasks": total,
                    "timeout_per_run_sec": 10,
                    "solver_seed": 0,
                    "methods": {
                        "carrier": {
                            "solved": solved,
                            "total": total,
                            "solver_time_sum_sec": 24.0,
                        }
                    },
                    "provenance": {
                        "binary_sha256": binary,
                        "execution_snapshot": "sealed_linux_memfd",
                    },
                    "suite": {
                        "definition_sha256": "s" * 64,
                        "tier": "fixture",
                    },
                }
            ),
            encoding="utf-8",
        )

    def test_report_is_data_driven_and_explains_two_pass_semantics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old_quick = [
                row("q1", True, 20, 100, 1000, 20, 100, 8000),
                row("q2", True, 10, 40, 100, 10, 40, 8000),
                row("q3", False),
            ]
            new_quick = [
                row(
                    "q1", True, 18, 110, 900, 20, 100, 2000,
                    "STRICT_IMPROVEMENT",
                ),
                row(
                    "q2", True, 10, 41, 110, 10, 41, 9000,
                    "SEARCH_CUTOFF",
                ),
                row("q3", False),
            ]
            old_full = old_quick + [
                row("f1", True, 6, 15, 50, 6, 15, 7000),
            ]
            new_full = new_quick + [
                row(
                    "f1", True, 6, 14, 50, 6, 15, 500,
                    "REFERENCE_SUFFIX_ACCEPTED", 1,
                ),
            ]
            historical_full = [
                row("q1", True, 18, 110, 950, 18, 110, 500),
                row("q2", True, 9, 50, 90, 9, 50, 600),
                row("q3", False),
                row("f1", True, 7, 13, 45, 7, 13, 400),
            ]

            paths = {}
            for name, rows in (
                ("old_quick", old_quick),
                ("new_quick", new_quick),
                ("old_full", old_full),
                ("new_full", new_full),
                ("historical_full", historical_full),
            ):
                paths[name] = root / f"{name}.csv"
                self._write_rows(paths[name], rows)

            for name, rows in (
                ("old_quick_timing", old_quick),
                ("new_quick_timing", new_quick),
                ("old_full_timing", old_full),
                ("new_full_timing", new_full),
                ("historical_full_timing", historical_full),
            ):
                paths[name] = root / f"{name}.json"
                self._write_timing(
                    paths[name],
                    len(rows),
                    sum(int(record["success"]) for record in rows),
                    "b" * 64 if name.startswith("new") else "a" * 64,
                )

            approval = root / "approval.json"
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
            out = root / "site"
            data = generate_report(
                baseline_quick_rows_path=paths["old_quick"],
                current_quick_rows_path=paths["new_quick"],
                baseline_quick_timing_path=paths["old_quick_timing"],
                current_quick_timing_path=paths["new_quick_timing"],
                baseline_full_rows_path=paths["old_full"],
                current_full_rows_path=paths["new_full"],
                baseline_full_timing_path=paths["old_full_timing"],
                current_full_timing_path=paths["new_full_timing"],
                historical_full_rows_path=paths["historical_full"],
                historical_full_timing_path=(
                    paths["historical_full_timing"]
                ),
                approval_path=approval,
                out_dir=out,
                cpp_tests=296,
                python_tests=157,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )
            main_index = (
                Path(__file__).resolve().parents[1]
                / "viz_web"
                / "index.html"
            ).read_text(encoding="utf-8")

            self.assertEqual(data["full"]["current"]["total"], 4)
            self.assertEqual(data["full"]["current"]["solved"], 3)
            self.assertEqual(
                data["full"]["comparison"]["lexicographic"],
                {"better": 2, "equal": 0, "worse": 1},
            )
            self.assertEqual(
                data["historical_full"]["comparison"]["lexicographic"],
                {"better": 1, "equal": 1, "worse": 1},
            )
            self.assertEqual(
                summary["historical_full"]["baseline"]["solved"], 3
            )
            self.assertEqual(
                summary["full"]["phase2_exit_reasons"],
                {
                    "REFERENCE_SUFFIX_ACCEPTED": 1,
                    "SEARCH_CUTOFF": 1,
                    "STRICT_IMPROVEMENT": 1,
                },
            )
            self.assertEqual(
                summary["full"]["reference"]["suffix_accepted"], 1
            )
            self.assertEqual(
                summary["full"]["suffix_example"]["instance"], "f1"
            )
            self.assertEqual(
                summary["full"]["planning_time"]["paired_count"], 3
            )
            self.assertEqual(
                summary["full"]["planning_time"]["first_ms"]["median"],
                110,
            )
            self.assertEqual(
                summary["full"]["planning_time"]["final_ms"]["median"],
                2000,
            )
            self.assertEqual(
                summary["full"]["planning_time"]["delay_ms"]["median"],
                1100,
            )
            self.assertEqual(
                summary["full"]["planning_time"]["milestones"]["1000"],
                {"first_count": 3, "final_count": 1},
            )

            for text in (
                "两遍规划控制器",
                "第一遍：先拿到可交付首解",
                "第二遍：固定终态，只找第一份严格改进",
                "找不到改进就返回首轮",
                "makespan 优先，work 次优先",
                "共享同一个 10 秒 deadline",
                "参考后缀不是下界",
                "3/4",
                "2/0/1",
                "pre-v5 full",
                "1/1/1",
                "REFERENCE_SUFFIX_ACCEPTED",
                "296 个 C++ 测试",
                "157 个 Python 测试",
                "f1",
                "首解可用时间 vs 最终返回时间",
                "累计案例比例",
                "最终返回 P50",
                "首解到返回 P50",
                'class="timing-chart"',
            ):
                self.assertIn(text, page)
            for evidence in (
                "old_full.csv",
                "old_full_timing.json",
                "new_full.csv",
                "new_full_timing.json",
            ):
                self.assertIn(evidence, page)
            self.assertIn("overflow-x:auto", page)
            self.assertIn("overflow-wrap:anywhere", page)
            self.assertGreater(page.count("<table>"), 0)
            self.assertEqual(
                page.count('<div class="table-wrap"><table>'),
                page.count("<table>"),
            )
            self.assertIn("<td>变差</td>", page)
            self.assertNotIn("<td>回退</td>", page)
            self.assertIn(
                '<meta name="viewport" '
                'content="width=device-width,initial-scale=1">',
                main_index,
            )
            self.assertIn(
                'href="carrier_lacam_two_pass_20260906/index.html"',
                main_index,
            )
            for stale in ("479/509", "47/77", "344 个"):
                self.assertNotIn(stale, page)

    def test_cli_requires_explicit_test_counts_and_approval(self):
        script = (
            Path(__file__).resolve().parent.parent
            / "generate_two_pass_final_report.py"
        )
        result = subprocess.run(
            [sys.executable, str(script)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("--cpp-tests", result.stderr)
        self.assertIn("--python-tests", result.stderr)
        self.assertIn("--approval", result.stderr)
        self.assertIn("--historical-full-rows", result.stderr)
        self.assertIn("--historical-full-timing", result.stderr)


if __name__ == "__main__":
    unittest.main()
