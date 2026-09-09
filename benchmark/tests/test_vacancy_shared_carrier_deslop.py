import tempfile
import unittest
from pathlib import Path

from generate_vacancy_shared_carrier_final_report import generate_report


BENCH = Path(__file__).resolve().parent.parent


class VacancySharedCarrierDeslopTest(unittest.TestCase):
    def test_final_report_uses_plain_hierarchy_instead_of_card_template(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "report"
            generate_report(
                baseline_full_rows=(
                    BENCH / "results_full_rho_v2_20260906" / "rows.csv"
                ),
                baseline_full_timing=(
                    BENCH / "results_full_rho_v2_20260906" / "timing.json"
                ),
                current_full_rows=(
                    BENCH
                    / "results_full_vacancy_shared_carrier_20260907"
                    / "rows.csv"
                ),
                current_full_timing=(
                    BENCH
                    / "results_full_vacancy_shared_carrier_20260907"
                    / "timing.json"
                ),
                phase_a_quick_rows=(
                    BENCH
                    / "results_quick_vacancy_phase_a_carrier_20260906"
                    / "rows.csv"
                ),
                current_quick_rows=(
                    BENCH
                    / "results_quick_vacancy_shared_carrier_20260907_r5"
                    / "rows.csv"
                ),
                phase_a_brd_full_rows=(
                    BENCH
                    / "results_full_vacancy_phase_a_20260906"
                    / "rows.csv"
                ),
                current_brd_full_rows=(
                    BENCH
                    / "results_full_vacancy_shared_carrier_brd_20260907"
                    / "rows.csv"
                ),
                current_brd_full_timing=(
                    BENCH
                    / "results_full_vacancy_shared_carrier_brd_20260907"
                    / "timing.json"
                ),
                manifest_path=(
                    BENCH
                    / "viz_web"
                    / "warehouse_case_proposal"
                    / "factorial_suite"
                    / "manifest.json"
                ),
                approval_path=(
                    BENCH
                    / "full_review_approval_vacancy_shared_carrier_20260907.json"
                ),
                out_dir=out,
                cpp_tests=408,
                python_tests=195,
            )
            page = (out / "index.html").read_text(encoding="utf-8")

        self.assertIn("<h1>Full 少解 4 例</h1>", page)
        for class_name in (
            "verdict",
            "measurements",
            "mechanism-list",
            "audit-list",
        ):
            self.assertIn('class="{}"'.format(class_name), page)
        for stale in (
            'class="cards"',
            'class="card"',
            'class="panel"',
            'class="flow"',
            'class="step"',
            'class="notice"',
            "--good",
            "--bad",
            "#806b42",
            "#292417",
            "局部目标达成了",
            "因此结论不是",
        ):
            self.assertNotIn(stale, page)

    def test_root_index_separates_current_results_from_history(self):
        page = (
            BENCH / "viz_web" / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn("<h2>当前结果</h2>", page)
        self.assertIn("<summary>基线与专题结果</summary>", page)
        self.assertIn("<summary>历史版本与实验</summary>", page)
        self.assertNotIn("最新：", page)
        self.assertLess(
            page.index(
                "vacancy_shared_carrier_final_report_20260907/index.html"
            ),
            page.index(
                "historical_pre_v5_full_benchmark_509_20260905/"
                "index.html"
            ),
        )
        for text in ("历史 pre-v5", "旧二进制", "不作为 v5 完成证据"):
            self.assertIn(text, page)


if __name__ == "__main__":
    unittest.main()
