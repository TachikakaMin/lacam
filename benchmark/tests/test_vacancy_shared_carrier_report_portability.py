import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path

from generate_vacancy_shared_carrier_final_report import generate_report


BENCH = Path(__file__).resolve().parent.parent


class _Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.hrefs = []

    def handle_starttag(self, tag, attrs):
        if tag == "a":
            self.hrefs.extend(
                value for key, value in attrs if key == "href"
            )


class VacancySharedCarrierReportPortabilityTest(unittest.TestCase):
    def test_published_evidence_links_stay_inside_the_static_report(self):
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
                python_tests=197,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            links = _Links()
            links.feed(page)
            evidence = [
                href for href in links.hrefs
                if href.startswith("evidence/")
            ]

            self.assertEqual(len(evidence), 4)
            self.assertNotIn("../../results_", page)
            self.assertNotIn("../../full_review_approval_", page)
            for href in evidence:
                target = (out / href).resolve()
                target.relative_to(out.resolve())
                self.assertTrue(target.is_file(), href)


if __name__ == "__main__":
    unittest.main()
