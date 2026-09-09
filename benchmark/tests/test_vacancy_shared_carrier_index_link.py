import unittest
from pathlib import Path


class VacancySharedCarrierIndexLinkTest(unittest.TestCase):
    def test_main_index_links_the_final_vacancy_artifacts(self):
        index = (
            Path(__file__).resolve().parents[1]
            / "viz_web"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'href="vacancy_shared_carrier_final_report_20260907/'
            'index.html"',
            index,
        )
        self.assertIn(
            'href="full_benchmark_vacancy_shared_carrier_20260907/'
            'index.html"',
            index,
        )
        self.assertIn(
            'href="full_comparison_rho_v2_vs_vacancy_shared_carrier_'
            '20260907/index.html"',
            index,
        )


if __name__ == "__main__":
    unittest.main()
