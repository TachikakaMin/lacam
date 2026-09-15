import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "generate_current_carrier_vs_brd_like_original.py"
)
SPEC = importlib.util.spec_from_file_location(
    "current_carrier_vs_brd_report", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CurrentCarrierVsBrdReportSemanticsTest(unittest.TestCase):
    def test_axis_direction_and_time_ratio_are_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / "index.html").write_text(
                "<div id=\"legend\">对角线下方 = carrier 数值更小。</div>",
                encoding="utf-8",
            )
            (out / "index_en.html").write_text(
                "<div id=\"legend\">Below the diagonal = "
                "a lower Carrier value.</div>",
                encoding="utf-8",
            )

            MODULE.update_comparison_semantics(out, 0.473696)

            zh = (out / "index.html").read_text(encoding="utf-8")
            en = (out / "index_en.html").read_text(encoding="utf-8")
            self.assertIn(
                "对角线上方 = carrier 数值更小",
                zh,
            )
            self.assertIn(
                "Carrier / baseline = 0.474，小于 1 表示 carrier 更快",
                zh,
            )
            self.assertIn(
                "Above the diagonal = a lower Carrier value",
                en,
            )
            self.assertIn(
                "Carrier / baseline = 0.474; values below 1 mean "
                "Carrier is faster",
                en,
            )


if __name__ == "__main__":
    unittest.main()
