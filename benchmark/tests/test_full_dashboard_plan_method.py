import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from generate_full_benchmark_dashboard import generate_case_animations


class FullDashboardPlanMethodTest(unittest.TestCase):
    def test_animation_uses_the_timing_method_in_the_plan_filename(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows_path = root / "run" / "rows.csv"
            rows_path.parent.mkdir()
            rows_path.write_text("", encoding="utf-8")
            output = root / "site"
            data = {
                "timing": {"method": "carrier_brd"},
                "all_cases": [
                    {
                        "instance": "case-a",
                        "success": True,
                    }
                ],
            }
            seen = []

            def fake_generate(case, instance_path, plan_path, output_path):
                seen.append(Path(plan_path).name)
                return Path(output_path)

            with patch(
                "generate_full_benchmark_dashboard._full_instance_map",
                return_value={"case-a": root / "case-a.yaml"},
            ):
                with patch(
                    "generate_full_benchmark_dashboard."
                    "generate_case_animation",
                    side_effect=fake_generate,
                ):
                    generated = generate_case_animations(
                        data, rows_path, output, jobs=1
                    )

        self.assertEqual(seen, ["case-a.carrier_brd.plan"])
        self.assertEqual(generated, ["case-a"])


if __name__ == "__main__":
    unittest.main()
