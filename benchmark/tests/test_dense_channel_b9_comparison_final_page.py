"""Final publication contracts for the dense b9 comparison page."""

import importlib.util
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
SCRIPT = BENCH / "generate_dense_channel_b9_comparison_report.py"
SPEC = importlib.util.spec_from_file_location("dense_b9_comparison_final", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DenseChannelB9ComparisonFinalPageTest(unittest.TestCase):
    def test_tooltip_formatter_is_neutral_inside_side_specific_labels(self):
        source = """<title>carrier vs baseline(carrier_brd) — full 509 首解时间与解质量</title>
<a href="index_en.html">English version</a>
<h3>首解时间（ms，log-log）</h3>
carrier 首解时间 (ms) baseline 首解时间 (ms)
const f = s => s.solved ? `首解 ${s.first} ms，mk ${s.mk}，soc ${s.soc}` : "10 s 内未解出";
`carrier：${f(r.c)}<br>baseline：${f(r.b)}<br>`
<div>对角线下方 = carrier 数值更小。</div>"""
        page = MODULE.specialize_page(source, english=False)
        self.assertIn("carrier（首个全局 goal）", page)
        self.assertIn("baseline（final）", page)
        self.assertIn("`时间 ${s.first} ms，mk ${s.mk}，soc ${s.soc}`", page)
        self.assertNotIn("`首解 ${s.first} ms", page)

    def test_publication_banner_states_full_shared_provenance_and_protocol(self):
        timing = {
            "n_tasks": 24,
            "jobs": 14,
            "timeout_per_run_sec": 10,
            "solver_seed": 0,
            "objective_weights": {
                "alpha": 1.0,
                "beta": 1.0,
                "gamma": 1.0,
                "delta": 1.0,
            },
            "following": "allowed",
            "provenance": {
                "git_commit": "500e3a7d14425c2d1bacab33d6a1e4919c979f1d",
                "binary_sha256": MODULE.FROZEN_BINARY_SHA256,
            },
        }
        source = "<div id=\"prov\"></div>"
        zh = MODULE.inject_protocol_banner(source, timing, dict(timing), False)
        en = MODULE.inject_protocol_banner(source, timing, dict(timing), True)
        for token in (
            "同一 24 个 testcase",
            "500e3a7d14425c2d1bacab33d6a1e4919c979f1d",
            MODULE.FROZEN_BINARY_SHA256,
            "每例 10 秒",
            "14 workers",
            "solver seed 0",
            "weights (1,1,1,1)",
            "following allowed",
        ):
            self.assertIn(token, zh)
        for token in (
            "same 24 test cases",
            "same commit",
            "same reviewed binary",
            "10 seconds per case",
            "14 workers",
            "solver seed 0",
            "weights (1,1,1,1)",
            "following allowed",
        ):
            self.assertIn(token, en)

    def test_provenance_validator_rejects_different_commits(self):
        carrier = {
            "provenance": {
                "git_commit": "a" * 40,
                "binary_sha256": MODULE.FROZEN_BINARY_SHA256,
            }
        }
        baseline = {
            "provenance": {
                "git_commit": "b" * 40,
                "binary_sha256": MODULE.FROZEN_BINARY_SHA256,
            }
        }
        with self.assertRaisesRegex(ValueError, "git_commit"):
            MODULE.validate_binary_provenance(carrier, baseline)


if __name__ == "__main__":
    unittest.main()
